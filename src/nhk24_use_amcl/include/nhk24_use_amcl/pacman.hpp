/**
 * @file pacman.hpp
 * @brief 軌道追従を行う。
 * 
 * 設計が悪い。ros2のparameterにしている部分はmsg::Pathに含めてしまうべきだった。
 * また、追従と停止しかしないにせよ、状態をvariantで表現するべきだった。
 * enum State{PathTracing(msg::Path), Stopped(Twist2d)}; みたいに。
 */

#pragma once

#include <vector>
#include <optional>
#include <variant>
#include <functional>
#include <utility>
#include <chrono>
#include <ranges>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nhk24_utils/msg/path.hpp>
#include <nhk24_utils/msg/result.hpp>

#include <nhk24_utils/std_type.hpp>
#include <nhk24_utils/vec2d.hpp>
#include <nhk24_utils/twist2d.hpp>
#include <nhk24_utils/pid.hpp>
#include <nhk24_utils/geometry_msgs_convertor.hpp>

namespace nhk24_use_amcl::stew::pacman::impl {
	using namespace std::chrono_literals;
	using namespace crs_lib::integer_types;
	using nhk24_utils::stew::vec2d::Vec2d;
	using nhk24_utils::stew::twist2d::Twist2d;
	using nhk24_utils::stew::geometry_msgs_convertor::MsgConvertor;
	using nhk24_utils::stew::pid::Pid;

	template<class T_>
	struct Averaging {
		T_ sum;
		std::vector<std::optional<T_>> buf;
		size_t len;
		size_t head;


		static auto make(const size_t n, const T_ ide) -> Averaging {
			return Averaging{ide, std::vector<std::optional<T_>>(n), 0, 0};
		}

		auto update(T_&& latest, auto logger) -> T_ {
			sum += latest;
			if(const auto index = (head + len) % buf.size(); buf[index]) {
				sum -= *buf[index];
				buf[index] = std::make_optional(std::move(latest));
				++head;
			}
			else {
				buf[index] = std::make_optional(std::move(latest));
				++len;
			}

			return sum / len;
		}
	};

	struct PacMan final : rclcpp::Node {
		private:
		std::variant<std::monostate, nhk24_utils::msg::Path, Twist2d> path;
		Pid<Twist2d, double> pid_controller;
		Averaging<Twist2d> current_pose_averaging;
		uint32_t lookahead;
		uint32_t lookback;
		uint32_t step;
		u32 current_index;

		tf2_ros::Buffer tf2_buffer;
		tf2_ros::TransformListener tf2_listener;
		tf2_ros::TransformBroadcaster tf2_broadcaster;

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
		rclcpp::Publisher<nhk24_utils::msg::Result>::SharedPtr result_pub;
		rclcpp::Subscription<nhk24_utils::msg::Path>::SharedPtr path_sub;
		rclcpp::TimerBase::SharedPtr timer;

		public:
		PacMan(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: rclcpp::Node("pacman", options)
			, path()
			, pid_controller{Pid<Twist2d, double>::make(1.0, 0.0, 0.0, {{3.0, 3.0}, 1.0})}
			, current_pose_averaging{Averaging<Twist2d>::make(20, Twist2d{{0.0, 0.0}, 0.0})}
			, lookahead{1}
			, lookback{0}
			, step{1}
			, current_index{0}
			, tf2_buffer{this->get_clock()}
			, tf2_listener{tf2_buffer}
			, tf2_broadcaster{*this}
			, cmd_vel_pub(this->create_publisher<geometry_msgs::msg::Twist>("body_twist", 1))
			, result_pub(this->create_publisher<nhk24_utils::msg::Result>("result", 10))
			, path_sub(this->create_subscription<nhk24_utils::msg::Path>("path", 10, std::bind(&PacMan::path_callback, this, std::placeholders::_1)))
			, timer(this->create_wall_timer(10ms, std::bind(&PacMan::timer_callback, this)))
		{
			this->declare_parameter<double>("k_p", pid_controller.k_p);
			this->declare_parameter<double>("k_i", pid_controller.k_i);
			this->declare_parameter<double>("k_d", pid_controller.k_d);
			this->declare_parameter<double>("max_integral_xy", pid_controller.max_integral.linear.x);
			this->declare_parameter<double>("max_integral_th", pid_controller.max_integral.angular);
			this->declare_parameter<int>("lookahead", lookahead);
			this->declare_parameter<int>("lookback", lookback);
			this->declare_parameter<int>("step", step);
		}

		private:
		void refresh_parameters() {
			pid_controller = Pid<Twist2d, double>::make(
				this->get_parameter("k_p").as_double(),
				this->get_parameter("k_i").as_double(),
				this->get_parameter("k_d").as_double(),
				Twist2d {
					Vec2d {
						this->get_parameter("max_integral_xy").as_double(),
						this->get_parameter("max_integral_xy").as_double()
					},
					this->get_parameter("max_integral_th").as_double()
				}
			);
			lookahead = this->get_parameter("lookahead").as_int();
			lookback = this->get_parameter("lookback").as_int();
			step = this->get_parameter("step").as_int();
		}

		void path_callback(const nhk24_utils::msg::Path::SharedPtr msg) {
			if(const auto path_p = std::get_if<nhk24_utils::msg::Path>(&path); path_p) {
				nhk24_utils::msg::Result result{};
				result.time = std::move(path_p->time);
				result.is_goal_reached = false;
				result_pub->publish(result);
			}

			if(msg->path.size() == 0) {
				RCLCPP_ERROR(this->get_logger(), "empty path has been subscribed.");
				path = std::monostate{};
			}

			// no matter whether path is empty or not, refresh parameters on every path_callback
			refresh_parameters();

			path = std::move(*msg);
			current_index = 0;
		}

		void timer_callback() {
			constexpr double dt = 0.001;
			constexpr auto take_subrange = [](size_t begin, size_t end) {
				return std::ranges::views::drop(begin) | std::ranges::views::take(end - begin);
			};

			if(const auto current_pose_ = get_current_pose(); current_pose_) {
				const auto current_pose = current_pose_averaging.update(Twist2d{*current_pose_}, this->get_logger());
				{
					geometry_msgs::msg::TransformStamped msg{};
					msg.header.frame_id = "map";
					msg.child_frame_id = "base_link2";
					tf2::Quaternion q{};
					q.setEuler(current_pose.angular, 0, 0);
					msg.transform.rotation = MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::toMsg(q);
					msg.transform.translation.x = current_pose.linear.x;
					msg.transform.translation.y = current_pose.linear.y;
					msg.transform.translation.z = 0;
					tf2_broadcaster.sendTransform(msg);
				}

				RCLCPP_INFO_STREAM(this->get_logger(), current_pose.linear.x << " " << current_pose.linear.y << " " << current_pose.angular);

				Twist2d target_pose = {};
				if(std::get_if<std::monostate>(&path)) return;
				else if(const auto twist_p = std::get_if<Twist2d>(&path); twist_p) {
					target_pose = *twist_p;
				}
				else {
					RCLCPP_INFO_STREAM(this->get_logger(), "current_index: " << current_index);
					const auto& [time, path, goal_raius] = std::get<nhk24_utils::msg::Path>(this->path);
					
					// get target_pose
					target_pose = Twist2d::from_msg<nhk24_utils::msg::Twist2d>(path[std::min<u32>(current_index + step, path.size() - 1)]);

					// update current_index to the most closest point in looking subrange
					constexpr auto cast_to_twist2d = std::ranges::views::transform(Twist2d::from_msg<nhk24_utils::msg::Twist2d>);
					const u32 begin = std::max((i32)current_index - (i32)lookback, 0);
					const u32 end = std::min<u32>(current_index + lookahead + 1, path.size());

					u32 most_closest_index = begin;
					double most_closest_distance = std::numeric_limits<double>::max();
					for(size_t i = begin; const Twist2d& pose : path | take_subrange(begin, end) | cast_to_twist2d) {
						const double distance = (pose.linear - current_pose.linear).norm2();
						if(distance < most_closest_distance) {
							most_closest_index = i;
							most_closest_distance = distance;
						}

						++i;
					}
					
					// update current_index
					current_index = most_closest_index;

					// if((current_pose.linear - Vec2d::from_msg<nhk24_utils::msg::Vec2d>(path[0].linear)).norm2() < goal_raius * goal_raius) {
					if(current_index == path.size() - 1) {
						nhk24_utils::msg::Result result{};
						result.time = time;
						result.is_goal_reached = true;
						result_pub->publish(result);

						this->path = Twist2d::from_msg<nhk24_utils::msg::Twist2d>(path.back());
						
						RCLCPP_INFO_STREAM(this->get_logger(), "GOAL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
						return;
					}
				}

				// calculate target twist
				const Twist2d target_twist = pid_controller.update(target_pose - current_pose, dt);
				// convert target_twist to local one
				const auto [local_x, local_y] = rot(target_twist.linear, -current_pose.angular);
				
				// publish cmd_vel
				geometry_msgs::msg::Twist msg{};
				msg.linear.x = local_x;
				msg.linear.y = local_y;
				msg.angular.z = target_twist.angular;
				cmd_vel_pub->publish(msg);
			}
		}

		auto get_current_pose() const -> std::optional<Twist2d> {
			try {
				const auto transform = tf2_buffer.lookupTransform("map", "base_link", tf2::TimePointZero).transform;
				const auto v = transform.translation;
				double roll, pitch, yaw;
				tf2::Matrix3x3{MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::fromMsg(transform.rotation)}
					.getRPY(roll, pitch, yaw);
				return Twist2d{v.x, v.y, yaw};
			
			} catch(const tf2::TransformException& e) {
				RCLCPP_ERROR(this->get_logger(), "pacman:  %s", e.what());
				return std::nullopt;
			}
		}
	};
}

namespace nhk24_use_amcl::stew::pacman {
	using impl::PacMan;
}