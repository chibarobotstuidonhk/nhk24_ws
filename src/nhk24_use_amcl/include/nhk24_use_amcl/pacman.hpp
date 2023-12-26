#pragma once

#include <vector>
#include <optional>
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

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nhk24_use_amcl/msg/path.hpp>
#include <nhk24_use_amcl/msg/result.hpp>

#include "std_type.hpp"
#include "vec2d.hpp"
#include "twist2d.hpp"
#include "pid.hpp"
#include "geometry_msgs_convertor.hpp"

namespace nhk24_use_amcl::stew::pacman::impl {
	using namespace std::chrono_literals;
	using namespace crs_lib::integer_types;
	using vec2d::Vec2d;
	using twist2d::Twist2d;
	using geometry_msgs_convertor::MsgConvertor;

	struct PacMan final : rclcpp::Node {
		private:
		std::optional<msg::Path> path;
		pid::Pid<Twist2d, double> pid_controller;
		uint32_t lookahead;
		uint32_t lookback;
		uint32_t step;
		u32 current_index;

		tf2_ros::Buffer tf2_buffer;
		tf2_ros::TransformListener tf2_listener;

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
		rclcpp::Publisher<nhk24_use_amcl::msg::Result>::SharedPtr result_pub;
		rclcpp::Subscription<msg::Path>::SharedPtr path_sub;
		rclcpp::TimerBase::SharedPtr timer;

		public:
		PacMan(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: rclcpp::Node("pacman", options)
			, path(std::nullopt)
			, pid_controller{pid::Pid<Twist2d, double>::make(1.0, 0.0, 0.0)}
			, lookahead{10}
			, lookback{10}
			, step{5}
			, current_index{0}
			, tf2_buffer{this->get_clock()}
			, tf2_listener{tf2_buffer}
			, cmd_vel_pub(this->create_publisher<geometry_msgs::msg::Twist>("body_twist", 1))
			, result_pub(this->create_publisher<nhk24_use_amcl::msg::Result>("result", 10))
			, path_sub(this->create_subscription<msg::Path>("path", 10, std::bind(&PacMan::path_callback, this, std::placeholders::_1)))
			, timer(this->create_wall_timer(1ms, std::bind(&PacMan::timer_callback, this)))
		{
			this->declare_parameter<double>("k_p", pid_controller.k_p);
			this->declare_parameter<double>("k_i", pid_controller.k_i);
			this->declare_parameter<double>("k_d", pid_controller.k_d);
			this->declare_parameter<int>("lookahead", lookahead);
			this->declare_parameter<int>("lookback", lookback);
			this->declare_parameter<int>("step", step);
		}

		private:
		void refresh_parameters() {
			pid_controller = pid::Pid<Twist2d, double>::make(
				this->get_parameter("k_p").as_double(),
				this->get_parameter("k_i").as_double(),
				this->get_parameter("k_d").as_double()
			);
			lookahead = this->get_parameter("lookahead").as_int();
			lookback = this->get_parameter("lookback").as_int();
			step = this->get_parameter("step").as_int();
		}

		void path_callback(const msg::Path::SharedPtr msg) {
			if(path.has_value()) {
				msg::Result result{};
				result.time = std::move(path->time);
				result.is_goal_reached = false;
				result_pub->publish(result);
			}

			if(msg->path.size() == 0) {
				RCLCPP_ERROR(this->get_logger(), "empty path has been subscribed.");
				path = std::nullopt;
			}

			// no matter whether path is empty or not, refresh parameters on every path_callback
			refresh_parameters();

			path = std::move(*msg);
			current_index = 0;
		}

		// void mcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
		// 	using Message = geometry_msgs::msg::PoseWithCovarianceStamped;
		// 	using Out = tf2::WithCovarianceStamped<tf2::Transform>;
			
		// 	Out transform = MsgConvertor<Out, Message>::fromMsg(std::move(*msg));
		// 	const auto v = transform.getOrigin();
		// 	double roll, pitch, yaw;
		// 	tf2::Matrix3x3{transform.getRotation()}.getRPY(roll, pitch, yaw);
			
		// 	current_pose = Twist2d{v.x(), v.y(), yaw};
		// }

		void timer_callback() {
			constexpr double dt = 0.001;
			constexpr auto take_subrange = [](size_t begin, size_t end) {
				return std::ranges::views::drop(begin) | std::ranges::views::take(end - begin);
			};

			if(!path.has_value()) return;

			const auto& [time, path, goal_raius] = *this->path;
			if(const auto current_pose_ = get_current_pose(); current_pose_) {
				const auto current_pose = *current_pose_;

				// get target_pose
				auto target_pose = Twist2d::from_msg<nhk24_use_amcl::msg::Twist2d>(path[std::min<u32>(current_index + step, path.size() - 1)]);

				// update current_index to the most closest point in looking subrange
				constexpr auto cast_to_twist2d = std::ranges::views::transform(Twist2d::from_msg<nhk24_use_amcl::msg::Twist2d>);
				const u32 begin = std::max((i32)current_index - (i32)lookback, 0);
				const u32 end = std::min<u32>(current_index + lookahead, path.size());

				u32 most_closest_index = begin;
				double most_closest_distance = std::numeric_limits<double>::max();
				for(size_t i = 0; const Twist2d& pose : path | take_subrange(begin, end) | cast_to_twist2d) {
					const double distance = (pose.linear - current_pose.linear).norm2();
					if(distance < most_closest_distance) {
						most_closest_index = i;
						most_closest_distance = distance;
					}

					++i;
				}
				
				// update current_index
				current_index = most_closest_index;

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
				tf2::Matrix3x3{geometry_msgs_convertor::MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::fromMsg(transform.rotation)}
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