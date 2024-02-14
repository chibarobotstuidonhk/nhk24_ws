#pragma once

#include <optional>
#include <vector>
#include <utility>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nhk24_utils/msg/balls.hpp>
#include <nhk24_utils/msg/twist2d.hpp>

#include <nhk24_utils/std_type.hpp>
#include <nhk24_utils/pid.hpp>
#include <nhk24_utils/vec2d.hpp>
#include <nhk24_utils/twist2d.hpp>
#include <nhk24_utils/geometry_msgs_convertor.hpp>
#include <nhk24_utils/ball.hpp>

namespace nhk24_use_amcl::stew::ball_chaser::impl {
	using namespace crs_lib::integer_types;
	using nhk24_utils::stew::pid::Pid;
	using nhk24_utils::stew::vec2d::Vec2d;
	using nhk24_utils::stew::twist2d::Twist2d;
	using nhk24_utils::stew::geometry_msgs_convertor::MsgConvertor;
	using nhk24_utils::stew::ball::Ball;

	struct BallChaser final : rclcpp::Node {
		std::vector<Ball> balls{};
		Pid<Vec2d, double> pid_xy{};
		Pid<double, double> pid_th{};
		double distance_to_ball{};
		double angle_to_ball{};

		tf2_ros::Buffer tf2_buffer;
		tf2_ros::TransformListener tf2_listener;
		
		rclcpp::Publisher<nhk24_utils::msg::Twist2d>::SharedPtr body_twist_pub;
		rclcpp::Subscription<nhk24_utils::msg::Balls>::SharedPtr ball_pos_sub;
		rclcpp::TimerBase::SharedPtr timer;

		BallChaser(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: Node("nhk24_1st_ball_chaser", options)
			, tf2_buffer{this->get_clock()}
			, tf2_listener{tf2_buffer}
			, body_twist_pub {
				this->create_publisher<nhk24_utils::msg::Twist2d>("body_twist", 1)
			}
			, ball_pos_sub {
				this->create_subscription<nhk24_utils::msg::Balls> (
					"ball_pos"
					, 1
					, [this](const nhk24_utils::msg::Balls::SharedPtr msg) {
						std::vector<Ball> balls(msg->balls.size());
						for(size_t i = 0; i < balls.size(); ++i) {
							balls[i] = Ball::from_msg(msg->balls[i]);
						}
						this->balls = std::move(balls);
					}
				)
			}
			, timer {
				this->create_wall_timer (
					std::chrono::milliseconds(100)
					, std::bind(&BallChaser::timer_callback, this)
				)
			}
		{
			this->declare_parameter("pid_xy_p", 1.0);
			this->declare_parameter("pid_xy_i", 0.0);
			this->declare_parameter("pid_xy_d", 0.0);
			this->declare_parameter("pid_xy_max_integral_x", 0.0);
			this->declare_parameter("pid_xy_max_integral_y", 0.0);
			
			this->declare_parameter("pid_th_p", 1.0);
			this->declare_parameter("pid_th_i", 0.0);
			this->declare_parameter("pid_th_d", 0.0);
			this->declare_parameter("pid_th_max_integral", 0.0);

			this->declare_parameter("distance_to_ball", 1.0);
			this->declare_parameter("angle_to_ball", 0.1508);

			this->update_params();
		}

		void timer_callback() {
			const auto current_pose = this->get_current_pose();
			const auto& balls = this->balls;

			if(!current_pose || balls.empty()) {
				return;
			}

			const auto target = this->target_pose();

			const auto error = target - *current_pose;
			const auto cmd_vel = Twist2d {
				this->pid_xy.update(error.linear, 0.1)
				, this->pid_th.update(error.angular, 0.1)
			};

			this->body_twist_pub->publish(cmd_vel.to_msg<nhk24_utils::msg::Twist2d>());
		}

		/// @todo: implement the logic to choose a ball and best pose to pick it up
		auto target_pose() -> Twist2d {
			return Twist2d{0.0, 0.0, 0.0};
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
				RCLCPP_ERROR(this->get_logger(), "ball_chaser:  %s", e.what());
				return std::nullopt;
			}
		}

		void update_params() {
			{
				const auto p = this->get_parameter("pid_xy_p").as_double();
				const auto i = this->get_parameter("pid_xy_i").as_double();
				const auto d = this->get_parameter("pid_xy_d").as_double();
				const auto max_integral = Vec2d {
					this->get_parameter("pid_xy_max_integral_x").as_double()
					, this->get_parameter("pid_xy_max_integral_y").as_double()
				};
				this->pid_xy = Pid<Vec2d, double>::make(p, i, d, max_integral);
			}
			{
				const auto p = this->get_parameter("pid_th_p").as_double();
				const auto i = this->get_parameter("pid_th_i").as_double();
				const auto d = this->get_parameter("pid_th_d").as_double();
				const auto max_integral = this->get_parameter("pid_th_max_integral").as_double();
				this->pid_th = Pid<double, double>::make(p, i, d, max_integral);
			}
			{
				this->distance_to_ball = this->get_parameter("distance_to_ball").as_double();
				this->angle_to_ball = this->get_parameter("angle_to_ball").as_double();
			}
		}
	};
}

namespace nhk24_use_amcl::stew::ball_chaser {
	using impl::BallChaser;
}