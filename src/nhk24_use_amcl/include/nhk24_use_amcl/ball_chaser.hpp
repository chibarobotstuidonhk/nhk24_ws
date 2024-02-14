#pragma once

#include <optional>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nhk24_utils/msg/vec2d.hpp>

#include <nhk24_utils/pid.hpp>
#include <nhk24_utils/twist2d.hpp>
#include <nhk24_utils/geometry_msgs_convertor.hpp>

namespace nhk24_use_amcl::stew::ball_chaser::impl {
	using nhk24_utils::stew::pid::Pid;
	using nhk24_utils::stew::twist2d::Twist2d;

	struct BallChaser final : rclcpp::Node {
		Twist2d ball_pos{};
		Pid<Twist2d, double> pid;

		tf2_ros::Buffer tf2_buffer;
		tf2_ros::TransformListener tf2_listener;
		
		rclcpp::Subscription<nhk24_utils::msg::Vec2d>::SharedPtr ball_pos_sub;
		rclcpp::TimerBase::SharedPtr timer;

		BallChaser(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: Node("nhk24_1st_ball_chaser", options)
			, pid{1.0, 0.0, 0.0}
			, tf2_buffer{this->get_clock()}
			, tf2_listener{tf2_buffer}
			, ball_pos_sub {
				create_subscription<nhk24_utils::msg::Twist2d> (
					"ball_pos"
					, 1
					, [this](const nhk24_utils::msg::Twist2d::SharedPtr& msg) {
						this->ball_pos = Twist2d::from_msg<nhk24_utils::msg::Twist2d>(std::move(*msg));
					}
				)
			}
		{}

		auto get_current_pose() const -> std::optional<Twist2d> {
			try {
				const auto transform = tf2_buffer.lookupTransform("map", "base_link", tf2::TimePointZero).transform;
				const auto v = transform.translation;
				double roll, pitch, yaw;
				tf2::Matrix3x3{geometry_msgs_convertor::MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::fromMsg(transform.rotation)}
					.getRPY(roll, pitch, yaw);
				return Twist2d{v.x, v.y, yaw};
			
			} catch(const tf2::TransformException& e) {
				RCLCPP_ERROR(this->get_logger(), "ball_chaser:  %s", e.what());
				return std::nullopt;
			}
		}
	};
}