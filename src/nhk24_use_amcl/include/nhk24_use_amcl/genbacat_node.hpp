#pragma once
#include <chrono>
#include <utility>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include<tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>

#include <nhk24_utils/std_type.hpp>
#include <nhk24_utils/twist2d.hpp>
#include <nhk24_utils/geometry_msgs_convertor.hpp>

namespace nhk24_use_amcl::stew::genbacat_node::impl {
	using namespace std::chrono_literals;
	using namespace crs_lib::integer_types;
	using nhk24_utils::stew::twist2d::Twist2d;
	using nhk24_utils::stew::geometry_msgs_convertor::MsgConvertor;

	struct GenbacatNode final : rclcpp::Node {
		Twist2d current_twist{0, 0};
		std::mt19937 engine{0};
		std::normal_distribution<double> dist_xy{0.0, 0.01};
		std::normal_distribution<double> dist_th{0.0, 0.07};

		tf2_ros::TransformBroadcaster tf2_broadcaster;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr actual_cmd_vel_sub;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reset_current_twist_sub;
		rclcpp::TimerBase::SharedPtr timer;

		GenbacatNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: rclcpp::Node("genbacat_node", options)
			, tf2_broadcaster{*this}
			, actual_cmd_vel_sub{this->create_subscription<geometry_msgs::msg::Twist> (
				"actual_cmd_vel"
				, 1
				, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
					const auto vel = Twist2d::from_msg<geometry_msgs::msg::Twist>(std::move(*msg));
					this->current_twist += {rot(vel.linear, this->current_twist.angular), vel.angular};
				}
			)}
			, reset_current_twist_sub{this->create_subscription<geometry_msgs::msg::Twist> (
				"reset_current_twist"
				, 10
				, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
					this->current_twist = Twist2d::from_msg<geometry_msgs::msg::Twist>(std::move(*msg));
				}
			)}
			, timer{this->create_wall_timer (
				1ms
				, [this]() {
					geometry_msgs::msg::TransformStamped msg{};
					msg.header.frame_id = "odom";
					msg.header.stamp = this->now();
					msg.child_frame_id = "base_link";
					// msg.transform.translation.x = current_twist.linear.x + this->dist_xy(this->engine);
					// msg.transform.translation.y = current_twist.linear.y + this->dist_xy(this->engine);
					msg.transform.translation.x = 0.0 + this->dist_xy(this->engine);
					msg.transform.translation.y = 0.0 + this->dist_xy(this->engine);
					msg.transform.translation.z = 0.0;
					tf2::Quaternion q{};
					// q.setRPY(0.0, 0.0, current_twist.angular + this->dist_th(this->engine));
					q.setRPY(0.0, 0.0, 0.0 + this->dist_th(this->engine));
					msg.transform.rotation = MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::toMsg(q);
					this->tf2_broadcaster.sendTransform(std::move(msg));
			})}
		{}
	};
}

namespace nhk24_use_amcl::stew::genbacat_node {
	using impl::GenbacatNode;
}