#pragma once
#include <chrono>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include<tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>

#include "std_type.hpp"
#include "twist2d.hpp"
#include "geometry_msgs_convertor.hpp"

namespace nhk24_use_amcl::stew::genbacat_node::impl {
	using namespace std::chrono_literals;
	using namespace crs_lib::integer_types;
	using twist2d::Twist2d;

	struct GenbacatNode final : rclcpp::Node {
		Twist2d current_twist{0, 0};

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
					msg.transform.translation.x = current_twist.linear.x;
					msg.transform.translation.y = current_twist.linear.y;
					msg.transform.translation.z = 0.0;
					tf2::Quaternion q{};
					q.setEuler(current_twist.angular, 0.0, 0.0);
					msg.transform.rotation = geometry_msgs_convertor::MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::toMsg(q);
					this->tf2_broadcaster.sendTransform(std::move(msg));
			})}
		{}
	};
}

namespace nhk24_use_amcl::stew::genbacat_node {
	using impl::GenbacatNode;
}