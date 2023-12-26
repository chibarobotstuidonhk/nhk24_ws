#pragma once

#include <rclcpp/rclcpp.hpp>
#include<tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>

#include "std_type.hpp"
#include "twist2d.hpp"

namespace nhk24_use_amcl::stew::genbacat_node::impl {
	using namespace crs_lib::integer_types;
	using twist2d::Twist2d;

	struct GenbacatNode final : rclcpp::Node {
		Twist2d current_twist{0, 0};

		tf2_ros::TransformBroadcaster tf2_broadcaster;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr actual_cmd_vel_sub;
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reset_current_twist_sub;

		GenbacatNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: rclcpp::Node("genbacat_node", options)
			, tf2_broadcaster{*this}
			, actual_cmd_vel_sub{create_subscription<geometry_msgs::msg::Twist> (
				"actual_cmd_vel"
				, 1
				, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
					const auto vel = Twist2d::from_msg<geometry_msgs::msg::Twist>(std::move(*msg));
					this->current_twist += {rot(vel.linear, this->current_twist.angular), vel.angular};
				}
			)}
			, reset_current_twist_sub{create_subscription<geometry_msgs::msg::Twist> (
				"reset_current_twist"
				, 10
				, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
					this->current_twist = Twist2d::from_msg<geometry_msgs::msg::Twist>(std::move(*msg));
				}
			)}
		{}
	};
}

namespace nhk24_use_amcl::stew::genbacat_node {
	using impl::GenbacatNode;
}