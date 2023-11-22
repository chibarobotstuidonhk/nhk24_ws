#pragma once

#include <cstddef>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace nhk24_use_amcl::stew::laser_filter {
	struct LaserFilter final : rclcpp::Node
	{
		static constexpr float footprint_size = 0.5f;  // 正方形な機体の1辺の半分[m]

		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;

		LaserFilter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: Node("nhk24_0th_laser_filter", options)
			, pub(create_publisher<sensor_msgs::msg::LaserScan>("scan", 10))
			, sub(create_subscription<sensor_msgs::msg::LaserScan>("scan_nonfiltered", 10, std::bind(&LaserFilter::callback, this, std::placeholders::_1)))
		{}

		private:
		void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
		{
			sensor_msgs::msg::LaserScan filtered_msg{};
			filtered_msg.header = msg->header;
			filtered_msg.angle_min = msg->angle_min;
			filtered_msg.angle_max = msg->angle_max;
			filtered_msg.angle_increment = msg->angle_increment;
			filtered_msg.time_increment = msg->time_increment;
			filtered_msg.scan_time = msg->scan_time;
			filtered_msg.range_min = msg->range_min;
			filtered_msg.range_max = msg->range_max;
			filtered_msg.ranges.reserve(msg->ranges.size());

			for(std::size_t i = 0; i < msg->ranges.size(); ++i)
			{
				filtered_msg.ranges.push_back(cutoff_square(msg->ranges[i], msg->angle_min + msg->angle_increment * i));
			}
			pub->publish(filtered_msg);
		}

		static constexpr auto cutoff_square(const float r, const float theta) noexcept -> float
		{
			const auto r_x = r * std::cos(theta);
			const auto r_y = r * std::sin(theta);
			return r_x * r_x < footprint_size * footprint_size && r_y * r_y < footprint_size * footprint_size ? 0.0f : r;
		}
	};
}