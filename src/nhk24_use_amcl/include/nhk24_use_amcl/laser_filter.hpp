#pragma once

#include <cmath>
#include <functional>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "std_type.hpp"
#include "vec2d.hpp"

namespace nhk24_use_amcl::stew::laser_filter::impl {
	using namespace crs_lib::integer_types;

	struct LaserFilter final : rclcpp::Node
	{
		static constexpr float footprint_size = 0.504f / 1.4142f;  // 正方形な機体の1辺の半分[m]
		static constexpr stew::vec2d::Vec2d base_to_lidar = {0.010, -0.040};


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
			sensor_msgs::msg::LaserScan filtered_msg = std::move(*msg);

			for(size_t i = 0; i < filtered_msg.ranges.size(); ++i)
			{
				filtered_msg.ranges[i] = cutoff_square(filtered_msg.ranges[i], filtered_msg.angle_min + filtered_msg.angle_increment * i);
			}
			pub->publish(filtered_msg);
		}

		static constexpr auto cutoff_square(const float r, const float theta) noexcept -> float
		{
			const auto r_x = r * std::cos(theta) - base_to_lidar.x;
			const auto r_y = r * std::sin(theta) - base_to_lidar.y;
			return r_x * r_x < footprint_size * footprint_size && r_y * r_y < footprint_size * footprint_size ? -0.0125f : r;
		}
	};
}

namespace nhk24_use_amcl::stew::laser_filter {
	using impl::LaserFilter;
}