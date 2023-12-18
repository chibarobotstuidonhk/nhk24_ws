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

			struct RTheta final {
				float r;
				float theta;

				constexpr auto to_xy() const noexcept -> stew::vec2d::Vec2d
				{
					return {r * std::cos(theta), r * std::sin(theta)};
				}
			};

			constexpr auto get_r_theta = [](const sensor_msgs::msg::LaserScan& msg, const u32 i) noexcept -> RTheta
			{
				return {msg.ranges[i], msg.angle_min + msg.angle_increment * i};
			};

			// remove the points inside the robot
			constexpr auto exclude_square = [](const RTheta& p) noexcept -> bool
			{
				constexpr float footprint_size = 0.700f / 1.4142f;  // 正方形な機体の1辺の半分[m]
				constexpr stew::vec2d::Vec2d base_to_lidar = {0.010, -0.040};  // ほんとはbase_linkから取得すべき

				const auto [r_x, r_y] = p.to_xy() - base_to_lidar;
				if(r_x * r_x < footprint_size * footprint_size && r_y * r_y < footprint_size * footprint_size) return true;

				return false;
			};
			
			constexpr auto exclude_radial_gap = [](const RTheta& p, const RTheta& p_last) noexcept -> bool
			{
				constexpr float threshold = 0.100f;

				const auto gap = (p.r - p_last.r) * 2 / (p.r + p_last.r);
				return gap * gap > threshold * threshold;
			};

			RTheta p_last = get_r_theta(filtered_msg, 0);
			for(size_t i = 0; i < filtered_msg.ranges.size(); ++i)
			{
				const auto p = get_r_theta(filtered_msg, i);
				if(exclude_square(p) || exclude_radial_gap(p, p_last))
				{
					// filtered_msg.ranges[i] = std::nanf("");
					filtered_msg.ranges[i] = 0.0f;
				}
				p_last = p;
			}

			pub->publish(filtered_msg);
		}
	};
}

namespace nhk24_use_amcl::stew::laser_filter {
	using impl::LaserFilter;
}