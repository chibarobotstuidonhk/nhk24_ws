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

				auto& operator+=(const RTheta& rhs) noexcept -> RTheta&
				{
					r += rhs.r;
					theta += rhs.theta;
					return *this;
				}

				auto& operator-=(const RTheta& rhs) noexcept -> RTheta&
				{
					r -= rhs.r;
					theta -= rhs.theta;
					return *this;
				}
			};

			constexpr auto get_r_theta = [](const sensor_msgs::msg::LaserScan& msg, const u32 i) noexcept -> RTheta
			{
				return {msg.ranges[i], msg.angle_min + msg.angle_increment * i};
			};

			// remove the points inside the robot
			constexpr auto exclude_square = [](const float r, const float theta) noexcept -> bool
			{
				constexpr float footprint_size = 0.600f / 1.4142f;  // 正方形な機体の1辺の半分[m]
				constexpr stew::vec2d::Vec2d base_to_lidar = {0.010, -0.040};  // ほんとはbase_linkから取得すべき

				const auto r_x = r * std::cos(theta) - base_to_lidar.x;
				const auto r_y = r * std::sin(theta) - base_to_lidar.y;
				if(r_x * r_x < footprint_size * footprint_size && r_y * r_y < footprint_size * footprint_size) return true;

				return false;
			};
			
			constexpr auto exclude_radial = [](const RTheta& p_this, const RTheta& p_next, ) noexcept -> bool
			{
				constexpr float threshold = 0.001f;  // [m]
				constexpr float threshold_rad = 0.001f;  // [rad]

				if(r < threshold) return true;
				if(std::abs(theta) < threshold_rad) return true;

				return false;
			};

			for(size_t i = 0; i < filtered_msg.ranges.size(); ++i)
			{
				
			}
			pub->publish(filtered_msg);
		}

		// static constexpr auto cutoff(const float r, const float theta) noexcept -> float
		// {
		// 	// remove the points inside the robot
		// 	constexpr float footprint_size = 0.600f / 1.4142f;  // 正方形な機体の1辺の半分[m]
		// 	constexpr stew::vec2d::Vec2d base_to_lidar = {0.010, -0.040};

		// 	const auto r_x = r * std::cos(theta) - base_to_lidar.x;
		// 	const auto r_y = r * std::sin(theta) - base_to_lidar.y;
		// 	if(r_x * r_x < footprint_size * footprint_size && r_y * r_y < footprint_size * footprint_size) goto cut;

		// 	// remove the points aligned in radial direction
			

		// 	return r;

		// 	cut:
		// 	return -0.001f;
		// }
	};
}

namespace nhk24_use_amcl::stew::laser_filter {
	using impl::LaserFilter;
}