#pragma once

#include <cmath>
#include <limits>
#include <vector>
#include <tuple>
#include "std_type.hpp"
#include "vec2d.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

namespace nhk24_utils::stew::scan::impl {
	using namespace crs_lib::integer_types;
	using Vec2d = vec2d::Vec2d_<float>;

	constexpr float nan = std::numeric_limits<float>::quiet_NaN();

	struct RTheta final {
		float r;
		float theta;

		static auto make(const float r, const float theta) -> RTheta {
			return RTheta{r, theta};
		}

		auto is_nan() const noexcept -> bool {
			return std::isnan(this->r) || std::isnan(this->theta);
		}

		auto to_vec2d() const noexcept -> Vec2d {
			return Vec2d{this->r * std::cos(this->theta), this->r * std::sin(this->theta)};
		}
	};

	struct Scan final {
		static constexpr u16 max_length = 1440;

		std::vector<float> rs;
		float angle_min;
		float angle_max;
		float angle_increment;

		static auto from_msg(sensor_msgs::msg::LaserScan&& scan) -> std::tuple<Scan, sensor_msgs::msg::LaserScan> {
			return {
				Scan{std::move(scan.ranges), scan.angle_min, scan.angle_max, scan.angle_increment}
				, sensor_msgs::msg::builder::Init_LaserScan_header()
					.header(std::move(scan.header))
					.angle_min(std::move(scan.angle_min))
					.angle_max(std::move(scan.angle_max))
					.angle_increment(std::move(scan.angle_increment))
					.time_increment(std::move(scan.time_increment))
					.scan_time(std::move(scan.scan_time))
					.range_min(std::move(scan.range_min))
					.range_max(std::move(scan.range_max))
					.ranges(std::vector<float>())
					.intensities(std::move(scan.intensities))
			};
		}

		auto nth_rtheta(const i16 i) const noexcept -> RTheta {
			if(i < 0 || (i16)this->rs.size() <= i) return RTheta::make(nan, nan);
			else return RTheta::make(this->rs[i], this->angle_min + this->angle_increment * i);
		}

		auto nth_r(const i16 i) const noexcept -> float {
			if(i < 0 || (i16)this->rs.size() <= i) return nan;
			else return this->rs[i];
		}
	};
}

namespace nhk24_utils::stew::scan {
	using impl::RTheta;
	using impl::Scan;
	using impl::nan;
}