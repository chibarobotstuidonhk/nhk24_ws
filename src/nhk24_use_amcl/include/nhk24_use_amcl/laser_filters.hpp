#pragma once

#include <vector>
#include <tuple>
#include <utility>
#include "std_type.hpp"
#include "vec2d.hpp"
#include "scan.hpp"

namespace nhk24_use_amcl::stew::laser_filters::impl {
	using namespace crs_lib::integer_types;
	using Vec2d = vec2d::Vec2d_<float>;
	using Scan = scan::Scan;
	using RTheta = scan::RTheta;
	using scan::nan;

	template<class ... Filters>
	inline constexpr auto filter_chain(Filters&& ... filters) noexcept {
		return [...filters = std::forward<Filters>(filters)](const Scan& scan, const u16 i) constexpr noexcept -> float {
			float r = scan.nth_r(i);

			[&r](const Scan& scan, const u16 i, auto&& ... filters) {
				((r = filters.filter(scan, i)), ...);
			}(scan, i, std::forward<decltype(filters)>(filters) ...);
			
			return r;
		};
	}

	struct ShadowFilter final {
		float threshold_tan;
		u16 window;

		// threshold_angle: [rad], 0.0 < threshold_angle < pi/2
		static constexpr auto make(const float threshold_angle, const u16 window) noexcept -> ShadowFilter {
			return ShadowFilter{std::tan(threshold_angle), window};
		}

		auto filter(const Scan& scan, u16 i) const -> float {
			if(const RTheta r_i_ = scan.nth_rtheta(i); !r_i_.is_nan()) {
				const float r_i = r_i_.r;
				const float theta_i = r_i_.theta;

				for(u16 j = i - window; j <= i + window; j++) {
					if(const RTheta r_j_ = scan.nth_rtheta(j); !r_j_.is_nan()) {
						const float r_j = r_j_.r;
						const float theta_j = r_j_.theta;

						if(is_shadow(r_i, r_j, theta_j - theta_i)) return nan;
					}
				}
				return r_i;
			}

			return nan;
		}

		auto is_shadow(const float r, const float other_r, const float delta_theta) const noexcept -> bool {
			const float diff_x = r * std::sin(delta_theta);
			const float diff_y = other_r - r * std::cos(delta_theta);
			return std::fabs(diff_y / diff_x) < threshold_tan;
		}
	};

	struct BoxFilter final {
		Vec2d center;
		Vec2d size;

		static constexpr auto make(const Vec2d& center, const Vec2d& size) noexcept -> BoxFilter {
			return BoxFilter{center, size};
		}

		auto filter(const Scan& scan, const u16 i) const noexcept -> float {
			if(const RTheta r_i_ = scan.nth_rtheta(i); !r_i_.is_nan()) {
				const Vec2d r_i = r_i_.to_vec2d();
				const Vec2d r_i_center = r_i - this->center;
				return std::fabs(r_i_center.x) < this->size.x && std::fabs(r_i_center.y) < this->size.y ? r_i_.r : nan;
			}
			return nan;
		}
	};
}

namespace nhk24_use_amcl::stew::laser_filters {
	using impl::ShadowFilter;
	using impl::BoxFilter;
	using impl::filter_chain;
}