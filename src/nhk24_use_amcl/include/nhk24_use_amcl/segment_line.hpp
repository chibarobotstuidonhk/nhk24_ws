#pragma once

#include <vector>
#include <utility>
#include <nhk24_utils/std_type.hpp>
#include <nhk24_utils/vec2d.hpp>
#include <nhk24_utils/scan.hpp>

namespace nhk24_use_amcl::stew::segment_line::impl {
	using namespace crs_lib::integer_types;
	using Vec2d = nhk24_utils::stew::vec2d::Vec2d_<float>;
	using nhk24_utils::stew::scan::Scan;
	using nhk24_utils::stew::scan::RTheta;

	struct SegmentLine final {
		std::vector<u16> indices;
		Vec2d accumulated;
		u16 count;
		u16 window;
		float threshold;

		static auto make(const u16 window, const float threshold) -> SegmentLine {
			std::vector<u16> tmp{};
			tmp.reserve(Scan::max_length);
			return SegmentLine{std::move(tmp), Vec2d{0.0f, 0.0f}, 0, window, threshold};
		}

		auto update(const Scan& scan, const u16 i) && -> SegmentLine {
			auto self = std::move(*this);

			if(const RTheta r_i_ = scan.nth_rtheta(i); !r_i_.is_nan()) {
				const Vec2d r_i = r_i_.to_vec2d(); 
			
				if(const RTheta rth = scan.nth_rtheta(i - window - 1); !rth.is_nan()) {
					self.accumulated -= rth.to_vec2d();
					self.count--;
				}
				if(const RTheta rth = scan.nth_rtheta(i + window); !rth.is_nan()) {
					self.accumulated += rth.to_vec2d();
					self.count++;
				}

				if(((self.accumulated / self.count - r_i) / r_i_.r).norm2() > self.threshold * self.threshold) {
					self.indices.push_back(i);
				}
			}

			return self;
		}
	};
}

namespace nhk24_use_amcl::stew::segment_line {
	using impl::SegmentLine;
}