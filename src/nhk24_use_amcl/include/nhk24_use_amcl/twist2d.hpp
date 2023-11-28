#pragma once

#include "vec2d.hpp"

namespace nhk24_use_amcl::stew::twist2d {
	struct Twist2d final {
		vec2d::Vec2d linear{0.0, 0.0};
		double angular{0.0};
	};
}