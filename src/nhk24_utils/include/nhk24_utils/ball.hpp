#pragma once

#include <nhk24_utils/msg/ball.hpp>

#include "std_type.hpp"
#include "vec2d.hpp"

namespace nhk24_utils::stew::ball::impl {
	using namespace crs_lib::integer_types;
	using vec2d::Vec2d;

	enum class BallColor : u8 {
		Purple
		, Red
		, Blue
	};
	
	struct Ball {
		Vec2d position;
		BallColor color;

		static auto from_msg(const nhk24_utils::msg::Ball& msg) -> Ball {
			return Ball {
				.position = Vec2d::from_msg<nhk24_utils::msg::Vec2d>(msg.position)
				, .color = static_cast<BallColor>(msg.color)
			};
		}

		auto to_msg() const -> nhk24_utils::msg::Ball {
			nhk24_utils::msg::Ball ret{};
			ret.position = position.to_msg<nhk24_utils::msg::Vec2d>();
			ret.color = static_cast<u8>(color);
		}
	};
}

namespace nhk24_utils::stew::ball {
	using impl::Ball;
	using impl::BallColor;
}