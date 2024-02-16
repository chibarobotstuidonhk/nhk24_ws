#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <nhk24_utils/msg/ball.hpp>

#include "std_type.hpp"
#include "vec3d.hpp"

namespace nhk24_utils::stew::ball::impl {
	using namespace crs_lib::integer_types;
	using vec3d::Vec3d;

	enum class BallColor : u8 {
		Purple
		, Red
		, Blue
	};
	
	struct Ball {
		Vec3d position;
		u16 id;
		BallColor color;

		static auto from_msg(const nhk24_utils::msg::Ball& msg) -> Ball {
			return Ball {
				.position = Vec3d::from_msg<geometry_msgs::msg::Point>(msg.position)
				, .id = msg.id
				, .color = static_cast<BallColor>(msg.color)
			};
		}

		auto to_msg() const -> nhk24_utils::msg::Ball {
			nhk24_utils::msg::Ball ret{};
			ret.position = position.to_msg<geometry_msgs::msg::Point>();
			ret.id = id;
			ret.color = static_cast<u8>(color);
		}
	};
}

namespace nhk24_utils::stew::ball {
	using impl::Ball;
	using impl::BallColor;
}