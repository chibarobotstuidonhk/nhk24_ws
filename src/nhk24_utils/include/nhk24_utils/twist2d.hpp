#pragma once

#include <type_traits>
#include <concepts>
#include <compare>
#include "vec2d.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nhk24_utils/msg/twist2d.hpp>

namespace nhk24_utils::stew::twist2d {
	struct Twist2d final {
		vec2d::Vec2d linear{0.0, 0.0};
		double angular{0.0};

		friend constexpr auto operator<=>(const Twist2d& lhs, const Twist2d& rhs) noexcept = default;
		friend constexpr auto operator==(const Twist2d& lhs, const Twist2d& rhs) noexcept -> bool = default;

		friend constexpr auto operator+(const Twist2d& lhs, const Twist2d& rhs) noexcept -> Twist2d {
			return Twist2d{lhs.linear + rhs.linear, lhs.angular + rhs.angular};
		}

		friend constexpr auto operator-(const Twist2d& lhs, const Twist2d& rhs) noexcept -> Twist2d {
			return Twist2d{lhs.linear - rhs.linear, lhs.angular - rhs.angular};
		}

		friend constexpr auto operator*(const Twist2d& lhs, const double rhs) noexcept -> Twist2d {
			return Twist2d{lhs.linear * rhs, lhs.angular * rhs};
		}

		friend constexpr auto operator*(const double lhs, const Twist2d& rhs) noexcept -> Twist2d {
			return rhs * lhs;
		}

		friend constexpr auto operator/(const Twist2d& lhs, const double rhs) noexcept -> Twist2d {
			return Twist2d{lhs.linear / rhs, lhs.angular / rhs};
		}

		friend constexpr auto operator/(const double lhs, const Twist2d& rhs) noexcept -> Twist2d {
			return rhs / lhs;
		}

		constexpr auto operator+=(const Twist2d& rhs) noexcept -> Twist2d& {
			linear += rhs.linear;
			angular += rhs.angular;
			return *this;
		}

		constexpr auto operator-=(const Twist2d& rhs) noexcept -> Twist2d& {
			linear -= rhs.linear;
			angular -= rhs.angular;
			return *this;
		}

		constexpr auto operator*=(const double rhs) noexcept -> Twist2d& {
			linear *= rhs;
			angular *= rhs;
			return *this;
		}

		constexpr auto operator/=(const double rhs) noexcept -> Twist2d& {
			linear /= rhs;
			angular /= rhs;
			return *this;
		}

		constexpr auto operator-() const noexcept -> Twist2d {
			return Twist2d{-linear, -angular};
		}

		template<class M>
		static auto from_msg(const std::type_identity_t<M>& msg) noexcept -> Twist2d {
			if constexpr(std::same_as<M, geometry_msgs::msg::Twist>) {
				return Twist2d{vec2d::Vec2d{msg.linear.x, msg.linear.y}, msg.angular.z};
			}
			else if constexpr(std::same_as<M, nhk24_utils::msg::Twist2d>) {
				return Twist2d{vec2d::Vec2d{msg.linear.x, msg.linear.y}, msg.angular};
			}
			else {
				static_assert([]{return false;}(), "invalid message type");
			}
		}

		template<class M>
		auto to_msg() const noexcept -> M {
			if constexpr(std::same_as<M, geometry_msgs::msg::Twist>) {
				M msg{};
				msg.linear.x = linear.x;
				msg.linear.y = linear.y;
				msg.angular.z = angular;
				return msg;
			}
			else if constexpr(std::same_as<M, nhk24_utils::msg::Twist2d>) {
				M msg{};
				msg.linear.x = linear.x;
				msg.linear.y = linear.y;
				msg.angular = angular;
				return msg;
			}
			else {
				static_assert([]{return false;}(), "invalid message type");
			}
		}
	};
}