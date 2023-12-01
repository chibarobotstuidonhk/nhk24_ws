#pragma once

#include <compare>
#include "vec2d.hpp"

namespace nhk24_use_amcl::stew::twist2d {
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
	};
}