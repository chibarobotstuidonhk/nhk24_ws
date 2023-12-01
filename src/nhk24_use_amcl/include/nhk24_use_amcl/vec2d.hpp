#pragma once

#include <cmath>
#include <compare>

namespace nhk24_use_amcl::stew::vec2d {
	struct Vec2d final {
		double x{0.0};
		double y{0.0};

		friend constexpr auto operator<=>(const Vec2d& lhs, const Vec2d& rhs) noexcept = default;
		friend constexpr auto operator==(const Vec2d& lhs, const Vec2d& rhs) noexcept -> bool = default;

		friend constexpr auto operator+(const Vec2d& lhs, const Vec2d& rhs) noexcept -> Vec2d {
			return Vec2d{lhs.x + rhs.x, lhs.y + rhs.y};
		}

		friend constexpr auto operator-(const Vec2d& lhs, const Vec2d& rhs) noexcept -> Vec2d {
			return Vec2d{lhs.x - rhs.x, lhs.y - rhs.y};
		}

		friend constexpr auto operator*(const Vec2d& lhs, const double rhs) noexcept -> Vec2d {
			return Vec2d{lhs.x * rhs, lhs.y * rhs};
		}

		friend constexpr auto operator*(const double lhs, const Vec2d& rhs) noexcept -> Vec2d {
			return rhs * lhs;
		}

		friend constexpr auto operator/(const Vec2d& lhs, const double rhs) noexcept -> Vec2d {
			return Vec2d{lhs.x / rhs, lhs.y / rhs};
		}

		friend constexpr auto operator/(const double lhs, const Vec2d& rhs) noexcept -> Vec2d {
			return rhs / lhs;
		}

		friend constexpr auto dot(const Vec2d& lhs, const Vec2d& rhs) noexcept -> double {
			return lhs.x * rhs.x + lhs.y * rhs.y;
		}
		
		friend constexpr auto rot(const Vec2d& vec, const double theta) noexcept -> Vec2d {
			return Vec2d{vec.x * std::cos(theta) - vec.y * std::sin(theta), vec.x * std::sin(theta) + vec.y * std::cos(theta)};
		}

		constexpr auto operator+=(const Vec2d& rhs) noexcept -> Vec2d& {
			x += rhs.x;
			y += rhs.y;
			return *this;
		}

		constexpr auto operator-=(const Vec2d& rhs) noexcept -> Vec2d& {
			x -= rhs.x;
			y -= rhs.y;
			return *this;
		}

		constexpr auto operator*=(const double rhs) noexcept -> Vec2d& {
			x *= rhs;
			y *= rhs;
			return *this;
		}

		constexpr auto operator/=(const double rhs) noexcept -> Vec2d& {
			x /= rhs;
			y /= rhs;
			return *this;
		}

		constexpr auto operator-() const noexcept -> Vec2d {
			return Vec2d{-x, -y};
		}
	
		constexpr auto norm2() const noexcept -> double {
			return x * x + y * y;
		}

		constexpr auto unitize() const noexcept -> Vec2d {
			return *this * (1.0 / std::sqrt(norm2()));
		}
	};
}