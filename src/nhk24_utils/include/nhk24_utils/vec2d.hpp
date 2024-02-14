#pragma once

#include <cmath>
#include <type_traits>
#include <concepts>
#include <compare>

#include <nhk24_utils/msg/vec2d.hpp>

namespace nhk24_utils::stew::vec2d {
	template<class T>
	struct Vec2d_ final {
		T x{0.0};
		T y{0.0};

		friend constexpr auto operator<=>(const Vec2d_& lhs, const Vec2d_& rhs) noexcept = default;
		friend constexpr auto operator==(const Vec2d_& lhs, const Vec2d_& rhs) noexcept -> bool = default;

		friend constexpr auto operator+(const Vec2d_& lhs, const Vec2d_& rhs) noexcept -> Vec2d_ {
			return Vec2d_{lhs.x + rhs.x, lhs.y + rhs.y};
		}

		friend constexpr auto operator-(const Vec2d_& lhs, const Vec2d_& rhs) noexcept -> Vec2d_ {
			return Vec2d_{lhs.x - rhs.x, lhs.y - rhs.y};
		}

		friend constexpr auto operator*(const Vec2d_& lhs, const T rhs) noexcept -> Vec2d_ {
			return Vec2d_{lhs.x * rhs, lhs.y * rhs};
		}

		friend constexpr auto operator*(const T lhs, const Vec2d_& rhs) noexcept -> Vec2d_ {
			return rhs * lhs;
		}

		friend constexpr auto operator/(const Vec2d_& lhs, const T rhs) noexcept -> Vec2d_ {
			return Vec2d_{lhs.x / rhs, lhs.y / rhs};
		}

		friend constexpr auto operator/(const T lhs, const Vec2d_& rhs) noexcept -> Vec2d_ {
			return rhs / lhs;
		}

		friend constexpr auto dot(const Vec2d_& lhs, const Vec2d_& rhs) noexcept -> T {
			return lhs.x * rhs.x + lhs.y * rhs.y;
		}
		
		friend constexpr auto rot(const Vec2d_& vec, const T theta) noexcept -> Vec2d_ {
			return Vec2d_{vec.x * std::cos(theta) - vec.y * std::sin(theta), vec.x * std::sin(theta) + vec.y * std::cos(theta)};
		}

		constexpr auto operator+=(const Vec2d_& rhs) noexcept -> Vec2d_& {
			x += rhs.x;
			y += rhs.y;
			return *this;
		}

		constexpr auto operator-=(const Vec2d_& rhs) noexcept -> Vec2d_& {
			x -= rhs.x;
			y -= rhs.y;
			return *this;
		}

		constexpr auto operator*=(const T rhs) noexcept -> Vec2d_& {
			x *= rhs;
			y *= rhs;
			return *this;
		}

		constexpr auto operator/=(const T rhs) noexcept -> Vec2d_& {
			x /= rhs;
			y /= rhs;
			return *this;
		}

		constexpr auto operator-() const noexcept -> Vec2d_ {
			return Vec2d_{-x, -y};
		}
	
		constexpr auto norm2() const noexcept -> T {
			return x * x + y * y;
		}

		constexpr auto unitize() const noexcept -> Vec2d_ {
			return *this * (1.0 / std::sqrt(norm2()));
		}

		template<class M>
		static auto from_msg(const std::type_identity_t<M>& msg) noexcept -> Vec2d_ {
			if constexpr(std::same_as<M, nhk24_utils::msg::Vec2d>) {
				return Vec2d_{msg.x, msg.y};
			}
			else {
				static_assert([]{return false;}(), "invalid messge type");
			}
		}

		template<class M>
		auto to_msg() const noexcept -> std::type_identity_t<M> {
			if constexpr(std::same_as<M, nhk24_utils::msg::Vec2d>) {
				nhk24_utils::msg::Vec2d msg{};
				msg.x = x;
				msg.y = y;
				return msg;
			}
			else {
				static_assert([]{return false;}(), "invalid messge type");
			}
		}
	};

	using Vec2d = Vec2d_<double>;
}