#pragma once

#include <cmath>
#include <type_traits>
#include <concepts>
#include <compare>

#include <geometry_msgs/msg/point.hpp>

namespace nhk24_utils::stew::vec3d {
	template<class T>
	struct Vec3d_ final {
		T x{0.0};
		T y{0.0};
		T z{0.0};

		friend constexpr auto operator<=>(const Vec3d_& lhs, const Vec3d_& rhs) noexcept = default;
		friend constexpr auto operator==(const Vec3d_& lhs, const Vec3d_& rhs) noexcept -> bool = default;

		friend constexpr auto operator+(const Vec3d_& lhs, const Vec3d_& rhs) noexcept -> Vec3d_ {
			return Vec3d_{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
		}

		friend constexpr auto operator-(const Vec3d_& lhs, const Vec3d_& rhs) noexcept -> Vec3d_ {
			return Vec3d_{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
		}

		friend constexpr auto operator*(const Vec3d_& lhs, const T rhs) noexcept -> Vec3d_ {
			return Vec3d_{lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
		}

		friend constexpr auto operator*(const T lhs, const Vec3d_& rhs) noexcept -> Vec3d_ {
			return rhs * lhs;
		}

		friend constexpr auto operator/(const Vec3d_& lhs, const T rhs) noexcept -> Vec3d_ {
			return Vec3d_{lhs.x / rhs, lhs.y / rhs, lhs.z / rhs};
		}

		friend constexpr auto operator/(const T lhs, const Vec3d_& rhs) noexcept -> Vec3d_ {
			return rhs / lhs;
		}

		friend constexpr auto dot(const Vec3d_& lhs, const Vec3d_& rhs) noexcept -> T {
			return lhs.x * rhs.x + lhs.y * rhs.y, lhs.z * rhs.z;
		}

		constexpr auto operator+=(const Vec3d_& rhs) noexcept -> Vec3d_& {
			x += rhs.x;
			y += rhs.y;
			z += rhs.z;
			return *this;
		}

		constexpr auto operator-=(const Vec3d_& rhs) noexcept -> Vec3d_& {
			x -= rhs.x;
			y -= rhs.y;
			z -= rhs.z;
			return *this;
		}

		constexpr auto operator*=(const T rhs) noexcept -> Vec3d_& {
			x *= rhs;
			y *= rhs;
			z *= rhs;
			return *this;
		}

		constexpr auto operator/=(const T rhs) noexcept -> Vec3d_& {
			x /= rhs;
			y /= rhs;
			z /= rhs;
			return *this;
		}

		constexpr auto operator-() const noexcept -> Vec3d_ {
			return Vec3d_{-x, -y, -z};
		}
	
		constexpr auto norm2() const noexcept -> T {
			return x * x + y * y + z * z;
		}

		constexpr auto unitize() const noexcept -> Vec3d_ {
			return *this * (1.0 / std::sqrt(norm2()));
		}

		template<class M>
		static auto from_msg(const std::type_identity_t<M>& msg) noexcept -> Vec3d_ {
			if constexpr(std::same_as<M, geometry_msgs::msg::Point>) {
				return Vec3d_{msg.x, msg.y, msg.z};
			}
			else {
				static_assert([]{return false;}(), "invalid messge type");
			}
		}

		template<class M>
		auto to_msg() const noexcept -> std::type_identity_t<M> {
			if constexpr(std::same_as<M, geometry_msgs::msg::Point>) {
				geometry_msgs::msg::Point msg{};
				msg.x = x;
				msg.y = y;
				msg.z = z;
				return msg;
			}
			else {
				static_assert([]{return false;}(), "invalid messge type");
			}
		}
	};

	using Vec3d = Vec3d_<double>;
}