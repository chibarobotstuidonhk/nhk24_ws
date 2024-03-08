/**
 * @file xyth.hpp
 * @brief 2次元座標系・3次元座標系に関する型と演算を提供する
 */

#pragma once

#include <cmath>
#include <numbers>

#include "operator_generator.hpp"

namespace nhk24_2nd_ws::xyth::impl {
	using nhk24_2nd_ws::operator_generator::BinaryLeftOp;
	using nhk24_2nd_ws::operator_generator::UnaryOp;

	// [-π, π)の範囲での差を求める
	namespace {
		struct ThetaNearistDiffOp final
			: BinaryLeftOp<"-", +[](const double lhs, const double rhs) -> double {
				const double diff = std::fmod(lhs - rhs, 2.0 * std::numbers::pi);
				return (diff >= std::numbers::pi) ? diff - 2.0 * std::numbers::pi : diff;  // 括弧がないと構文的なエラーが出る。多分gccのバグ
			}>
		{};
	}

	struct Xy final {
		double x;
		double y;

		static constexpr auto make(double x, double y) noexcept -> Xy {
			return Xy{x, y};
		}

		static constexpr auto from_polar(double r, double th) noexcept -> Xy {
			return Xy{r * std::cos(th), r * std::sin(th)};
		}

		static constexpr auto zero() noexcept -> Xy {
			return Xy{0.0, 0.0};
		}

		static constexpr auto unit_x() noexcept -> Xy {
			return Xy{1.0, 0.0};
		}

		static constexpr auto unit_y() noexcept -> Xy {
			return Xy{0.0, 1.0};
		}

		constexpr auto rot(double th) const noexcept -> Xy {
			const double c = std::cos(th);
			const double s = std::sin(th);
			return Xy{x * c - y * s, x * s + y * c};
		}

		constexpr auto norm2() const noexcept -> double {
			return x * x + y * y;
		}

		constexpr auto norm() const noexcept -> double {
			return std::sqrt(norm2());
		}

		constexpr auto unitize() const noexcept -> Xy {
			const double n = norm();
			return Xy{x / n, y / n};
		}
	};

	namespace {
		struct XyOp final
			: BinaryLeftOp<"+", +[](const Xy& lhs, const Xy& rhs) -> Xy {
				return Xy::make(lhs.x + rhs.x, lhs.y + rhs.y);
			}>
			, BinaryLeftOp<"-", +[](const Xy& lhs, const Xy& rhs) -> Xy {
				return Xy::make(lhs.x - rhs.x, lhs.y - rhs.y);
			}>
			, BinaryLeftOp<"*", +[](const double lhs, const Xy& rhs) -> Xy {
				return Xy::make(lhs * rhs.x, lhs * rhs.y);
			}>
			, BinaryLeftOp<"*", +[](const Xy& lhs, const double rhs) -> Xy {
				return Xy::make(lhs.x * rhs, lhs.y * rhs);
			}>
			, BinaryLeftOp<"/", +[](const Xy& lhs, const double rhs) -> Xy {
				return Xy::make(lhs.x / rhs, lhs.y / rhs);
			}>
			, BinaryLeftOp<"%", +[](const Xy& lhs, const Xy& rhs) -> double {
				return lhs.x * rhs.x + lhs.y * rhs.y;
			}>
			, UnaryOp<"-", +[](const Xy& xy) -> Xy {
				return Xy::make(-xy.x, -xy.y);
			}>
		{};
	}

	struct Xyz final {
		double x;
		double y;
		double z;

		static constexpr auto make(double x, double y, double z) noexcept -> Xyz {
			return Xyz{x, y, z};
		}

		static constexpr auto zero() noexcept -> Xyz {
			return Xyz{0.0, 0.0, 0.0};
		}

		constexpr auto norm2() const noexcept -> double {
			return x * x + y * y + z * z;
		}

		constexpr auto norm() const noexcept -> double {
			return std::sqrt(norm2());
		}

		constexpr auto unitize() const noexcept -> Xyz {
			const double n = norm();
			return Xyz{x / n, y / n, z / n};
		}
	};

	namespace {
		struct XyzOp final
			: BinaryLeftOp<"+", +[](const Xyz& lhs, const Xyz& rhs) -> Xyz {
				return Xyz::make(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
			}>
			, BinaryLeftOp<"-", +[](const Xyz& lhs, const Xyz& rhs) -> Xyz {
				return Xyz::make(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
			}>
			, BinaryLeftOp<"*", +[](const double lhs, const Xyz& rhs) -> Xyz {
				return Xyz::make(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
			}>
			, BinaryLeftOp<"*", +[](const Xyz& lhs, const double rhs) -> Xyz {
				return Xyz::make(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
			}>
			, BinaryLeftOp<"/", +[](const Xyz& lhs, const double rhs) -> Xyz {
				return Xyz::make(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
			}>
			, BinaryLeftOp<"%", +[](const Xyz& lhs, const Xyz& rhs) -> double {
				return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
			}>
		{};
	}

	struct Xyth final {
		Xy xy;
		double th;

		static constexpr auto make(Xy xy, double th) noexcept -> Xyth {
			return Xyth{xy, th};
		}

		static constexpr auto zero() noexcept -> Xyth {
			return Xyth{Xy::zero(), 0.0};
		}
	};

	namespace {
		struct XythNearistDiffOp final
			: BinaryLeftOp<"-", +[](const Xyth& lhs, const Xyth& rhs) -> Xyth {
				return Xyth {
					.xy = lhs.xy -XyOp{}- rhs.xy,
					.th = lhs.th -ThetaNearistDiffOp{}- rhs.th
				};
			}>
		{};
	}
}

namespace nhk24_2nd_ws::xyth {
	using impl::ThetaNearistDiffOp;
	using impl::Xy;
	using impl::XyOp;
	using impl::Xyz;
	using impl::XyzOp;
	using impl::Xyth;
	using impl::XythNearistDiffOp;
}