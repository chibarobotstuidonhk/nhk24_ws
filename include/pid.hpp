/**
 * @file pid.hpp
 * @brief PID制御器。算術オーバーフローを考慮していないので注意。特にI制御を使うときは浮動小数なり自前の閉じた整数型なりを使うこと。
 */

#pragma once

#include <concepts>
#include <compare>
#include <optional>

#include "operator_generator.hpp"

namespace nhk24_2nd_ws::pid::impl {
	using nhk24_2nd_ws::operator_generator::AutoBinaryLeftOp;
	using nhk24_2nd_ws::operator_generator::AutoUnaryOp;

	namespace {
		struct TrivialOp final
			: AutoBinaryLeftOp<"+", [](const auto& lhs, const auto& rhs) -> auto { return lhs + rhs; }>
			, AutoBinaryLeftOp<"-", [](const auto& lhs, const auto& rhs) -> auto { return lhs - rhs; }>
			, AutoBinaryLeftOp<"*", [](const auto& lhs, const auto& rhs) -> auto { return lhs * rhs; }>
			, AutoBinaryLeftOp<"/", [](const auto& lhs, const auto& rhs) -> auto { return lhs / rhs; }>
			, AutoUnaryOp<"-", [](const auto& x) -> auto { return -x; }>
		{};
	}

	inline constexpr auto trivial = TrivialOp{};

	template<class T_, auto op = trivial>
	concept additive = requires(T_ a, T_ b) {
		{a +op+ b} -> std::convertible_to<T_>;
		{a -op- b} -> std::convertible_to<T_>;
		{(-op)* a} -> std::convertible_to<T_>;
	};

	template<class T_, auto op = trivial>
	concept arithmetic = additive<T_, op> && requires(T_ a, T_ b) {
		{a *op* b} -> std::convertible_to<T_>;
		{a /op/ b} -> std::convertible_to<T_>;
	};

	template<class Scalar_, class Vector_, auto s_op = trivial, auto v_op = trivial, auto sv_op = trivial>
	concept linear_algebra = arithmetic<Scalar_, s_op>
	&& additive<Vector_, v_op>
	&& requires (Scalar_ s, Vector_ v) {
		{v *sv_op* s} -> std::convertible_to<Vector_>;
		{s *sv_op* v} -> std::convertible_to<Vector_>;
		{v /sv_op/ s} -> std::convertible_to<Vector_>;
	};

	/**
	 * @brief PID制御器
	 * 
	 * @tparam ValueT_ 線型空間の要素
	 * @tparam GainT_ 線型空間のスカラー
	 * @attention 算術オーバーフローを考慮していないので注意。特にI制御を使うときは浮動小数なり自前の閉じた整数型なりを使うこと。
	 */
	template<class ValueT_, class GainT_ = ValueT_, auto s_op = trivial, auto v_op = trivial, auto sv_op = trivial>
	requires linear_algebra<GainT_, ValueT_, s_op, v_op, sv_op>
	struct Pid final {
		GainT_ k_p;
		GainT_ k_i;
		GainT_ k_d;

		ValueT_ last_error;
		ValueT_ integral;

		static auto make(GainT_ k_p, GainT_ k_i, GainT_ k_d) {
			return Pid{k_p, k_i, k_d, {}, {}};
		}

		auto update(ValueT_ error, GainT_ dt) noexcept {
			const auto derivative = (error -v_op- last_error) /sv_op/ dt;
			integral = integral +v_op+ error *sv_op* dt;
			last_error = error;
			return k_p *sv_op* error +v_op+ k_i *sv_op* integral +v_op+ k_d *sv_op* derivative;
		}
	};
}

namespace nhk24_2nd_ws::pid {
	using impl::Pid;
	using impl::trivial;
}