#pragma once

namespace nhk24_utils::stew::pid {

	template<class T>
	concept additive = requires(T a, T b) {
		{a + b} -> std::convertible_to<T>;
		{a - b} -> std::convertible_to<T>;
		{-a} -> std::convertible_to<T>;
	};

	template<class T>
	concept arithmetic = additive<T> && requires(T a, T b) {
		{a * b} -> std::convertible_to<T>;
		{a / b} -> std::convertible_to<T>;
	};

	template<class Scalar, class Vector>
	concept linear_algebra = arithmetic<Scalar>
	&& additive<Vector>
	&& requires (Scalar s, Vector v) {
		{v * s} -> std::convertible_to<Vector>;
		{s * v} -> std::convertible_to<Vector>;
		{v / s} -> std::convertible_to<Vector>;
	};

	template<class ValueT, class GainT = ValueT>
	requires linear_algebra<GainT, ValueT>
	struct Pid final {
		GainT k_p;
		GainT k_i;
		GainT k_d;

		ValueT max_integral;
		ValueT integral;
		ValueT last_error;

		static auto make(GainT k_p, GainT k_i, GainT k_d, ValueT max_integral) {
			return Pid{k_p, k_i, k_d, max_integral, {}, {}};
		}

		auto update(ValueT error, GainT dt) noexcept {
			integral = integral + error * dt;
			const auto derivative = (error - last_error) / dt;
			last_error = error;
			return k_p * error + k_i * integral + k_d * derivative;
		}
	};
}