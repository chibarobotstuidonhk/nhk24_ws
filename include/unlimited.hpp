#error "未実装。よっぽどシビアな環境でもなきゃ浮動小数を使っていれば良い気がする"

// #pragma once

// #include <concepts>
// #include <compare>
// #include <limits>
// #include <variant>

// namespace nhk24_2nd_ws::unlimited {

// 	struct Inf final {};
// 	struct NegInf final {};
// 	struct Nan final {};

// 	template<class T_>
// 	struct UnlimitedAlgebra final {
// 		std::variant<T_, Inf, NegInf, Nan> value;

// 		friend constexpr auto operator<=>(const Unlimited& lhs, const Unlimited& rhs) = default;
// 		friend constexpr bool operator==(const Unlimited& lhs, const Unlimited& rhs) = default;
// 	};

// 	template<class T_>
// 	struct UnlimitedAlgebraImpl;

// 	template<class T_>
// 	concept unlimited_impl = requires {
// 		typename UnlimitedAlgebraImpl<T_>;
// 		// T_の最大値を返す
// 		{UnlimitedAlgebraImpl<T_>::max()} -> std::convertible_to<UnlimitedAlgebra<T_>>;
// 		// T_の最小値を返す
// 		{UnlimitedAlgebraImpl<T_>::min()} -> std::convertible_to<UnlimitedAlgebra<T_>>;
// 	};

// 	template<class T_>
// 	concept unlimited_group_impl = requires(const UnlimitedAlgebra<T_> a, const UnlimitedAlgebra<T_> b) {
// 		requires unlimited_impl<T_>;
// 		{UnlimitedAlgebraImpl<T_>::zero()} -> std::convertible_to<UnlimitedAlgebra<T_>>;
// 		{UnlimitedAlgebraImpl<T_>::add(a, b)} -> std::convertible_to<UnlimitedAlgebra<T_>>;
// 		{UnlimitedAlgebraImpl<T_>::sub(a, b)} -> std::convertible_to<UnlimitedAlgebra<T_>>;
// 	};

// 	template<class T_>
// 	concept unlimited_ring_impl = requires(const UnlimitedAlgebra<T_> a, const UnlimitedAlgebra<T_> b) {
// 		requires unlimited_group_impl<T_>;
// 		{UnlimitedAlgebraImpl<T_>::one()} -> std::convertible_to<UnlimitedAlgebra<T_>>;
// 		{UnlimitedAlgebraImpl<T_>::mul(a, b)} -> std::convertible_to<UnlimitedAlgebra<T_>>;
// 		{UnlimitedAlgebraImpl<T_>::div(a, b)} -> std::convertible_to<UnlimitedAlgebra<T_>>;
// 	};

// 	template<std::unsigned_integral T_>
// 	struct UnlimitedAlgebraImpl<T_> {
// 		static constexpr auto max() {
// 			return UnlimitedAlgebra<T_>{std::numeric_limits<T_>::max()};
// 		}

// 		static constexpr auto min() {
// 			return UnlimitedAlgebra<T_>{std::numeric_limits<T_>::min()};
// 		}

// 		static constexpr auto zero() {
// 			return UnlimitedAlgebra<T_>{0};
// 		}

// 		static constexpr auto add(const UnlimitedAlgebra<T_>& a, const UnlimitedAlgebra<T_>& b) {
// 			return std::visit([]<class A, class B>(const A& a, const B& b) {
// 				if constexpr(std::same_as<A, T_>) {
// 					if constexpr(std::same_as<B, T_>) {
// 						if(a <= std::numeric_limits<T_>::max() - b) {
// 							return UnlimitedAlgebra<T_>{a + b};
// 						} else {
// 							return UnlimitedAlgebra<T_>{Inf{}};
// 						}
// 					} else {
// 						return UnlimitedAlgebra<T_>{B{}};
// 					}
// 				} else {
// 					if constexpr(std::same_as<B, T_>) {
// 						return UnlimitedAlgebra<T_>{A{}};
// 					} else {
// 						if constexpr(std::same_as<A, Inf> && std::same_as<B, Inf>) {
// 							return UnlimitedAlgebra<T_>{Inf{}};
// 						} else if constexpr(std::same_as<A, NegInf> && std::same_as<B, NegInf>) {
// 							return UnlimitedAlgebra<T_>{NegInf{}};
// 						} else {
// 							return UnlimitedAlgebra<T_>{Nan{}};
// 						}
// 					}
// 				}
// 			}, a.value, b.value);
// 		}

// 		static constexpr auto sub(const UnlimitedAlgebra<T_>& a, const UnlimitedAlgebra<T_>& b) {
// 			return std::visit([]<class A, class B>(const A& a, const B& b) {
// 				if constexpr(std::same_as<A, T_>) {
// 					if constexpr(std::same_as<B, T_>) {
// 						if(a >= std::numeric_limits<T_>::min() + b) {
// 							return UnlimitedAlgebra<T_>{a - b};
// 						} else {
// 							return UnlimitedAlgebra<T_>{NegInf{}};
// 						}
// 					} else {
// 						return UnlimitedAlgebra<T_>{B{}};
// 					}
// 				} else {
// 					if constexpr(std::same_as<B, T_>) {
// 						return UnlimitedAlgebra<T_>{A{}};
// 					} else {
// 						if constexpr(std::same_as<A, Inf> && std::same_as<B, NegInf>) {
// 							return UnlimitedAlgebra<T_>{Inf{}};
// 						} else if constexpr(std::same_as<A, NegInf> && std::same_as<B, Inf>) {
// 							return UnlimitedAlgebra<T_>{NegInf{}};
// 						} else {
// 							return UnlimitedAlgebra<T_>{Nan{}};
// 						}
// 					}
// 				}
// 			}, a.value, b.value);
// 		}
// 	};
// }