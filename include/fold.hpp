#pragma once

#include <utility>

namespace nhk24_2nd_ws::fold::impl {

	template<class A_, auto f_>
	struct Fold final {
		A_ a;

		template<class B_>
		constexpr auto operator>>(B_&& b) {
			return Fold<decltype(f_(std::forward<A_>(a), std::forward<B_>(b))), f_>{f_(std::forward<A_>(a), std::forward<B_>(b))};
		}

		template<class B_>
		constexpr auto operator<<=(B_&& b) {
			return Fold<decltype(f_(std_forward<A_>(a), std::forward<B_>(b))), f_>{f_(std::forward<A_>(a), std::forward<B_>(b))};
		}
	};

	template<auto f_>
	struct Fold<void, f_> final {
		template<class B_>
		constexpr auto operator>>(B_&& b) -> Fold<B_, f_> {
			return Fold<B_, f_>{std::forward<B_>(b)};
		}

		template<class B_>
		constexpr auto operator<<=(B_&& b) -> Fold<B_, f_> {
			return Fold<B_, f_>{std::forward<B_>(b)};
		}
	};

	/**
	 * @brief 左畳み込み。途中で型を変えることができる
	 * 
	 * @tparam f_ 
	 * @tparam Args_ 
	 * @param args 
	 * @return constexpr auto 
	 */
	template<auto f_, class ... Args_>
	inline constexpr auto foldl(Args_&& ... args) {
		decltype(auto) a = (Fold<void, f_>{} >> ... >> std::forward<Args_>(args)).a;
		return std::forward<decltype(a)>(a);
	}

	/**
	 * @brief 右畳み込み。途中で型を変えることができる
	 * 
	 * @tparam f_ 
	 * @tparam Args_ 
	 * @param args 
	 * @return constexpr auto 
	 */
	template<auto f_, class ... Args_>
	inline constexpr auto foldr(Args_&& ... args) {
		decltype(auto) a = (std::forward<Args_>(args) <<= ... <<= Fold<void, f_>{}).a;
		return std::forward<decltype(a)>(a);
	}
}

namespace nhk24_2nd_ws::fold {
	using impl::foldl;
	using impl::foldr;
}