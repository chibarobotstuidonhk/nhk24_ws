/**
 * @file parser_combinator.hpp
 * @brief パーサーコンビネーター。使用例はtool/output_filter/testを参照。
 */

#pragma once

#include <variant>
#include <tuple>
#include <optional>
#include <vector>
#include <utility>
#include <type_traits>
#include <concepts>
#include <string_view>

#include "std_types.hpp"
#include "holder.hpp"
#include "string_literal.hpp"
#include "fold.hpp"

namespace nhk24_2nd_ws::parser_combinator::impl {
	using namespace nhk24_2nd_ws::std_types;
	using nhk24_2nd_ws::holder::TypeHolder;
	using nhk24_2nd_ws::holder::ValueHolder;
	using nhk24_2nd_ws::holder::JoinHolder;

	template<usize n_>
	using U8StringLiteral = nhk24_2nd_ws::string_literal::StringLiteral<char8_t, n_>;

	using nhk24_2nd_ws::fold::foldl;

	template<class T_>
	inline constexpr auto some(T_&& value) {
		return std::optional<std::remove_cvref_t<T_>>{std::forward<T_>(value)};
	}

	/**
	 * @brief 値っぽくない型じゃないかをチェックするコンセプト
	 * 
	 * @tparam T_ 
	 */
	template<class T_>
	concept value_like = std::same_as<T_, std::remove_cvref_t<T_>> && std::is_object_v<T_> && (!std::is_array_v<T_>);

	/**
	 * @brief パース結果を持つ型。valueがstd::nulloptの場合はパース失敗。
	 * 
	 * @tparam T_ パース結果
	 */
	template<value_like T_>
	struct Parsult final {
		std::optional<T_> value;
		std::u8string_view rest;

		explicit constexpr operator bool() const noexcept {
			return value.has_value();
		}
	};

	/**
	 * @brief パーサーのマーカートレイト
	 * 
	 */
	struct ParserMarker {};

	/**
	 * @brief パーサーのコンセプト。独自パーサーを作る場合はこれを満たすこと。
	 * 
	 * @tparam T_ 
	 */
	template<class T_>
	concept parser_like = requires {
		requires std::derived_from<T_, ParserMarker>;
		typename T_::V;
		requires value_like<typename T_::V>;
		requires requires(const std::u8string_view src) {
			{T_::parse(src)} -> std::same_as<Parsult<typename T_::V>>;
		};
	};

	/**
	 * @brief 文字列リテラルからパーサーを作る。終端記号のイメージ。
	 * 
	 * @tparam literal_ 
	 */
	template<U8StringLiteral literal_>
	struct Literal final : ParserMarker {
		static constexpr auto literal = literal_;

		using V = ValueHolder<literal_>;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			auto rest = src;
			if(rest.starts_with(static_cast<std::u8string_view>(literal_))) {
				rest.remove_prefix(literal_.length());
				return Parsult<V>{some(V{}), rest};
			}
			else {
				return Parsult<V>{std::nullopt, src};
			}
		}
	};

	template<U8StringLiteral literal_>
	Literal<literal_> literal{};

	/**
	 * @brief マッチするとnulloptを、マッチしないとNot::Successを返す
	 * 
	 * @tparam Parser_ 
	 */
	template<parser_like Parser_>
	struct Not final : ParserMarker {
		struct Success final {};
		using V = Success;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			if(auto result = Parser_::parse(src); result) {
				return Parsult<V>{std::nullopt, src};
			}
			else {
				return Parsult<V>{some(V{}), result.rest};
			}
		}
	};

	/**
	 * @brief 0個ないし1個にマッチする。グリーディ。
	 * 
	 * @tparam Parser_ 
	 */
	template<parser_like Parser_>
	struct Maybe final : ParserMarker {
		using V = std::optional<typename Parser_::V>;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			if(auto result = Parser_::parse(src); result) {
				return Parsult<V>{some(std::move(*result.value)), result.rest};
			}
			else {
				return Parsult<V>{some(V{std::nullopt}), src};
			}
		}
	};

	/**
	 * @brief 連結。
	 * 
	 * @tparam Parsers_ 
	 */
	template<parser_like ... Parsers_>
	struct Concat final : ParserMarker {
		using V = std::tuple<typename Parsers_::V ...>;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			auto result = foldl <
				[]<class Tup_, parser_like Parser_>(Parsult<Tup_>&& l, TypeHolder<Parser_>) -> Parsult<decltype(std::tuple_cat(std::move(*l.value), std::declval<std::tuple<typename Parser_::V>>()))> {
					if(l) {
						auto p = Parser_::parse(l.rest);
						if(p) {
							auto tup = std::tuple_cat(std::move(*l.value), std::tuple<typename Parser_::V>{std::move(*p.value)});
							return Parsult<decltype(tup)>{std::move(tup), p.rest};
						}
						else {
							return {std::nullopt, l.rest};
						}
					}
					else {
						return {std::nullopt, l.rest};
					}
				}
			>(Parsult<std::tuple<>>{std::tuple<>{}, src}, TypeHolder<Parsers_>{} ...);

			if(result) {
				return Parsult<V>{V{std::move(*result.value)}, result.rest};
			}
			else {
				return Parsult<V>{std::nullopt, src};
			}
		}
	};

	template<usize n_, class T_>
	struct Indexer final {
		static constexpr auto n = n_;
		using T = T_;
		T_ value;
	};

	/**
	 * @brief 優先度つき選択。左から順に試す。
	 * 
	 * @tparam Parsers_ 
	 */
	template<parser_like ... Parsers_>
	struct PrioAlter final : ParserMarker {
		using V = std::variant<typename Parsers_::V ...>;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			auto indexer = foldl <
				[]<usize n_, parser_like Parser_>(Indexer<n_, Parsult<V>>&& indexer, TypeHolder<Parser_>) -> Indexer<n_ + 1, Parsult<V>> {
					auto&& l = indexer.value;
					if(l) {
						return {std::move(l)};
					}
					else {
						auto p = Parser_::parse(l.rest);
						if(p) {
							return {Parsult<V>{V{std::in_place_index<n_>, std::move(*p.value)}, p.rest}};
						}
						else {
							return {Parsult<V>{std::nullopt, l.rest}};
						}
					}
				}
			>(Indexer<0, Parsult<V>>{Parsult<V>{std::nullopt, src}}, TypeHolder<Parsers_>{} ...);

			if(const auto result = indexer.value; result) {
				return Parsult<V>{V{std::move(*result.value)}, result.rest};
			}
			else {
				return Parsult<V>{std::nullopt, src};
			}
		}
	};

	/**
	 * @brief 0個以上の繰り返し。グリーディ。
	 * 
	 * @tparam Parser_ 
	 */
	template<parser_like Parser_>
	struct Star final : ParserMarker {
		using V = std::vector<typename Parser_::V>;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			auto result = std::vector<typename Parser_::V>{};
			auto rest = src;
			while(!rest.empty()) {
				if(auto p = Parser_::parse(rest); p) {
					result.emplace_back(std::move(*p.value));
					rest = p.rest;
				}
				else {
					break;
				}
			}
			return Parsult<V>{std::move(result), rest};
		}
	};

	/**
	 * @brief 1個以上の繰り返し。グリーディ。
	 * 
	 * @tparam Parser_ 
	 */
	template<parser_like Parser_>
	struct Plus final : ParserMarker {
		using V = std::vector<typename Parser_::V>;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			auto result = std::vector<typename Parser_::V>{};
			auto rest = src;
			while(!rest.empty()) {
				if(auto p = Parser_::parse(rest); p) {
					result.emplace_back(std::move(*p.value));
					rest = p.rest;
				}
				else {
					break;
				}
			}

			if(rest.data() == src.data()) {
				return Parsult<V>{std::nullopt, src};
			}
			else {
				return Parsult<V>{std::move(result), rest};
			}
		}
	};

	/**
	 * @brief 0個以上、マッチするまで繰り返す。
	 * 
	 * @tparam Parser_ 
	 */
	template<parser_like Parser_>
	struct UntilStar final : ParserMarker {
		using V = std::u8string_view;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			auto rest = src;
			while(!rest.empty()) {
				if(auto p = Parser_::parse(rest); p) {
					break;
				}
				else {
					rest.remove_prefix(1);
				}
			}

			return Parsult<V>{V{src.data(), static_cast<usize>(rest.data() - src.data())}, rest};
		}
	};

	/**
	 * @brief 1個以上、マッチするまで繰り返す。
	 * 
	 * @tparam Parser_ 
	 */
	template<parser_like Parser_>
	struct UntilPlus final : ParserMarker {
		using V = std::u8string_view;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			auto rest = src;
			while(!rest.empty()) {
				if(auto p = Parser_::parse(rest); p) {
					break;
				}
				else {
					rest.remove_prefix(1);
				}
			}

			if(rest.data() == src.data()) {
				return Parsult<V>{std::nullopt, src};
			}
			else {
				return Parsult<V>{V{src.data(), static_cast<usize>(rest.data() - src.data())}, rest};
			}
		}
	};

	template<parser_like Parser_>
	inline constexpr auto until_star(Parser_) -> UntilStar<Parser_> {
		return {};
	}

	template<parser_like Parser_>
	inline constexpr auto until_plus(Parser_) -> UntilPlus<Parser_> {
		return {};
	}

	/**
	 * @brief パース結果に関数を適用するパーサー
	 * 
	 * @tparam Parser_ 
	 * @tparam F 
	 */
	template<parser_like Parser_, class F>
	requires std::invocable<F, typename Parser_::V>
	struct ParserWrapper final : ParserMarker {
		using V = std::invoke_result_t<F, typename Parser_::V>;

		static constexpr auto parse(const std::u8string_view src) -> Parsult<V> {
			auto result = Parser_::parse(src);
			if(result) {
				return Parsult<V>{some(F{}(std::move(*result.value))), result.rest};
			}
			else {
				return Parsult<V>{std::nullopt, src};
			}
		}
	};

	namespace ops {
		template<parser_like L_, parser_like R_>
		inline constexpr auto operator-(L_, R_) -> typename decltype(JoinHolder<Concat>::join(TypeHolder<L_>{}, TypeHolder<R_>{}))::T {
			return {};
		}

		template<parser_like L_, parser_like R_>
		inline constexpr auto operator|(L_, R_) -> typename decltype(JoinHolder<PrioAlter>::join(TypeHolder<L_>{}, TypeHolder<R_>{}))::T {
			return {};
		}

		template<parser_like Parser_>
		inline constexpr auto operator*(Parser_) -> Star<Parser_> {
			return {};
		}

		template<parser_like Parser_>
		inline constexpr auto operator+(Parser_) -> Plus<Parser_> {
			return {};
		}

		template<parser_like Parser_>
		inline constexpr auto operator!(Parser_) -> Not<Parser_> {
			return {};
		}

		template<parser_like Parser_>
		inline constexpr auto operator~(Parser_) -> Maybe<Parser_> {
			return {};
		}

		template<parser_like Parser_, class F>
		requires std::invocable<F, typename Parser_::V>
		inline constexpr auto operator>>=(Parser_, F) -> ParserWrapper<Parser_, F> {
			return {};
		}
	}
}

namespace nhk24_2nd_ws::parser_combinator {
	using impl::TypeHolder;
	using impl::ValueHolder;
	using impl::Parsult;
	using impl::ParserMarker;
	using impl::Literal;
	using impl::literal;
	using impl::Not;
	using impl::Maybe;
	using impl::Concat;
	using impl::PrioAlter;
	using impl::Star;
	using impl::Plus;
	using impl::UntilStar;
	using impl::UntilPlus;
	using impl::until_star;
	using impl::until_plus;
	using impl::ParserWrapper;

	namespace ops {
		using impl::ops::operator!;
		using impl::ops::operator~;
		using impl::ops::operator*;
		using impl::ops::operator+;
		using impl::ops::operator-;
		using impl::ops::operator|;
		using impl::ops::operator>>=;
	}
}