#pragma once

#include <string_view>
#include <optional>

#include "std_types.hpp"
#include "parser_combinator.hpp"

namespace nhk24_2nd_ws::basic_parser::impl {
	using parser_combinator::ParserMarker;
	using parser_combinator::Parsult;
	using parser_combinator::literal;
	using namespace parser_combinator::ops;


	struct Alphabet final : ParserMarker {
		using V = char;

		static constexpr auto parse(std::u8string_view sv) -> Parsult<Alphabet::V> {
			if(sv.empty()) {
				return Parsult<Alphabet::V>{std::nullopt, sv};
			}

			const char c = sv.front();
			if(('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z')) {
				return Parsult<Alphabet::V>{c, sv.substr(1)};
			} else {
				return Parsult<Alphabet::V>{std::nullopt, sv};
			}
		}
	};
	inline constexpr auto alphabet = Alphabet{};

	template<u8 begin, u8 included_end>
	struct Digit final : ParserMarker {
		using V = u8;

		static constexpr auto parse(std::u8string_view sv) -> Parsult<Digit::V> {
			if(sv.empty()) {
				return Parsult<Digit::V>{std::nullopt, sv};
			}

			const u8 c = sv.front();
			if(begin <= c && c <= included_end) {
				return Parsult<Digit::V>{c - '0', sv.substr(1)};
			} else {
				return Parsult<Digit::V>{std::nullopt, sv};
			}
		}
	};

	template<u8 begin, u8 included_end>
	inline constexpr auto digit = Digit<begin, included_end>{};

	inline constexpr auto decimal = ~digit<'1', '9'> - *(digit<'0', '9'>) >>= [](auto&& x) -> u64 {
		u64 ret = std::get<0>(x) ? *std::get<0>(x) : 0;
		for(const auto& d : std::get<1>(x)) {
			ret = ret * 10 + d;
		}

		return ret;
	};

	inline constexpr auto floating = decimal - ~literal<u8"."> - *digit<'0', '9'> >>= [](auto&& x) -> double {
		double ret = std::get<0>(x);
		u64 div = 1;
		for(const auto& d : std::get<2>(x)) {
			ret = ret * 10 + d;
			div *= 10;
		}

		return static_cast<double>(ret / div);
	};
}

namespace nhk24_2nd_ws::basic_parser {
	using impl::alphabet;
	using impl::digit;
	using impl::decimal;
	using impl::floating;
}