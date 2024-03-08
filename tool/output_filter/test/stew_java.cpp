/**
 * @file stew_java.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-24
 * 
 * @copyright Copyright (c) 2024
 * 
 * @brief Unicode Escapes に未対応。
 * 
 */

#include <fstream>
#include <iostream>
#include <string>
#include "../string_literal.hpp"
#include "../parser_combinator.hpp"
#include "../debug_print.hpp"

namespace stew_java {
	using futaiten0::parser_combinator::Parsult;
	using futaiten0::parser_combinator::until_star;
	using namespace futaiten0::parser_combinator::ops;
	using crs_lib::string_literal::StringLiteral;

	template<StringLiteral sl_>
	inline constexpr auto literal = futaiten0::parser_combinator::literal<sl_>;

	inline constexpr auto line_terminator = literal<u8"\r\n"> | literal<u8"\n"> | literal<u8"\r">;
	inline constexpr auto whitespace = literal<u8" "> | literal<u8"\t">;

	inline constexpr auto oneline_comment = literal<u8"//"> - until_star(line_terminator) - line_terminator
		>>= [](auto&& v) -> std::u8string_view {return std::get<1>(v);};
	inline constexpr auto traditional_comment = literal<u8"/*"> - until_star(literal<u8"*/">) - literal<u8"*/">
		>>= [](auto&& v) -> std::u8string_view {return std::get<1>(v);};
	inline constexpr auto comment = oneline_comment | traditional_comment
		>>= [](auto&& v) -> std::u8string_view {return std::visit([](auto&& v){return v;}, std::forward<decltype(v)>(v));};
}

using crs_lib::holder::ValueHolder;
using crs_lib::string_literal::StringLiteral;
namespace crs_lib::debug_print {
	template<auto v_>
	struct DebugPrint<ValueHolder<v_>> final {
		static constexpr std::string_view name = "ValueHolder";

		static void print(ValueHolder<v_>) {
			debug_print::print(v_);
		}
	};

	template<class Char_, usize n_>
	struct DebugPrint<StringLiteral<Char_, n_>> final {
		static constexpr std::string_view name = "StringLiteral";

		static void print(const StringLiteral<Char_, n_>& sl) {
			debug_print::print(static_cast<std::basic_string_view<Char_>>(sl));
		}
	};
}

using futaiten0::parser_combinator::impl::U8StringLiteral;
using crs_lib::debug_print::printlns;
using crs_lib::debug_print::PinCOut;

int main(int argc, char** argv) {
	(void)argc;
	(void)argv;

	using namespace std::literals;

	constexpr auto read_content = []() -> std::optional<std::string> {
		std::ifstream ifs{};
		std::istreambuf_iterator<char> iter{};
		try {
			ifs = std::ifstream("sample.cpp");
			std::cout << "is_open: " << ifs.is_open() << std::endl;
			iter = std::istreambuf_iterator<char>(ifs);
		}
		catch(const std::exception& e) {
			std::cerr << __LINE__ << ": " << e.what();
			return {};
		}
		
		try {
			return std::string(iter, std::istreambuf_iterator<char>{});
		}
		catch(const std::exception& e) {
			std::cerr << __LINE__ << ": " << e.what();
			return {};
		}
	};

	const auto content = read_content();
	if(!content) return 0;
	auto src = std::u8string_view{(const char8_t *)(const void *)content->data(), content->size()};
	printlns(src);

	std::ofstream out{"stew_java_txt.txt"};
	auto pin = PinCOut::make(out);

	using namespace stew_java;
	using namespace std::literals;

	unsigned int i = 0;
	while(!src.empty()) {
		auto result = decltype(comment)::parse(src);
		if(result) {
			printlns(i, ": ", result.value);
			src = result.rest;
			++i;
		}
		else {
			src.remove_prefix(1);
		}
	}

	return 0;
}