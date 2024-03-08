#include <include/std_types.hpp>
#include <include/parser_combinator.hpp>
#include <include/basic_parser.hpp>

namespace nhk24_2nd_ws::output_filter {
	using parser_combinator::literal;
	using namespace parser_combinator::ops;
	using basic_parser::alphabet;
	using basic_parser::digit;

	constexpr auto path = +(alphabet | digit<'0', '9'> | literal<u8"_"> | literal<u8"-"> | literal<u8"/"> | literal<u8"."> | literal<u8" ">);
}

int main() {

}