#pragma once

#include <iostream>
#include <iomanip>
#include <array>
#include <vector>
#include <string>
#include <string_view>
#include <tuple>
#include <optional>
#include <variant>
#include <concepts>
#include <type_traits>
#include <algorithm>

#include "std_types.hpp"

namespace nhk24_2nd_ws::debug_print::impl {
	template<class T_>
	struct DebugPrint;

	template<class T_>
	concept debug_printable = requires(const T_& imut) {
		{DebugPrint<T_>::print(imut)};
		{DebugPrint<T_>::name} -> std::convertible_to<std::string_view>;
	} || requires(const T_& imut) {
		{std::cout << imut};
	};

	/**
	 * @brief デバッグ用の出力を行う
	 * 
	 * @tparam T_ 
	 * @param imut 
	 */
	template<debug_printable T_>
	void print(const T_& imut) {
		if constexpr(requires {DebugPrint<T_>::print(imut);}) {
			DebugPrint<std::remove_cvref_t<T_>>::print(imut);
		} else {
			std::cout << "unknown{" << imut << "}";
		}
	}

	/**
	 * @brief デバッグ用の出力を行う。末尾に改行あり
	 * 
	 * @tparam T_ 
	 * @param imut 
	 */
	template<debug_printable T_>
	void println(const T_& imut) {
		debug_print::impl::print(imut);
		std::cout << std::endl;
	}

	/**
	 * @brief デバッグ用の出力を行う。複数個の引数を受け取る
	 * 
	 * @tparam Ts_ 
	 * @param imuts 
	 */
	template<class ... Ts_>
	void prints(const Ts_& ... imuts) {
		((debug_print::impl::print(imuts), std::cout << ' '), ...);
	}

	/**
	 * @brief デバッグ用の出力を行う。複数個の引数を受け取る。末尾に改行あり
	 * 
	 * @tparam Ts_ 
	 * @param imuts 
	 */
	template<class ... Ts_>
	void printlns(const Ts_& ... imuts) {
		prints(imuts...);
		std::cout << std::endl;
	}

	template<>
	struct DebugPrint<bool> {
		static constexpr std::string_view name = "bool";
		static void print(const bool& imut) {
			std::cout << (imut ? "true" : "false");
		}
	};

	template<>
	struct DebugPrint<char> {
		static constexpr std::string_view name = "char";
		static void print(const char& imut) {
			std::cout << "\'" << imut << "\'";
		}
	};

	template<>
	struct DebugPrint<char8_t> {
		static constexpr std::string_view name = "char8_t";
		static void print(const char8_t& imut) {
			std::cout << "\'\\x" << std::hex << (unsigned int)imut << std::dec << "\'8";
		}
	};

	template<>
	struct DebugPrint<u8> {
		static constexpr std::string_view name = "u8";
		static void print(const u8& imut) {
			std::cout << static_cast<u16>(imut) << "_u8";
		}
	};

	template<>
	struct DebugPrint<u16> {
		static constexpr std::string_view name = "u16";
		static void print(const u16& imut) {
			std::cout << imut << "_u16";
		}
	};

	template<>
	struct DebugPrint<u32> {
		static constexpr std::string_view name = "u32";
		static void print(const u32& imut) {
			std::cout << imut << "_u32";
		}
	};

	template<>
	struct DebugPrint<u64> {
		static constexpr std::string_view name = "u64";
		static void print(const u64& imut) {
			std::cout << imut << "_u64";
		}
	};

	template<>
	struct DebugPrint<i8> {
		static constexpr std::string_view name = "i8";
		static void print(const i8& imut) {
			std::cout << static_cast<i16>(imut) << "_i8";
		}
	};

	template<>
	struct DebugPrint<i16> {
		static constexpr std::string_view name = "i16";
		static void print(const i16& imut) {
			std::cout << imut << "_i16";
		}
	};

	template<>
	struct DebugPrint<i32> {
		static constexpr std::string_view name = "i32";
		static void print(const i32& imut) {
			std::cout << imut << "_i32";
		}
	};

	template<>
	struct DebugPrint<i64> {
		static constexpr std::string_view name = "i64";
		static void print(const i64& imut) {
			std::cout << imut << "_i64";
		}
	};

	template<>
	struct DebugPrint<float> {
		static constexpr std::string_view name = "float";
		static void print(const float& imut) {
			std::cout << imut << "_f";
		}
	};

	template<>
	struct DebugPrint<double> {
		static constexpr std::string_view name = "double";
		static void print(const double& imut) {
			std::cout << imut << "_d";
		}
	};

	template<>
	struct DebugPrint<long double> {
		static constexpr std::string_view name = "long double";
		static void print(const long double& imut) {
			std::cout << imut << "_L";
		}
	};

	template<usize n_>
	struct DebugPrint<char[n_]> {
		static constexpr std::string_view name = "char[n_]";
		static void print(const char (&imut)[n_]) {
			std::cout << "\"" << imut << "\"";
		}
	};

	template<>
	struct DebugPrint<std::string> {
		static constexpr std::string_view name = "std::string";
		static void print(const std::string& imut) {
			std::cout << "\"" << imut << "\"s";
		}
	};

	template<>
	struct DebugPrint<std::string_view> {
		static constexpr std::string_view name = "std::string_view";
		static void print(const std::string_view& imut) {
			std::cout << "\"" << imut << "\"sv";
		}
	};

	template<>
	struct DebugPrint<std::u8string_view> {
		static constexpr std::string_view name = "std::u8string_view";
		static void print(const std::u8string_view& imut) {
			std::cout << "\"" << std::string_view{(const char*)imut.data(), imut.size()} << "\"u8sv";
		}
	};

	template<>
	struct DebugPrint<std::u8string> {
		static constexpr std::string_view name = "std::u8string";
		static void print(const std::u8string& imut) {
			std::cout << "\"" << std::string_view{(const char*)imut.data(), imut.size()} << "\"s";
		}
	};

	template<usize n_>
	struct DebugPrint<char8_t[n_]> {
		static constexpr std::string_view name = "char8_t[n_]";
		static void print(const char8_t (&imut)[n_]) {
			std::cout << "\"" << std::string_view{imut} << "\"u8";
		}
	};

	template<debug_printable T_, usize n_>
	struct DebugPrint<T_[n_]> {
		static constexpr std::string_view name = "T[n_]";

		static void print(const T_ (&imut)[n_]) {
			std::cout << "[" << n_ << "]{";
			for(const auto& e : imut) {
				std::cout << " ";
				debug_print::impl::print(e);
				std::cout << ",";
			}
			std::cout << "}";
		}
	};

	template<debug_printable T_, usize n_>
	struct DebugPrint<std::array<T_, n_>> {
		static constexpr std::string_view name = "std::array<T_, n_>";

		static void print(const std::array<T_, n_>& imut) {
			std::cout << "std::array[" << n_ << "]{";
			for(const auto& e : imut) {
				std::cout << " ";
				debug_print::impl::print(e);
				std::cout << ",";
			}
			std::cout << "}";
		}
	};

	template<debug_printable T_>
	struct DebugPrint<std::vector<T_>> {
		static constexpr std::string_view name = "std::vector<T_>";

		static void print(const std::vector<T_>& imut) {
			std::cout << "std::vector{";
			for(const auto& e : imut) {
				std::cout << " ";
				debug_print::impl::print(e);
				std::cout << ",";
			}
			std::cout << "}";
		}
	};

	template<debug_printable ... Ts_>
	struct DebugPrint<std::tuple<Ts_...>> {
		static constexpr std::string_view name = "std::tuple<Ts_...>";

		static void print(const std::tuple<Ts_...>& imut) {
			std::cout << "std::tuple{";
			std::apply([](const auto& ... imuts) {
				((std::cout << ' ', debug_print::impl::print(imuts), std::cout << ','), ...);
			}, imut);
			std::cout << "}";
		}
	};

	template<debug_printable T_>
	struct DebugPrint<std::optional<T_>> {
		static constexpr std::string_view name = "std::optional<T_>";

		static void print(const std::optional<T_>& imut) {
			if(imut) {
				std::cout << "std::optional{";
				debug_print::impl::print(*imut);
				std::cout << "}";
			} else {
				std::cout << "std::optional{nullopt}";
			}\
		}
	};

	template<debug_printable ... Ts_>
	struct DebugPrint<std::variant<Ts_...>> {
		static constexpr std::string_view name = "std::variant<Ts_...>";

		static void print(const std::variant<Ts_...>& imut) {
			std::cout << "std::variant{";
			std::cout << imut.index() << ": ";
			std::visit([](const auto& imut) {
				debug_print::impl::print(imut);
			}, imut);
			std::cout << "}";
		}
	};

	template<>
	struct DebugPrint<std::monostate> {
		static constexpr std::string_view name = "std::monostate";

		static void print(const std::monostate&) {
			std::cout << "std::monostate";
		}
	};

	/**
	 * @brief coutを一時的に切り替える
	 * 
	 */
	struct PinCOut final {
		std::streambuf * old;

		static auto make(std::ostream& os) {
			return PinCOut{std::cout.rdbuf(os.rdbuf())};
		}

		private:
		PinCOut(std::streambuf *const old)
		: old{old}
		{}

		public:
		PinCOut(const PinCOut&) = delete;
		auto operator=(const PinCOut&) -> PinCOut& = delete;

		PinCOut(PinCOut&& other)
		: old{other.old}
		{
			other.old = nullptr;
		}
		
		auto operator=(PinCOut&&) = delete;

		~PinCOut() {
			std::cout.rdbuf(old);
		}
	};
}

namespace nhk24_2nd_ws::debug_print {
	using impl::debug_printable;
	using impl::DebugPrint;
	using impl::prints;
	using impl::printlns;
	using impl::PinCOut;
}