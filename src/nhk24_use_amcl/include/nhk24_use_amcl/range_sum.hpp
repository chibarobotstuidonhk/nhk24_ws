#pragma once

#include <utility>
#include <array>
#include "std_type.hpp"

namespace nhk24_use_amcl::stew::range_sum {
	template<class T, size_t n>
	struct RangeSum final {
		std::array<T, n> circuler_buffer;
		T sum;
		size_t index;

		static constexpr auto make(const T& v) noexcept -> RangeSum {
			return RangeSum {
				[]<size_t ... indices>(const T& v, std::index_sequence<indices ...>) {
					return std::array<T, n>{(indices, v) ...};
				}(v, std::make_index_sequence<n>{})
				, 0
			};
		}

		constexpr auto update(const T& v) noexcept -> T {
			sum -= circuler_buffer[index];
			circuler_buffer[index] = v;
			sum += v;
			index = (index + 1) % n;
			return sum;
		}
	};
}