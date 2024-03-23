/**
 * @file wanko.hpp
 * @brief 1つのバッファを持つキューに対するpush操作とpop操作を排他制御するクラスを定義する
 */

#pragma once

#include <shared_mutex>
#include <optional>

namespace nhk24_2nd_ws::wanko {
	template<class T_>
	struct Wanko final {
		std::mutex mtx{};
		std::optional<T_> val{};

		static auto make() -> Wanko<T_> {
			return Wanko<T_>();
		}

		auto get() -> std::optional<T_> {
			std::lock_guard lock(mtx);
			if(val) {
				auto ret = std::move(*val);
				val.reset();
				return ret;
			}
			else {
				return std::nullopt;
			}
		}

		void set(T_&& v) {
			std::lock_guard lock(mtx);
			val = std::move(v);
		}

		auto empty() const -> bool {
			std::shared_lock lock(mtx);
			return !val;
		}

		auto try_set(T_&& v) -> bool {
			std::lock_guard lock(mtx);
			if(val) {
				return false;
			}
			else {
				val = std::move(v);
				return true;
			}
		}
	};
}