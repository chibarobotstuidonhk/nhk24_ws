#pragma once

#include <shared_mutex>
#include <mutex>

namespace nhk24_2nd_ws::mutexed {
	template<class T_>
	struct Mutexed final {
		std::shared_mutex mtx;
		T_ val;

		static auto make(T_ v) -> Mutexed<T_> {
			return Mutexed<T_> {
				{}
				, v
			};
		}

		auto get() -> T_ {
			std::shared_lock lock(mtx);
			return val;
		}

		void set(T_ v) {
			std::lock_guard lock(mtx);
			val = v;
		}
	};
}