#pragma once

#include <concepts>
#include <tuple>
#include <utility>
#include <optional>
#include <memory>
#include <stop_token>

#include <my_include/std_types.hpp>

namespace nhk24_2nd_ws::state_machine::impl {
	template<class Io_>
	struct StateBase {
		virtual ~StateBase() = default;
		virtual auto run(Io_& io) -> std::unique_ptr<StateBase> = 0;
	};

	template<class T_, class Io_>
	concept state_like = std::derived_from<T_, StateBase<Io_>>;

	template<class Io_>
	struct StateMachine final {
		std::unique_ptr<StateBase<Io_>> state;

		void run(Io_& io) {
			while(state) {
				state = state->run(io);
			}
		}

		static auto make(std::unique_ptr<StateBase<Io_>>&& start_state) -> StateMachine<Io_> {
			return {std::move(start_state)};
		}
	};

	template<class T_>
	concept is_io = requires(T_ io) {
		{io.kill_interrupted()} -> std::convertible_to<bool>;
	};

	template<class T_, class Content_, class Io_>
	concept is_setup = requires(T_ setup, Io_& io) {
		{setup(io)} -> std::convertible_to<Content_>;
	};

	template<class T_, class Content_, class Io_>
	concept is_loop_part = requires(T_ loop_part, Content_& content, Io_& io) {
		{[](std::optional<std::unique_ptr<StateBase<Io_>>>){} (
			loop_part(content, io)
		)};
	};

	template<is_io Io_, class Content_, class Setup_, class ... LoopParts_>
	requires (
		is_setup<std::remove_cvref_t<Setup_>, Content_, Io_>
		&& (... && is_loop_part<std::remove_cvref_t<LoopParts_>, Content_, Io_>)
	)
	struct State final : StateBase<Io_> {
		Setup_ setup;
		std::tuple<LoopParts_...> loop_parts;
		std::chrono::duration<double> sleep_duration;

		State(Setup_&& setup, std::tuple<LoopParts_...>&& loop_parts, std::chrono::duration<double> sleep_duration)
			: setup(std::move(setup))
			, loop_parts(std::move(loop_parts))
			, sleep_duration(sleep_duration)
		{}

		auto run(Io_& io) -> std::unique_ptr<StateBase<Io_>> override {
			auto content = this->setup(io);

			while(!io.kill_interrupted()){
				std::this_thread::sleep_for(this->sleep_duration);

				auto next_state = std::optional<std::unique_ptr<StateBase<Io_>>>{};
				auto& loop_parts = this->loop_parts;
				[&]<usize ... indices_>(std::index_sequence<indices_ ...>) {
					((next_state ? next_state : next_state = std::get<indices_>(loop_parts)(content, io)), ...);
				}(std::index_sequence_for<LoopParts_...>{});

				if(next_state.has_value()) {
					return std::move(*next_state);
				}
			}

			return nullptr;
		}
	};

	template<class Io_, class Content_, class Setup_, class ... LoopParts_>
	requires (
		!std::is_reference_v<Content_>
		&& !std::is_reference_v<Setup_>
		&& is_setup<Setup_, std::remove_cvref_t<Content_>, Io_>
		&& (... && is_loop_part<std::remove_cvref_t<LoopParts_>, std::remove_cvref_t<Content_>, Io_>)
	)
	inline auto make_state(Setup_&& setup_parts, std::tuple<LoopParts_ ...>&& loop_parts, std::chrono::duration<double> sleep_duration) {
		return State<Io_, Content_, Setup_, LoopParts_ ...> (
			std::move(setup_parts)
			, std::move(loop_parts)
			, sleep_duration
		);
	}

	template<class Io_, class Run_>
	requires requires (Run_ run, Io_ io) {
		{run(io)} -> std::convertible_to<std::unique_ptr<StateBase<Io_>>>;
	}
	struct NonLoopState final : StateBase<Io_> {
		Run_ runnable;

		NonLoopState(Run_&& runnable) : runnable(std::move(runnable)) {}

		auto run(Io_& io) -> std::unique_ptr<StateBase<Io_>> override {
			return runnable(io);
		}
	};

	template<class Io_, class Run_>
	inline auto make_non_loop_state(Run_&& run) {
		return NonLoopState<Io_, Run_>(std::move(run));
	}
}

namespace nhk24_2nd_ws::state_machine {
	using impl::StateBase;
	using impl::State;
	using impl::StateMachine;
	using impl::make_state;
	using impl::NonLoopState;
	using impl::make_non_loop_state;
}