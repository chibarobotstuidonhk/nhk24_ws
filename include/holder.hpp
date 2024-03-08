/**
 * @file holder.hpp
 * @brief 様々なモノを型に持たせるためのホルダーたち。
 */

#pragma once

#include <concepts>

namespace nhk24_2nd_ws::holder::impl {

	struct CustomableTypeHolderMarker {};
	struct TypeHolderMarker : CustomableTypeHolderMarker {};
	template<class T_>
	struct TypeHolder final : TypeHolderMarker {
		using T = T_;
	};
	template<class T_>
	concept type_holder_like = requires {
		requires std::derived_from<T_, CustomableTypeHolderMarker>;
		typename T_::T;
	};
	template<class T_>
	concept type_holder = type_holder_like<T_> && requires {
		requires std::derived_from<T_, TypeHolderMarker>;
	};

	struct CustomableValueHolderMarker {};
	struct ValueHolderMarker : CustomableTypeHolderMarker {};
	template<auto v_>
	struct ValueHolder final : ValueHolderMarker {
		static constexpr auto v = v_;
	};
	template<class T_>
	concept value_holder_like = requires {
		requires std::derived_from<T_, CustomableValueHolderMarker>;
		T_::v;
	};
	template<class T_>
	concept value_holder = value_holder_like<T_> && requires {
		requires std::derived_from<T_, ValueHolderMarker>;
	};

	struct CustomableTemplateHolderMarker {};
	struct TemplateHolderMarker : CustomableTemplateHolderMarker {};
	template<template<class ...> class Temp_>
	struct TemplateHolder final : TemplateHolderMarker {
		template<class ... Ts_>
		using Temp = Temp_<Ts_ ...>;
	};
	template<class T_>
	concept template_holder_like = requires {
		requires std::derived_from<T_, CustomableTemplateHolderMarker>;
	};

	template<template<class ...> class Holder_>
	struct JoinHolder final {
		template<class ... L_, class ... R_>
		static auto join(TypeHolder<Holder_<L_ ...>>, TypeHolder<Holder_<R_ ...>>) -> TypeHolder<Holder_<L_ ..., R_ ...>> {
			return {};
		}

		template<class L_, class ... R_>
		static auto join(TypeHolder<L_>, TypeHolder<Holder_<R_ ...>>) -> TypeHolder<Holder_<L_, R_ ...>> {
			return {};
		}

		template<class ... L_, class R_>
		static auto join(TypeHolder<Holder_<L_ ...>>, TypeHolder<R_>) -> TypeHolder<Holder_<L_ ..., R_>> {
			return {};
		}

		template<class L_, class R_>
		static auto join(TypeHolder<L_>, TypeHolder<R_>) -> TypeHolder<Holder_<L_, R_>> {
			return {};
		}
	};
}

namespace nhk24_2nd_ws::holder {
	using impl::CustomableTypeHolderMarker;
	using impl::TypeHolder;
	using impl::type_holder_like;
	using impl::type_holder;
	using impl::CustomableValueHolderMarker;
	using impl::ValueHolder;
	using impl::value_holder_like;
	using impl::value_holder;
	using impl::CustomableTemplateHolderMarker;
	using impl::TemplateHolder;

	using impl::JoinHolder;
}