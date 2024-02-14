/*
XInputモードにしか対応していない。
DirectInputモードやその切り替えを十分に調べたら実装するかも。
まあ、コントローラーが保持する状態はx/Dモードのどちらかなのか以外にはない(はず)なので、間違えて切り替えてもきっと大丈夫。
*/


#pragma once

#include <cstdint>

#include "std_type.hpp"
#include "joy_to_key_button.hpp"

namespace crs_lib::ros2::logicool
{
	struct LogicoolXInputKeyMap final
	{
		struct Axes final
		{
			enum Enum : u8
			{
				l_stick_LR = 0,
				l_stick_UD,
				l_trigger,
				r_stick_LR,
				r_stick_UD,
				r_trigger,
				cross_LR,
				cross_UD,

				N
			};
		};

		struct Buttons final
		{
			enum Enum : u8
			{
				a = 0,
				b,
				x,
				y,
				lb,
				rb,
				back,
				start,
				dummmmmyyyyyy,
				l_push,
				r_push,

				N
			};
		};
	};

	using Logicool = joy_to_key_button::JoyToKeyButton<LogicoolXInputKeyMap>;
}