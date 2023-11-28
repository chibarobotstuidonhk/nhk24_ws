#pragma once

#include <cstdint>
#include <cmath>
#include <functional>
#include <utility>
#include <chrono>
#include <array>
#include <numbers>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <can_plugins2/msg/frame.hpp>

#include "logicool.hpp"
#include "std_type.hpp"
#include "vec2d.hpp"
#include "twist2d.hpp"
#include "shirasu.hpp"

namespace nhk24_use_amcl::stew::omni4::impl {
	using namespace std::chrono_literals;
	using namespace crs_lib::integer_types;
	using crs_lib::ros2::logicool::Logicool;
	
	using nhk24_use_amcl::stew::vec2d::Vec2d;
	using nhk24_use_amcl::stew::twist2d::Twist2d;

	enum class State : u8 {
		EmergencyStop,
		Manual,
		Auto
	};

	struct MotorSpeedFixer final {
		/// @todo 以下の値を適切に設定する
		static constexpr double max_velocity = 7.0;  // 最大速度[rad/s]
		static constexpr double max_acceleration = 7.0;  // 最大加速度[rad/s^2]

		double last_speed{0.0};

		constexpr auto update(double speed, const double dt) noexcept -> double {
			if(speed * speed > max_velocity * max_velocity) {
				speed = std::signbit(speed) ? -max_velocity : max_velocity;
			}

			const double diff_speed = speed - last_speed;
			const double max_diff_velocity = max_acceleration * dt;
			if(diff_speed * diff_speed > max_diff_velocity * max_diff_velocity) {
				speed = last_speed + (std::signbit(diff_speed) ? -max_diff_velocity : max_diff_velocity);
			}

			last_speed = speed;
			return speed;
		}
	};

	struct BodySpeedFixer final {
		/// @todo 以下の値を適切に設定する
		static constexpr double max_linear = 1.0;  // 最大速度[m/s]
		static constexpr double max_angular = std::numbers::pi / 3.0;  // 最大角速度[rad/s]
		static constexpr double max_linear_acceleration = 1.0;  // 最大加速度[m/s^2]

		Twist2d last_twist{{0.0, 0.0}, 0.0};

		constexpr auto update(const Twist2d& twist, const double dt) noexcept -> Twist2d {
			const double max_diff_linear = max_linear_acceleration * dt;
			
			auto linear = twist.linear;
			if(const double norm2 = linear.norm2(); norm2 > max_linear * max_linear) {
				linear *= max_linear / std::sqrt(norm2);  // max_linear * max_linearが十分大きいと仮定
			}

			auto diff_linear = linear - last_twist.linear;
			if(const double norm2 = diff_linear.norm2(); norm2 > max_diff_linear * max_diff_linear) {
				diff_linear *= max_diff_linear / std::sqrt(norm2);  // max_diff_linear * max_diff_linearが十分大きいと仮定
				linear = last_twist.linear + diff_linear;
			}

			auto angular = twist.angular;
			if(angular * angular > max_angular * max_angular) {
				angular *= std::signbit(angular) ? -max_angular : max_angular;
			}

			return Twist2d{linear, angular};
		}
	};

	struct Omni4 final : rclcpp::Node {
		private:
		static constexpr std::array<u32, 4> ids = {0x400u, 0x404u, 0x408u, 0x40Cu};  // 第一象限から反時計回りに見ていく

		rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_pub;
		Logicool logicool;

		State state{State::EmergencyStop};
		Twist2d auto_twist_msg{{0.0, 0.0}, 0.0};
		BodySpeedFixer body_speed_fixer{};
		std::array<MotorSpeedFixer, 4> motor_speed_fixers{};
		rclcpp::Time last_time{this->now()};

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub{};
		rclcpp::TimerBase::SharedPtr timer{};

		public:
		Omni4(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: Node("nhk24_0th_omni4", options),
			can_pub(create_publisher<can_plugins2::msg::Frame>("can_tx", 10)),
			logicool{*this, "joy", {}, 10}
		{
			for(u32 i = 0; i < 4; ++i) {
				can_pub->publish(shirasu::command_frame(ids[i], shirasu::Command::recover_velocity));
			}

			twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("body_twist", 10, std::bind(&Omni4::twist_callback, this, std::placeholders::_1));
			timer = this->create_wall_timer(10ms, std::bind(&Omni4::timer_callback, this));
		}

		private:
		void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
			if(state == State::Auto) {
				auto_twist_msg = Twist2d{{msg->linear.x, msg->linear.y}, msg->angular.z};
			}
		}

		void timer_callback() {
			const auto now = this->now();
			const auto dt = (now - last_time).seconds();

			if(logicool.is_pushed_down(Logicool::Buttons::back)) {
				state = State::EmergencyStop;
			}
			else if(logicool.is_pushed_down(Logicool::Buttons::start)) {
				state = State::Manual;
			}
			else if(logicool.is_pushed_down(Logicool::Buttons::a)) {
				state = State::Auto;
			}

			switch(state) {
				case State::EmergencyStop:
				{
					update(Twist2d{{0.0, 0.0}, 0.0}, dt);
					break;
				}
				
				case State::Manual:
				{
					auto target_twist = Twist2d {
							{
							BodySpeedFixer::max_linear * std::sqrt(0.5) * logicool.get_axis(Logicool::Axes::l_stick_LR)
							, BodySpeedFixer::max_linear * std::sqrt(0.5) * logicool.get_axis(Logicool::Axes::l_stick_UD)
						}
						, BodySpeedFixer::max_angular * logicool.get_axis(Logicool::Axes::r_stick_LR)
					};
					update(target_twist, dt);
					break;
				}

				case State::Auto:
				{
					update(auto_twist_msg, dt);
					break;
				}
			}
		}

		void update(const Twist2d& target_twist, const double dt) {
			const auto fixed_body_twist = body_speed_fixer.update(target_twist, dt);
			const auto motor_speeds = calc_motor_speeds(fixed_body_twist);
			for(u32 i = 0; i < 4; ++i) {
				const auto motor_speed = motor_speed_fixers[i].update(motor_speeds[i], dt);
				const auto msg = shirasu::target_frame(ids[i], motor_speed);
				can_pub->publish(msg);
			}
		}

		static constexpr auto calc_motor_speeds(const Twist2d& fixed_body_twist) noexcept -> std::array<double, 4> {
			/// @todo 以下の値を適切に設定する
			constexpr double center_to_wheel = 0.7071;  // 中心から駆動輪までの距離[m](default: 0.5 * sqrt(2))
			constexpr double wheel_radius = 0.150;  // 駆動輪の半径[m]
			constexpr double wheel_to_motor_ratio = 1.0;  // 駆動輪からモーターへの倍速比

			return []<u32 ... i>(std::integer_sequence<u32, i...>, const Twist2d& fixed_body_twist) {
				return std::array<double, 4> {
					[](const Twist2d& fixed_body_twist) -> double {
						auto v = dot(fixed_body_twist.linear, rot(Vec2d{1, 0}, std::numbers::pi / 4.0 * i)) + fixed_body_twist.angular * center_to_wheel;
						return v / wheel_radius * wheel_to_motor_ratio;
					}(fixed_body_twist) ...
				};
			}(std::make_integer_sequence<u32, 4>{}, fixed_body_twist);
		}
	};
}

namespace nhk24_use_amcl::stew::omni4 {
	using nhk24_use_amcl::stew::omni4::impl::Omni4;
}