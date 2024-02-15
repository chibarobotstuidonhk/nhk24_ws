#pragma once

#include <cstdint>
#include <cmath>
#include <functional>
#include <utility>
#include <chrono>
#include <array>
#include <numbers>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <can_plugins2/msg/frame.hpp>

#include <nhk24_utils/logicool.hpp>
#include <nhk24_utils/std_type.hpp>
#include <nhk24_utils/vec2d.hpp>
#include <nhk24_utils/twist2d.hpp>
#include "shirasu.hpp"

namespace nhk24_use_amcl::stew::omni4::impl {
	using namespace std::chrono_literals;
	using namespace crs_lib::integer_types;
	using crs_lib::ros2::logicool::Logicool;
	
	using nhk24_utils::stew::vec2d::Vec2d;
	using nhk24_utils::stew::twist2d::Twist2d;

	enum class State : u8 {
		EmergencyStop,
		Manual,
		Auto,
		BallChaser,
	};

	struct MotorSpeedFixer final {
		/// @todo 以下の値を適切に設定する -> したつもり
		static constexpr double max_velocity = 500.0 * 0.5;  // 最大速度[rad/s]
		static constexpr double max_acceleration = 500.0 * 0.3;  // 最大加速度[rad/s^2]

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
		/// @todo 以下の値を適切に設定する -> したつもり
		static constexpr double max_linear = 1.0 * 0.3;  // 最大速度[m/s]
		static constexpr double max_angular = std::numbers::pi / 3.0 * 0.3;  // 最大角速度[rad/s]
		static constexpr double max_linear_acceleration = 1.0 * 0.1;  // 最大加速度[m/s^2]

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

	inline constexpr auto mkopt(auto&& x) {
		return std::make_optional(std::forward<decltype(x)>(x));
	}

	struct Omni4 final : rclcpp::Node {
		private:
		static constexpr std::array<std::optional<u32>, 4> ids = {mkopt(0x120), mkopt(0x144u), mkopt(0x150u), mkopt(0x110)};  // 第一象限から反時計回りに見ていく

		/// @todo 以下の値を適切に設定する -> したつもり
		static constexpr double center_to_wheel = 0.504;  // 中心から駆動輪までの距離[m](default: 実測値(雑)の504mm)
		static constexpr double wheel_radius = 0.060;  // 駆動輪の半径[m](雑)
		static constexpr double wheel_to_motor_ratio = 33.45;  // 駆動輪からモーターへの倍速比

		State state{State::EmergencyStop};
		Twist2d auto_twist_msg{{0.0, 0.0}, 0.0};
		BodySpeedFixer body_speed_fixer{};
		std::array<MotorSpeedFixer, 4> motor_speed_fixers{};
		rclcpp::Time last_time{this->now()};

		rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_pub;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_ball_pub;
		Logicool logicool;

		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub{};
		rclcpp::TimerBase::SharedPtr timer{};

		public:
		Omni4(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: Node("nhk24_1st_omni4", options),
			can_pub(create_publisher<can_plugins2::msg::Frame>("can_tx", 10)),
			cmd_vel_pub(create_publisher<geometry_msgs::msg::Twist>("actual_cmd_vel", 1)),
			logicool{*this, "joy", {}, 10}
		{
			twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("body_twist", 10, std::bind(&Omni4::twist_callback, this, std::placeholders::_1));
			twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("body_twist_ball", 10, std::bind(&Omni4::twist_ball_callback, this, std::placeholders::_1));
			timer = this->create_wall_timer(10ms, std::bind(&Omni4::timer_callback, this));
		}

		private:
		void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
			if(state == State::Auto) {
				auto_twist_msg = Twist2d::from_msg<geometry_msgs::msg::Twist>(*msg);
			}
		}

		void twist_ball_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
			if(state == State::BallChaser) {
				auto_twist_msg = Twist2d::from_msg<geometry_msgs::msg::Twist>(*msg);
			}
		}

		void timer_callback() {
			const auto now = this->now();
			const auto dt = (now - last_time).seconds();

			if(logicool.is_pushed_down(Logicool::Buttons::back)) {
				state = State::EmergencyStop;
				change_mode(shirasu::Command::shutdown);
			}
			else if(logicool.is_pushed_down(Logicool::Buttons::start)) {
				state = State::Manual;
				change_mode(shirasu::Command::recover_velocity);
			}
			else if(logicool.is_pushed_down(Logicool::Buttons::a)) {
				state = State::Auto;
				change_mode(shirasu::Command::recover_velocity);
			}
			else if(logicool.is_pushed_down(Logicool::Buttons::b)) {
				state = State::BallChaser;
				change_mode(shirasu::Command::recover_velocity);
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
							-BodySpeedFixer::max_linear * std::sqrt(0.5) * logicool.get_axis(Logicool::Axes::l_stick_LR)
							, BodySpeedFixer::max_linear * std::sqrt(0.5) * logicool.get_axis(Logicool::Axes::l_stick_UD)
						}
						, BodySpeedFixer::max_angular * logicool.get_axis(Logicool::Axes::r_stick_LR)
					};
					update(target_twist, dt);
					break;
				}

				case State::Auto:
				case State::BallChaser:
				{
					update(auto_twist_msg, dt);
					break;
				}
			}
		}

		void update(const Twist2d& target_twist, const double dt) {
			const auto fixed_body_twist = body_speed_fixer.update(target_twist, dt);
			const auto motor_speeds = calc_motor_speeds(fixed_body_twist);
			Twist2d actual_twist{{0.0, 0.0}, 0.0};
			for(u32 i = 0; i < 4; ++i) {
				const auto motor_speed = motor_speed_fixers[i].update(motor_speeds[i], dt);
				
				if(ids[i]) {
					const auto msg = shirasu::target_frame(*ids[i], motor_speed);
					can_pub->publish(msg);
					// rclcpp::sleep_for(1ms);
				}

				const auto v = motor_speed * wheel_radius / wheel_to_motor_ratio;
				actual_twist.linear += v * rot(Vec2d{1, 0}, std::numbers::pi / 4.0 * i);
				actual_twist.angular += v / center_to_wheel;
			}
			actual_twist *= (1.0 / 4.0);
			cmd_vel_pub->publish(actual_twist.to_msg<geometry_msgs::msg::Twist>());
		}

		void change_mode(const shirasu::Command cmd) {
			for(u32 i = 0; i < 4; ++i) {
				if(ids[i]) {
					can_pub->publish(shirasu::command_frame(*ids[i], cmd));
					rclcpp::sleep_for(1ms);
				}
			}
		} 

		static constexpr auto calc_motor_speeds(const Twist2d& fixed_body_twist) noexcept -> std::array<double, 4> {
			return []<u32 ... i>(std::integer_sequence<u32, i...>, const Twist2d& fixed_body_twist) {
				return std::array<double, 4> {
					[](const Twist2d& fixed_body_twist) -> double {
						auto v = dot(fixed_body_twist.linear, rot(Vec2d{0, 1}, std::numbers::pi / 4.0 + std::numbers::pi / 2.0 * i)) + fixed_body_twist.angular * center_to_wheel;
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