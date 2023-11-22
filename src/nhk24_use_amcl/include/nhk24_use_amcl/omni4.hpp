#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_2d_msgs/msg/twist2_d.hpp>

#include <CRSLibtmp/Ros2/reporter.hpp>
#include <CRSLibtmp/Ros2/can.hpp>
#include <CRSLibtmp/Motor/shirasu.hpp>
#include <CRSLibtmp/Mechanism/omni_wheel.hpp>
#include <CRSLibtmp/Mechanism/omni_n.hpp>

namespace nhk24_use_amcl::stew::omni4::impl {
	using CRSLib::Ros2::RosReporter;
	using CRSLib::Ros2::CanPillarbox;
	using CRSLib::Motor::Shirasu;
	using CRSLib::Mechanism::OmniWheel;
	using CRSLib::Mechanism::OmniN;

	using OmniWheelT = OmniWheel<Shirasu<CanPillarbox, RosReporter>>;

	struct Omni4 final : rclcpp::Node
	{
		private:
		OmniN<OmniWheelT, OmniWheelT, OmniWheelT, OmniWheelT> omni4;
		rclcpp::Subscription<nav_2d_msgs::msg::Twist2D>::SharedPtr sub;

		public:
		Omni4(const rclcpp::NodeOptions& options)
		: rclcpp::Node{"omni4", options}
		, omni4{make_omni4()}
		, sub{}
		{}

		private:
		static auto make_omni4() -> Omni4
		{
			return Omni4{};
		}
	};
}