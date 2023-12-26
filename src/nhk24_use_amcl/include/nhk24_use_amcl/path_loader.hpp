#pragma once

#include <functional>
#include <string>
#include <fstream>
#include <sstream>
#include <tuple>
#include <utility>
#include <optional>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nhk24_use_amcl/msg/path.hpp>

namespace nhk24_use_amcl::stew::path_loader {
	struct PathLoader final : rclcpp::Node {
		rclcpp::Publisher<nhk24_use_amcl::msg::Path>::SharedPtr path_pub;
		rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr path_sub;

		PathLoader()
		: rclcpp::Node("path_loader")
		, path_pub(create_publisher<nhk24_use_amcl::msg::Path>("path", 10))
		, path_sub(create_subscription<std_msgs::msg::Empty>("path_load", 10, std::bind(&PathLoader::path_load, this, std::placeholders::_1)))
		{
			this->declare_parameter<std::string>("path_file", "path.txt");
		}

		void path_load(const std_msgs::msg::Empty::SharedPtr) {
			std::string path_file;
			this->get_parameter("path_file", path_file);

			std::ifstream ifs(path_file);
			if (!ifs) {
				RCLCPP_ERROR(this->get_logger(), "Failed to open path file: %s", path_file.c_str());
				return;
			}

			constexpr auto read_line = []<class ... T>(auto& ifs, const auto& error_handler) -> std::optional<std::tuple<T ...>> {
				std::string line{};
				if(!std::getline(ifs, line)) return {std::nullopt};
				
				std::istringstream iss(line);
				std::tuple<T ...> ret{};

				const bool is_success = []<size_t ... indices>(std::index_sequence<indices ...>, auto& iss, auto& ret) -> bool {
					return static_cast<bool>((iss >> ... >> std::get<indices>(ret)));
				}(std::make_index_sequence<std::tuple_size_v<decltype(ret)> >{}, iss, ret);

				if(!is_success) {
					error_handler(line);
					return {std::nullopt};
				}
				else {
					return {std::move(ret)};
				}
			};

			const auto print_error = [this](const std::string& err) {
				RCLCPP_ERROR(this->get_logger(), "Failed to parse path line: %s", err.c_str());
			};

			nhk24_use_amcl::msg::Path path{};
			if (auto res = read_line.template operator()<size_t, double>(ifs, print_error); !res) {
				RCLCPP_ERROR(this->get_logger(), "Failed to parse path file: It is empty.");
			}
			else {
				path.path.reserve(std::get<0>(*res));
				path.goal_radius = std::get<1>(*res);
				for(size_t i = 0; i < std::get<0>(*res); ++i) if(auto res = read_line.template operator()<double, double, double>(ifs, print_error); res) {
					const auto [x, y, yaw] = std::move(*res);
					
					path.path.emplace_back();
					path.path.back().linear.x = x;
					path.path.back().linear.y = y;
					path.path.back().angular = yaw;
					path.time = this->now();
				}
			}

			path_pub->publish(path);
		}
	};
}