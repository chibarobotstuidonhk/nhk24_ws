#include <nhk24_use_amcl/omni4.hpp>

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<nhk24_use_amcl::stew::omni4::Omni4>());
	rclcpp::shutdown();
	return 0;
}