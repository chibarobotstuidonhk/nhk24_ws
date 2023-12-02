#include <nhk24_use_amcl/path_loader.hpp>

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<nhk24_use_amcl::stew::path_loader::PathLoader>());
	rclcpp::shutdown();
	return 0;
}