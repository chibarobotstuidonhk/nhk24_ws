#include <nhk24_use_amcl/pacman.hpp>

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<nhk24_use_amcl::stew::pacman::PacMan>());
	rclcpp::shutdown();
	return 0;
}