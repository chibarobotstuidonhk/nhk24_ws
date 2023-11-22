#include <nhk24_use_amcl/laser_filter.hpp>

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<nhk24_use_amcl::stew::laser_filter::LaserFilter>());
	rclcpp::shutdown();
	return 0;
}