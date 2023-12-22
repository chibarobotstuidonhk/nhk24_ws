#include <nhk24_use_amcl/filter_node.hpp>

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<nhk24_use_amcl::stew::filter_node::FilterNode>());
	rclcpp::shutdown();
	return 0;
}