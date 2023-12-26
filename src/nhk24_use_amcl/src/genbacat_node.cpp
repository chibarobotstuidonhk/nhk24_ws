#include <nhk24_use_amcl/genbacat_node.hpp>

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<nhk24_use_amcl::stew::genbacat_node::GenbacatNode>());
	rclcpp::shutdown();
	return 0;
}