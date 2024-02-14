#include <nhk24_use_amcl/ball_chaser.hpp>

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<nhk24_use_amcl::stew::ball_chaser::BallChaser>());
	rclcpp::shutdown();
	return 0;
}