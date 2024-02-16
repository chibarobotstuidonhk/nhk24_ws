#pragma once

#include <optional>
#include <vector>
#include <utility>
#include <functional>
#include <string_view>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <nhk24_utils/msg/balls.hpp>
#include <nhk24_utils/msg/twist2d.hpp>

#include <nhk24_utils/std_type.hpp>
#include <nhk24_utils/pid.hpp>
#include <nhk24_utils/vec2d.hpp>
#include <nhk24_utils/vec3d.hpp>
#include <nhk24_utils/twist2d.hpp>
#include <nhk24_utils/geometry_msgs_convertor.hpp>
#include <nhk24_utils/ball.hpp>

namespace nhk24_use_amcl::stew::ball_chaser::impl {
	using namespace crs_lib::integer_types;
	using nhk24_utils::stew::pid::Pid;
	using nhk24_utils::stew::vec2d::Vec2d;
	using nhk24_utils::stew::vec3d::Vec3d;
	using nhk24_utils::stew::twist2d::Twist2d;
	using nhk24_utils::stew::geometry_msgs_convertor::MsgConvertor;
	using nhk24_utils::stew::ball::Ball;

	struct BallChaser final : rclcpp::Node {
		std::vector<Ball> balls{};
		Pid<double, double> pid_dis{};
		Pid<double, double> pid_th{};
		double distance_to_ball{};
		double angle_to_ball{};

		tf2_ros::Buffer tf2_buffer;
		tf2_ros::TransformListener tf2_listener;
		
		rclcpp::Publisher<nhk24_utils::msg::Twist2d>::SharedPtr body_twist_pub;
		rclcpp::Subscription<nhk24_utils::msg::Balls>::SharedPtr ball_pos_sub;
		rclcpp::TimerBase::SharedPtr timer;

		BallChaser(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{})
			: Node("nhk24_1st_ball_chaser", options)
			, tf2_buffer{this->get_clock()}
			, tf2_listener{tf2_buffer}
			, body_twist_pub {
				this->create_publisher<nhk24_utils::msg::Twist2d>("body_twist_ball", 1)
			}
			, ball_pos_sub {
				this->create_subscription<nhk24_utils::msg::Balls> (
					"ball_pos"
					, 1
					, [this](const nhk24_utils::msg::Balls::SharedPtr msg) {
						std::vector<Ball> balls(std::move(this->balls));  // 並行処理で死ぬ
						balls.clear();
						balls.reserve(msg->balls.size());

						for(size_t i = 0; i < balls.size(); ++i) {
							balls[i] = Ball::from_msg(msg->balls[i]);
						}
						this->balls = std::move(balls);
					}
				)
			}
			, timer {
				this->create_wall_timer (
					std::chrono::milliseconds(100)
					, std::bind(&BallChaser::timer_callback, this)
				)
			}
		{
			this->declare_parameter("pid_dis_p", 1.0);
			this->declare_parameter("pid_dis_i", 0.0);
			this->declare_parameter("pid_dis_d", 0.0);
			this->declare_parameter("pid_dis_max_integral", 10.0);
			
			this->declare_parameter("pid_th_p", 1.0);
			this->declare_parameter("pid_th_i", 0.0);
			this->declare_parameter("pid_th_d", 0.0);
			this->declare_parameter("pid_th_max_integral", 10.0);

			this->declare_parameter("distance_to_ball", 1.0);
			this->declare_parameter("angle_to_ball", 0.1508);

			this->update_params();
		}

		void timer_callback() {
			const auto current_pose = [this]() -> std::optional<Twist2d> {
				const auto transform = this->lookup_transform("map", "base_link");
				if(transform) {
					return std::optional<Twist2d>{std::in_place, this->twist2d_from_tf2(transform->transform)};
				}
				else return {std::nullopt};
			}();

			const auto base_to_camera = this->lookup_transform("base_link", "camera_link");

			const auto& balls = this->balls;

			if(!current_pose || !base_to_camera || balls.empty()) {
				return;
			}

			// 手を横に突き出して、突き出した手がボールに向くように体ごと回転し、手の向くほうに移動している感じ
			const auto [dis_e, ang_e] = calc_pose_error(balls, *current_pose, *base_to_camera, this->distance_to_ball, this->angle_to_ball);

			const auto dis_out = this->pid_dis.update(dis_e, 0.1);
			const auto ang_out = this->pid_th.update(ang_e, 0.1);

			const auto cmd_vel = Twist2d {
				Vec2d {
					dis_out * std::cos(this->angle_to_ball)
					, dis_out * std::sin(this->angle_to_ball)
				}
				, ang_out
			};

			this->body_twist_pub->publish(cmd_vel.to_msg<nhk24_utils::msg::Twist2d>());
		}

		/// @todo: implement the logic to choose a ball and best pose to pick it up
		static auto calc_pose_error (
			const std::vector<Ball>& balls
			, const Twist2d& current_pose
			, const geometry_msgs::msg::TransformStamped& base_to_camera
			, const double distance_to_ball
			, const double angle_to_ball
		) -> std::tuple<double, double> {
			// とりあえず1つボールを選ぶ
			const auto ball = balls[0];
			
			// ボールのbase_linkからの座標を得る
			geometry_msgs::msg::PointStamped ball_from_camera{};
			ball_from_camera.header.frame_id = "camera_link";
			ball_from_camera.point = ball.position.to_msg<geometry_msgs::msg::Point>();
			geometry_msgs::msg::PointStamped ball_from_base{};
			tf2::doTransform(ball_from_camera, ball_from_base, base_to_camera);
			const auto ball_pos = Vec3d::from_msg<geometry_msgs::msg::Point>(ball_from_base.point);

			const auto angle = std::atan2(ball_pos.y, ball_pos.x) - current_pose.angular;
			const auto distance = std::hypot(ball_pos.x, ball_pos.y);

			return  std::tuple {
				distance - distance_to_ball
				, angle - angle_to_ball
			};
		}

		auto lookup_transform(const std::string_view from_frame, const std::string_view to_frame) const -> std::optional<geometry_msgs::msg::TransformStamped> {
			try {
				return tf2_buffer.lookupTransform(from_frame.data(), to_frame.data(), tf2::TimePointZero);
			} catch(const tf2::TransformException& e) {
				RCLCPP_ERROR(this->get_logger(), "ball_chaser:  %s", e.what());
				return std::nullopt;
			}
		}

		static auto twist2d_from_tf2(const geometry_msgs::msg::Transform& transform) -> Twist2d {
			const auto v = transform.translation;
			double roll, pitch, yaw;
			tf2::Matrix3x3{MsgConvertor<tf2::Quaternion, geometry_msgs::msg::Quaternion>::fromMsg(transform.rotation)}
				.getRPY(roll, pitch, yaw);
			return Twist2d{v.x, v.y, yaw};
		}

		void update_params() {
			{
				const auto p = this->get_parameter("pid_dis_p").as_double();
				const auto i = this->get_parameter("pid_dis_i").as_double();
				const auto d = this->get_parameter("pid_dis_d").as_double();
				const auto max_integral = this->get_parameter("pid_dis_max_integral").as_double();
				this->pid_dis = Pid<double, double>::make(p, i, d, max_integral);
			}
			{
				const auto p = this->get_parameter("pid_th_p").as_double();
				const auto i = this->get_parameter("pid_th_i").as_double();
				const auto d = this->get_parameter("pid_th_d").as_double();
				const auto max_integral = this->get_parameter("pid_th_max_integral").as_double();
				this->pid_th = Pid<double, double>::make(p, i, d, max_integral);
			}
			{
				this->distance_to_ball = this->get_parameter("distance_to_ball").as_double();
				this->angle_to_ball = this->get_parameter("angle_to_ball").as_double();
			}
		}
	};
}

namespace nhk24_use_amcl::stew::ball_chaser {
	using impl::BallChaser;
}