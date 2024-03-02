run_command joy "ros2 run joy joy_node"
run_command can_plugins2 "ros2 launch can_plugins2 slcan_bridge_launch.xml"
run_command omni4 "ros2 run nhk24_use_amcl omni4_node"

# URDF -> TF2 Smth about robot joint
run_command "robot_state_publisher" "ros2 launch launch/robot_state_publisher.launch.py"

run_command ball_chaser "ros2 run nhk24_use_amcl ball_chaser_node"
run_command ball_detecter "ros2 run image_sensing ball_detecter"