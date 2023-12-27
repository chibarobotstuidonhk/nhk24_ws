# create tab
run_command "rviz2" "rviz2"
run_command "odometry2024" "ros2 run odometry2024 odom_node"
run_command "can_plugins2" "ros2 launch src/can_plugins2/launch/slcan_bridge_launch.xml"
# run_command "tf2_ros" "ros2 run tf2_ros tf2_echo odom base_link"
