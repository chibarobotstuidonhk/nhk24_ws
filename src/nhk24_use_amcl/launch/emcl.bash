run_command "rviz2" "rviz2"

# # LiDAR(frame: laser) -> /scan_nonfiltered 
run_command "urg_node2" "ros2 launch src/nhk24_use_amcl/launch/urg_node2.launch.py"
# /scan_nonfiltered -> /scan
run_command "filter_node" "ros2 run nhk24_use_amcl filter_node"

# URDF -> TF2 Smth about robot joint
run_command "robot_state_publisher" "ros2 launch src/nhk24_use_amcl/launch/robot_state_publisher.launch.py"


# # /can_rx -> /odom, /imu
# run_command "odom_check" "ros2 run odom_check odom_check_node"
# # /odom, /imu -> TF2 frame: odom
# run_command "ekf_node" "ros2 launch src/nhk24_use_amcl/launch/ekf_node.launch.py"
# /can_rx -> TF2 frame: odom
run_command "odometry2024" "ros2 run odometry2024 odom_node"

run_command "emcl2_ros2" "ros2 launch src/nhk24_use_amcl/launch/emcl2.launch.py"