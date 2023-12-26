#!/bin/bash

# # LiDAR(frame: laser) -> /scan_nonfiltered 
run_command "urg_node2" "ros2 launch src/nhk24_use_amcl/launch/urg_node2.launch.py"
# /scan_nonfiltered -> /scan
run_command "filter_node" "ros2 run nhk24_use_amcl filter_node"

# MAP -> /map
run_command "map_server" "ros2 run nav2_map_server map_server --ros-args --params-file src/nhk24_use_amcl/launch/map_server.yaml"

# URDF -> TF2 Smth about robot joint
run_command "robot_state_publisher" "ros2 launch src/nhk24_use_amcl/launch/robot_state_publisher.launch.py"

# # /can_rx -> /odom, /imu
# run_command "odom_check" "ros2 run odom_check odom_check_node"
# # /odom, /imu -> TF2 frame: odom
# run_command "ekf_node" "ros2 launch src/nhk24_use_amcl/launch/ekf_node.launch.py"
# run_command "odometry2024" "ros2 run odometry2024 odom_node"
run_command "genbacat" "ros2 run nhk24_use_amcl genbacat_node"

sleep 1
ros2 topic pub -1 /reset_current_twist geometry_msgs/msg/Twist "linear:
  x: 1.0
  y: 1.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# /scan, /map, frame: odom, Smth about robot joints, param: initial_pose -> /amcl_pose, TF2 transform map->odom
run_command "amcl" "ros2 run nav2_amcl amcl  --ros-args --log-level debug --params-file src/nhk24_use_amcl/launch/amcl.yaml"