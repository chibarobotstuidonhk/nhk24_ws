#!/bin/bash

# LiDAR(frame: laser) -> /scan_nonfiltered 
run_command "urg_node2" "ros2 launch src/nhk24_use_amcl/launch/urg_node2.launch.py"
# /scan_nonfiltered -> /scan
run_command "filter_node" "ros2 run nhk24_use_amcl filter_node"

# MAP -> /map
run_command "map_server" "ros2 run nav2_map_server map_server --ros-args --params-file src/nhk24_use_amcl/launch/map_server.yaml"

# URDF -> TF2 Smth about robot joint
run_command "robot_state_publisher" "ros2 launch src/nhk24_use_amcl/launch/robot_state_publisher.launch.py"

# /can_rx -> /odom, /imu
run_command "odom_check" "ros2 run odom_check odom_check_node"
# /odom, /imu -> TF2 frame: odom
run_command "robot_localization" "ros2 run robot_localization ekf_node --ros-args --params-file src/nhk24_use_amcl/launch/ekf_node.yaml"

# /scan, /map, frame: odom, Smth about robot joints, param: initial_pose -> /amcl_pose, TF2 transform map->odom
run_command "amcl" "ros2 run nav2_amcl amcl --ros-args --params-file src/nhk24_use_amcl/launch/amcl.yaml; exec bash"