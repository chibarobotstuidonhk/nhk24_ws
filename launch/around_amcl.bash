# # LiDAR(frame: laser) -> /scan_nonfiltered 
run_command "urg_node2" "ros2 launch launch/urg_node2.launch.py"
# /scan_nonfiltered -> /scan
run_command "filter_node" "ros2 run r2 filter_node"

# MAP -> /map
run_command "map_server" "ros2 run nav2_map_server map_server --ros-args --params-file launch/map_server.yaml"
# run_command "map_server" "ros2 run nav2_map_server map_server"  # これではダメ、ダミーでも良いのでyamlファイルを指定しないと動かない

# URDF -> TF2 Smth about robot joint
run_command "robot_state_publisher" "ros2 launch launch/robot_state_publisher.launch.py"

# # /can_rx -> TF2 frame: odom
run_command "odometry2024" "ros2 run odometry2024 odom_node"

sleep 1

# /scan, /map, frame: odom, Smth about robot joints, param: initial_pose -> /amcl_pose, TF2 transform map->odom
run_command "amcl" "ros2 run nav2_amcl amcl  --ros-args --log-level debug --params-file launch/amcl.yaml"