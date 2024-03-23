# create tab
run_command "rviz2" "rviz2"
. launch/around_amcl.bash
run_command "can_plugins2" "ros2 launch src/can_plugins2/launch/slcan_bridge_launch.xml"
run_command "filter_node" "ros2 run r2 filter_node"
run_command "r2_node" "ros2 run r2 r2_node; bash"
run_command "joy" "ros2 run joy joy_node"

# lifecycle is now managed by r2_node