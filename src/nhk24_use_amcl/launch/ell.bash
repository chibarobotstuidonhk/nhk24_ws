# create tab
run_command "rviz2" "rviz2"
sleep 1
. src/nhk24_use_amcl/launch/emcl.bash
# run_command "can_plugins2" "ros2 launch src/can_plugins2/launch/slcan_bridge_launch.xml"
