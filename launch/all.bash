# create tab
run_command "rviz2" "rviz2"
. launch/around_amcl.bash
run_command "can_plugins2" "ros2 launch src/can_plugins2/launch/slcan_bridge_launch.xml"
sleep 1
. launch/omni4.bash
. launch/pacman.bash

# lifecycle manage
# urg_node2 automatically transit to active by default.

# for map_server
ros2 lifecycle set /map_server 1
ros2 lifecycle set /map_server 3

# for amcl
ros2 lifecycle set /amcl 1
ros2 lifecycle set /amcl 3
