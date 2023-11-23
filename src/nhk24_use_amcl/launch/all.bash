run_command() {
    gnome-terminal --tab --title="$1" -- bash -c ". bash/setup_install.bash; $2"
}

run_command "rviz2" "rviz2"
. src/nhk24_use_amcl/launch/around_amcl.bash
run_command "can_plugins2" "ros2 launch src/can_plugins2/launch/slcan_bridge_launch.xml"
