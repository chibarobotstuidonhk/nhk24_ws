import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    ekf_node_params_yaml = "src/nhk24_use_amcl/launch/ekf_node.yaml"

    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[ekf_node_params_yaml, {'use_sim_time': False}]
    )

    return launch.LaunchDescription([
        robot_localization_node,
    ])
