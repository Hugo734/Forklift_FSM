"""Launch the forklift FPGA driver node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = os.path.join(
        get_package_share_directory('forklift_ros2'),
        'config', 'forklift_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='forklift_ros2',
            executable='forklift_driver',
            name='forklift_driver',
            parameters=[params_file],
            output='screen',
        )
    ])
