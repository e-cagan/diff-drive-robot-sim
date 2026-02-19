"""
Launch file for bringing up the entire project.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    A function to generate launch description
    """

    return LaunchDescription([

        # 1. Launch custom gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('diffbot_gazebo'), 'launch', 'gazebo.launch.py')
            )
        ),

        # 2. Run the square driver node
        Node(
            name='square_driver_node',
            package='diffbot_control',
            executable='square_driver_node',
        ),

    ])