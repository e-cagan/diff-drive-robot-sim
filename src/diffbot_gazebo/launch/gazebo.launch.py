"""
Launch file for starting gazebo simulator along with robot description.
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

    # Made teleop node a variable
    teleop_node = Node(
        name='teleop_twist_keyboard_node',
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([

        # 1. Launch gazebo simulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            )
        ),

        # 2. Launch our description launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('diffbot_description'), 'launch', 'description.launch.py')
            ),
            launch_arguments={'gui': 'false'}.items()
        ),

        # 3. Spawn the entitiy
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'diffbot',
                '-topic', 'robot_description'
            ],
            output='screen',
        ),

        # 4. Run the teleop_twist_keyboard node to control robot manually
        # teleop_node,

    ])