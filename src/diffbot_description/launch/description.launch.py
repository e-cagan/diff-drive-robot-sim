"""
Launch file for bringing up descriptions, visualizations and so on.
"""

import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    A function to generate launch description
    """

    # Take the package directory
    package_dir = get_package_share_directory('diffbot_description')

    # Xacro file path
    xacro_path = os.path.join(package_dir, 'urdf', 'diffbot.urdf.xacro')
    
    # Convert xacro to urdf
    urdf_file = xacro.process_file(xacro_path).toxml()

    # Declare a parameter to open gui or not
    gui = LaunchConfiguration('gui')

    return LaunchDescription([

        # Give launch description our gui argument
        DeclareLaunchArgument('gui', default_value='True'),
        
        # 1. Publish the state using robot state publisher
        Node(
            name='robot_state_publisher_node',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': urdf_file
            }],
        ),

        # 2. Add joint_state_publisher_gui node to visualize and test the description
        Node(
            name='joint_state_publisher_gui_node',
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),

        # 3. Add rviz node to visualize the description also to save the rviz config file
        Node(
            name='rviz_node',
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(gui),
        ),

    ])