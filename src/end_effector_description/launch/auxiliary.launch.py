import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# This file launches the servers required for the demos used in this project, namely:
# -> camera_node
# -> gripper_server
# -> moveit_path_planner_server


def gripper_server_launch():
    return Node(
            package='gripper',
            executable='gripper_server',
            name='gripper_server_node',
            output='screen',
        )

def moveit_planner_launch():
    return Node(
            package='moveit_path_planner',
            executable='moveit_path_planning_server',
            name='moveit_path_planning_server_node',
            output='screen',
        )

def camera_node_launch():
    return Node(
            package='camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
        )

def generate_launch_description():
    return LaunchDescription([
        gripper_server_launch(),
        moveit_planner_launch(),
        camera_node_launch(),
    ])