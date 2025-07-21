import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription
import xacro
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name = 'end_effector_description'
    xacro_path = 'urdf/end_effector_only.xacro'
    rviz_path = 'rviz/end_effector_only.rviz'
	
    xacro_file = os.path.join(get_package_share_directory(package_name), xacro_path)
    xacro_raw_description = xacro.process_file(xacro_file).toxml()

    rviz_file = os.path.join(get_package_share_directory(package_name), rviz_path)

    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': xacro_raw_description}])

    joint_state_publisher = Node(
            name='joint_state_publisher_gui',
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': xacro_raw_description}])
    
    rviz_launch = Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
			)
    
    launch_description = [
        rviz_launch,
        robot_state_publisher,
        joint_state_publisher
    ]

    return LaunchDescription(launch_description)