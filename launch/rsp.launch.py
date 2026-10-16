import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def _setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    xacro_file = os.path.join(get_package_share_directory('mobile_robot'), 'description', 'robot.urdf.xacro')
    mappings = {k: LaunchConfiguration(k).perform(context)
                for k in ('hardware', 'hw_plugin', 'hw_device', 'hw_baud', 'hw_counts_per_rev')}
    robot_description = xacro.process_file(xacro_file, mappings=mappings).toxml()
    return [Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true', description='kept for compatibility'),
        DeclareLaunchArgument('hardware', default_value='sim', description='sim (Gazebo) | mock | real'),
        DeclareLaunchArgument('hw_plugin', default_value='mock_components/GenericSystem',
                              description='ros2_control system plugin for hardware:=real'),
        DeclareLaunchArgument('hw_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('hw_baud', default_value='57600'),
        DeclareLaunchArgument('hw_counts_per_rev', default_value='3436'),
        OpaqueFunction(function=_setup),
    ])
