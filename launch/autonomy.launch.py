"""Driving brain: command mux, SLAM (mapping), optional Nav2 and optional autonomous wandering.

Used by launch_sim.launch.py, and the same file works with a real robot (start your hardware driver first):
    ros2 launch mobile_robot autonomy.launch.py use_sim_time:=false

Needs (apt):  ros-jazzy-slam-toolbox  ros-jazzy-navigation2  ros-jazzy-nav2-bringup
Missing packages are skipped with a message, nothing else breaks.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _setup(context):
    pkg = get_package_share_directory('mobile_robot')
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    actions = []

    # who drives: keyboard > Nav2 > autonomous wandering -> /diff_cont/cmd_vel
    actions.append(Node(package='mobile_robot', executable='cmd_vel_mux.py', output='screen',
                        parameters=[{'use_sim_time': sim,
                                     'require_scan': LaunchConfiguration('require_scan').perform(context).lower() == 'true'}]))

    if LaunchConfiguration('slam').perform(context).lower() == 'true':
        try:
            slam_share = get_package_share_directory('slam_toolbox')
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'online_async_launch.py')),
                launch_arguments={'slam_params_file': os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
                                  'use_sim_time': str(sim).lower()}.items()))
        except PackageNotFoundError:
            actions.append(LogInfo(msg='slam_toolbox is not installed -> no map. Install: '
                                       'sudo apt install ros-jazzy-slam-toolbox'))

    if LaunchConfiguration('nav2').perform(context).lower() == 'true':
        try:
            nav_share = get_package_share_directory('nav2_bringup')
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation_launch.py')),
                launch_arguments={'use_sim_time': str(sim).lower(), 'autostart': 'true'}.items()))
        except PackageNotFoundError:
            actions.append(LogInfo(msg='Nav2 is not installed -> no goal navigation. Install: '
                                       'sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup'))

    actions.append(Node(package='mobile_robot', executable='auto_explore.py', output='screen',
                        parameters=[{'use_sim_time': sim,
                                     'enabled': LaunchConfiguration('auto').perform(context).lower() == 'true'}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('require_scan', default_value='true',
                              description='safety layer: no forward motion without a fresh /scan (set false only without a lidar)'),
        DeclareLaunchArgument('slam', default_value='true', description='build a map with slam_toolbox'),
        DeclareLaunchArgument('nav2', default_value='false', description='Nav2: drive to a goal clicked in RViz'),
        DeclareLaunchArgument('auto', default_value='false', description='start in autonomous wandering mode'),
        OpaqueFunction(function=_setup),
    ])
