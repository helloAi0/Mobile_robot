"""Driving brain: command mux, SLAM (mapping), optional Nav2 and optional autonomous wandering.

Used by launch_sim.launch.py, and the same file works with a real robot (start your hardware driver first):
    ros2 launch mobile_robot_bringup autonomy.launch.py use_sim_time:=false

Needs (apt):  ros-jazzy-slam-toolbox  ros-jazzy-navigation2  ros-jazzy-nav2-bringup  [ros-jazzy-robot-localization for ekf:=true]
Missing packages are skipped with a message, nothing else breaks.

  ekf:=true      fuse wheel odometry + IMU gyro (config/ekf.yaml); the EKF then owns odom -> base_link and the mux feeds
                 /odometry/filtered to /odom. The caller must start diff_cont with enable_odom_tf:=false (launch_sim.launch.py does).
  mux:=cpp       the C++ command mux (cmd_vel_mux_node) instead of cmd_vel_mux.py: same behaviour, see docs/PERFORMANCE.md
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
    pkg = get_package_share_directory('mobile_robot_bringup')
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    actions = []

    ekf = LaunchConfiguration('ekf').perform(context).lower() == 'true'
    cpp = LaunchConfiguration('mux').perform(context).lower() == 'cpp'

    # who drives: keyboard > Nav2 > autonomous wandering -> /diff_cont/cmd_vel
    mux_params = {'use_sim_time': sim, 'require_scan': LaunchConfiguration('require_scan').perform(context).lower() == 'true'}
    if ekf:
        mux_params['odom_in'] = '/odometry/filtered'      # Nav2 and SLAM read /odom: now the fused estimate
    actions.append(Node(package='mobile_robot_bringup', executable='cmd_vel_mux_node' if cpp else 'cmd_vel_mux.py',
                        name='cmd_vel_mux', output='screen', parameters=[mux_params]))

    if ekf:
        try:
            get_package_share_directory('robot_localization')
            actions.append(Node(package='mobile_robot_bringup', executable='imu_covariance.py', output='screen',
                                parameters=[{'use_sim_time': sim}]))
            actions.append(Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
                                parameters=[os.path.join(pkg, 'config', 'ekf.yaml'), {'use_sim_time': sim}]))
        except PackageNotFoundError:
            actions.append(LogInfo(msg='ekf:=true needs robot_localization: sudo apt install ros-jazzy-robot-localization. '
                                       'Without it nothing publishes odom -> base_link!'))

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
                launch_arguments={'use_sim_time': str(sim).lower(), 'autostart': 'true',
                                  'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml')}.items()))
        except PackageNotFoundError:
            actions.append(LogInfo(msg='Nav2 is not installed -> no goal navigation. Install: '
                                       'sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup'))

    if LaunchConfiguration('patrol').perform(context).lower() == 'true':
        actions.append(Node(package='mobile_robot_bringup', executable='patrol.py', output='screen',
                            parameters=[{'use_sim_time': sim}]))

    actions.append(Node(package='mobile_robot_bringup', executable='auto_explore.py', output='screen',
                        parameters=[{'use_sim_time': sim,
                                     'enabled': LaunchConfiguration('auto').perform(context).lower() == 'true'}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('require_scan', default_value='true',
                              description='safety layer: no forward motion without a fresh /scan (set false only without a lidar)'),
        DeclareLaunchArgument('ekf', default_value='false',
                              description='sensor fusion: wheel odometry + IMU gyro (robot_localization). The caller disables the controller TF.'),
        DeclareLaunchArgument('mux', default_value='python', description='python | cpp: which command mux implementation to run'),
        DeclareLaunchArgument('slam', default_value='true', description='build a map with slam_toolbox'),
        DeclareLaunchArgument('nav2', default_value='false', description='Nav2: drive to a goal clicked in RViz'),
        DeclareLaunchArgument('auto', default_value='false', description='start in autonomous wandering mode'),
        DeclareLaunchArgument('patrol', default_value='false', description='with nav2:=true: patrol the rooms (config/patrol_house.yaml)'),
        OpaqueFunction(function=_setup),
    ])
