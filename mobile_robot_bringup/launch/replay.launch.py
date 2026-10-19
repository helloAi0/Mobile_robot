"""Replay a recorded run in RViz: the robot, the lidar, the map, raw vs fused odometry, the plan, exactly as it happened.

  ros2 launch mobile_robot_bringup replay.launch.py bag:=bags/run_20261019_101500
  ros2 launch mobile_robot_bringup replay.launch.py bag:=<folder> rate:=2.0 loop:=true start:=30

Nothing else runs: no Gazebo, no SLAM, no Nav2. The bag contains /clock, /tf and /map, so RViz shows what the robot saw.
Because the same bag replays identically every time, a failure seen once can be inspected as often as needed.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def play_command(bag, rate, start, loop, qos_file):
    """The `ros2 bag play` command line (plain function so a test can check it)."""
    cmd = ['ros2', 'bag', 'play', bag, '--clock', '--rate', str(rate), '--start-offset', str(start),
           '--qos-profile-overrides-path', qos_file]
    if str(loop).lower() == 'true':
        cmd.append('--loop')
    return cmd


def _setup(context):
    pkg = get_package_share_directory('mobile_robot_bringup')
    lc = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    return [ExecuteProcess(cmd=play_command(lc('bag'), lc('rate'), lc('start'), lc('loop'), os.path.join(pkg, 'config', 'bag_qos.yaml')),
                           output='screen')]


def generate_launch_description():
    pkg = get_package_share_directory('mobile_robot_bringup')
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(pkg, 'rviz', 'replay.rviz')],
                parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='bag folder (made by record_bag.sh or record:=true)'),
        DeclareLaunchArgument('rate', default_value='1.0', description='playback speed'),
        DeclareLaunchArgument('start', default_value='0.0', description='skip this many seconds at the start'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        rviz,
        OpaqueFunction(function=_setup),
    ])
