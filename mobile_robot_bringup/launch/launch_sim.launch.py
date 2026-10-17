"""Simulation + navigation stack in one command (the top-level launch file).

  ros2 launch mobile_robot_bringup launch_sim.launch.py                       manual driving + live map
  ros2 launch mobile_robot_bringup launch_sim.launch.py auto:=true            wanders by itself and maps
  ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true            click "2D Goal Pose" in RViz
  ros2 launch mobile_robot_bringup launch_sim.launch.py world:=street_world.sdf

Composition:  mobile_robot_gazebo  (Gazebo + robot + sensor bridges + controllers)
            + autonomy.launch.py   (command mux + safety, slam_toolbox, Nav2, autopilot)
            + RViz, detector, point-cloud filter.   The robot model itself is in mobile_robot_description.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'mobile_robot_bringup'
    pkg_share = get_package_share_directory(package_name)
    gz_share = get_package_share_directory('mobile_robot_gazebo')

    args = [
        DeclareLaunchArgument('world', default_value='house_world.sdf',
                              description='world in mobile_robot_gazebo/worlds: house_world.sdf (rooms, for SLAM), street_world.sdf, camera_world.sdf'),
        DeclareLaunchArgument('slam', default_value='true', description='build a map (slam_toolbox)'),
        DeclareLaunchArgument('nav2', default_value='false', description='Nav2: drive to goals clicked in RViz'),
        DeclareLaunchArgument('auto', default_value='false', description='start in autonomous wandering mode'),
        DeclareLaunchArgument('rviz', default_value='true', description='open RViz'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = no Gazebo window (drive and watch only from RViz; needs working server-side rendering)'),
        DeclareLaunchArgument('detector', default_value='true', description='lidar object detector (orange balls in RViz)'),
        DeclareLaunchArgument('cloud', default_value='true', description='floor-free 3D camera cloud for RViz'),
        DeclareLaunchArgument('webcam', default_value='false', description='also publish a real USB webcam (/webcam/image_raw)'),
        DeclareLaunchArgument('degrade', default_value='true', description='lidar dropouts + range-bias drift (sensor realism)'),
        DeclareLaunchArgument('x', default_value='0.0', description='spawn x'),
        DeclareLaunchArgument('y', default_value='0.0', description='spawn y'),
    ]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in ('world', 'gui', 'x', 'y', 'degrade')}.items())

    detector = Node(
        package=package_name, executable='object_detector.py',
        parameters=[{'use_sim_time': True}], output='screen',
        condition=IfCondition(LaunchConfiguration('detector')))

    cloud_filter = Node(
        package=package_name, executable='cloud_filter.py',
        parameters=[{'use_sim_time': True}], output='screen',
        condition=IfCondition(LaunchConfiguration('cloud')))

    # command mux + SLAM + optional Nav2 + optional autonomous wandering (same file works on real hardware)
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'autonomy.launch.py')),
        launch_arguments={'use_sim_time': 'true',
                          'slam': LaunchConfiguration('slam'),
                          'nav2': LaunchConfiguration('nav2'),
                          'auto': LaunchConfiguration('auto')}.items())

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'camera.rviz')],
        parameters=[{'use_sim_time': True}], output='log',
        condition=IfCondition(LaunchConfiguration('rviz')))

    webcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'webcam.launch.py')),
        condition=IfCondition(LaunchConfiguration('webcam')))

    return LaunchDescription(args + [simulator, detector, cloud_filter, autonomy, rviz, webcam])
