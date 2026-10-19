"""Simulation + navigation stack in one command (the top-level launch file).

  ros2 launch mobile_robot_bringup launch_sim.launch.py                       manual driving + live map
  ros2 launch mobile_robot_bringup launch_sim.launch.py auto:=true            wanders by itself and maps
  ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true            click "2D Goal Pose" in RViz
  ros2 launch mobile_robot_bringup launch_sim.launch.py world:=street_world.sdf
  ros2 launch mobile_robot_bringup launch_sim.launch.py record:=true         rosbag of the run -> ./bags (replay.launch.py)
  ros2 launch mobile_robot_bringup launch_sim.launch.py ekf:=true             wheel odometry + IMU fused (robot_localization)
  ros2 launch mobile_robot_bringup launch_sim.launch.py nav2:=true patrol:=true   patrol the rooms with tuned Nav2

Composition:  mobile_robot_gazebo  (Gazebo + robot + sensor bridges + controllers)
            + autonomy.launch.py   (command mux + safety, slam_toolbox, Nav2, autopilot)
            + RViz, detector, point-cloud filter.   The robot model itself is in mobile_robot_description.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def rviz_environment():
    """WSL2 / some GPU drivers cannot compile RViz's map shader ("active samplers with a different type refer to the same
    texture image unit": the SLAM map stays invisible). Software OpenGL (Mesa llvmpipe) fixes it and is fast enough for
    RViz. Auto-enabled on WSL; force with  export MOBILE_ROBOT_RVIZ_GL=software  or  =native."""
    mode = os.environ.get('MOBILE_ROBOT_RVIZ_GL', 'auto').lower()
    if mode == 'auto':
        try:
            mode = 'software' if 'microsoft' in open('/proc/version').read().lower() else 'native'
        except OSError:
            mode = 'native'
    return {'LIBGL_ALWAYS_SOFTWARE': '1'} if mode == 'software' else {}


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
        DeclareLaunchArgument('yaw', default_value='0.0', description='spawn heading, rad'),
        DeclareLaunchArgument('ekf', default_value='false', description='fuse wheel odometry + IMU gyro with robot_localization'),
        DeclareLaunchArgument('mux', default_value='python', description='python | cpp: command mux implementation'),
        DeclareLaunchArgument('patrol', default_value='false', description='with nav2:=true: patrol the rooms (config/patrol_house.yaml)'),
        DeclareLaunchArgument('record', default_value='false', description='record a rosbag of the run (bag_topics.yaml) into ./bags'),
    ]

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={**{k: LaunchConfiguration(k) for k in ('world', 'gui', 'x', 'y', 'yaw', 'degrade')},
                          # with the EKF running, the controller must not publish odom -> base_link too
                          'odom_tf': PythonExpression(["'false' if '", LaunchConfiguration('ekf'), "' == 'true' else 'true'"])}.items())

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
                          'auto': LaunchConfiguration('auto'),
                          'ekf': LaunchConfiguration('ekf'),
                          'patrol': LaunchConfiguration('patrol'),
                          'mux': LaunchConfiguration('mux')}.items())

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'camera.rviz')],
        parameters=[{'use_sim_time': True}], output='log', additional_env=rviz_environment(),
        condition=IfCondition(LaunchConfiguration('rviz')))

    # rosbag of the run (config/bag_topics.yaml); the bag lands in ./bags/run_<time>
    recorder = ExecuteProcess(cmd=['ros2', 'run', 'mobile_robot_bringup', 'record_bag.sh'], output='log',
                              condition=IfCondition(LaunchConfiguration('record')))

    webcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'webcam.launch.py')),
        condition=IfCondition(LaunchConfiguration('webcam')))

    return LaunchDescription(args + [simulator, detector, cloud_filter, autonomy, rviz, webcam, recorder])
