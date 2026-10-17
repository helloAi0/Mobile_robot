"""Run the robot WITHOUT Gazebo: on the real hardware, or in 'mock' mode (no hardware at all, for testing the software stack).

  ros2 launch mobile_robot_bringup hardware.launch.py                                   # mock: controllers, mux, safety, RViz; wheels are simulated in software
  ros2 launch mobile_robot_bringup hardware.launch.py hardware:=real \
        hw_plugin:=diffdrive_arduino/DiffDriveArduino hw_device:=/dev/ttyACM0   # real motors through YOUR ros2_control driver
  ... auto:=true | nav2:=true | slam:=false | rviz:=false

What runs: robot_state_publisher, ros2_control (controller_manager + diff_drive_controller + joint_state_broadcaster),
cmd_vel_mux (keyboard > Nav2 > autonomy, with the obstacle guard, e-stop and acceleration limits), slam_toolbox, optional Nav2
and autonomous mode, RViz. YOU provide (docs/HARDWARE.md): the motor driver plugin and a lidar driver publishing /scan
(frame 'laser_frame', see mobile_robot_description/urdf/lidar.xacro for its mounting position).
"""
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _setup(context):
    pkg = get_package_share_directory('mobile_robot_bringup')
    lc = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    mode = lc('hardware')
    plugin = lc('hw_plugin') if mode == 'real' else 'mock_components/GenericSystem'
    if mode == 'real' and plugin.startswith('mock'):
        return [LogInfo(msg='hardware:=real needs hw_plugin:=<your ros2_control driver>, e.g. diffdrive_arduino/DiffDriveArduino. See docs/HARDWARE.md')]

    mappings = {'hardware': mode, 'hw_plugin': plugin, 'hw_device': lc('hw_device'),
                'hw_baud': lc('hw_baud'), 'hw_counts_per_rev': lc('hw_counts_per_rev')}
    robot_description = xacro.process_file(os.path.join(get_package_share_directory('mobile_robot_description'), 'urdf', 'robot.urdf.xacro'), mappings=mappings).toxml()

    # same controller settings as in simulation, but on the real clock
    with open(os.path.join(pkg, 'config', 'my_controllers.yaml')) as f:
        cfg = yaml.safe_load(f)
    for section in cfg.values():
        section['ros__parameters']['use_sim_time'] = False
    ctrl_file = os.path.join(tempfile.mkdtemp(prefix='mobile_robot_'), 'controllers.yaml')
    with open(ctrl_file, 'w') as f:
        yaml.safe_dump(cfg, f)

    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': False}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[{'robot_description': robot_description}, ctrl_file]),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_broad', '--param-file', ctrl_file, '--controller-manager-timeout', '60']),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['diff_cont', '--param-file', ctrl_file, '--controller-manager-timeout', '60']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'autonomy.launch.py')),
            launch_arguments={'use_sim_time': 'false', 'slam': lc('slam'), 'nav2': lc('nav2'), 'auto': lc('auto'),
                              'require_scan': 'false' if mode == 'mock' else 'true'}.items()),
        Node(package='rviz2', executable='rviz2', output='log', arguments=['-d', os.path.join(pkg, 'rviz', 'camera.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]
    if mode == 'mock':
        actions.insert(0, LogInfo(msg='MOCK mode: no motors and no lidar. Wheels are simulated in software; SLAM needs a /scan source.'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='mock', description='mock | real'),
        DeclareLaunchArgument('hw_plugin', default_value='mock_components/GenericSystem',
                              description='ros2_control system interface for hardware:=real'),
        DeclareLaunchArgument('hw_device', default_value='/dev/ttyUSB0', description='serial device of the motor controller'),
        DeclareLaunchArgument('hw_baud', default_value='57600'),
        DeclareLaunchArgument('hw_counts_per_rev', default_value='3436', description='encoder counts per wheel revolution'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='false'),
        DeclareLaunchArgument('auto', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
