"""Gazebo Harmonic + robot spawn + all sensor bridges + ros2_control controllers. No navigation here (see mobile_robot_bringup).

   ros2 launch mobile_robot_gazebo gazebo.launch.py world:=house_world.sdf
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    gz_share = get_package_share_directory('mobile_robot_gazebo')
    desc_share = get_package_share_directory('mobile_robot_description')

    args = [
        DeclareLaunchArgument('world', default_value='house_world.sdf',
                              description='file in mobile_robot_gazebo/worlds: house_world.sdf, street_world.sdf, camera_world.sdf'),
        DeclareLaunchArgument('gui', default_value='true', description='false = no Gazebo window'),
        DeclareLaunchArgument('x', default_value='0.0', description='spawn x'),
        DeclareLaunchArgument('y', default_value='0.0', description='spawn y'),
        DeclareLaunchArgument('degrade', default_value='true',
                              description='lidar dropouts + range-bias drift (the pillar mask is always on)'),
    ]

    # package://mobile_robot_description/meshes/... in the URDF is found through this path (the parent of share/<pkg>)
    resources = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(desc_share))

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'hardware': 'sim'}.items())

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            TextSubstitution(text='-r '),
            PythonExpression(["'' if '", LaunchConfiguration('gui'), "' == 'true' else '-s --headless-rendering '"]),
            PathJoinSubstitution([gz_share, 'worlds', LaunchConfiguration('world')]),
            TextSubstitution(text=f" --gui-config {os.path.join(gz_share, 'config', 'gui.config')}"),
        ]}.items())

    # one bridge process for everything: clock, camera (OAK-D), raw lidar, IMU, passive joint states
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                   '/gz_passive_joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        remappings=[('/camera/image', '/camera/image_raw'),
                    ('/camera/depth_image', '/camera/depth/image_raw'),
                    ('/gz_passive_joint_states', '/joint_states')])

    scan_degrader = Node(
        package='mobile_robot_gazebo', executable='scan_degrader.py', output='screen',
        parameters=[{'use_sim_time': True,
                     'dropout_prob': PythonExpression(["0.01 if '", LaunchConfiguration('degrade'), "' == 'true' else 0.0"]),
                     'bias_walk': PythonExpression(["0.0005 if '", LaunchConfiguration('degrade'), "' == 'true' else 0.0"])}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.08',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y')])

    # the controller_manager only exists after Gazebo has loaded the robot: give the spawners time
    diff_drive = Node(package='controller_manager', executable='spawner', output='screen',
                      arguments=['diff_cont', '--controller-manager-timeout', '120'])
    joint_broad = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_broad', '--controller-manager-timeout', '120'])

    return LaunchDescription(args + [resources, rsp, gazebo, bridge, scan_degrader, spawn, diff_drive, joint_broad])
