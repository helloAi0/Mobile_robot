import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'mobile_robot'
    pkg_share = get_package_share_directory(package_name)

    # ---- launch arguments --------------------------------------------------------------------
    #   ros2 launch mobile_robot launch_sim.launch.py                       manual driving + live map
    #   ros2 launch mobile_robot launch_sim.launch.py auto:=true            wanders by itself and maps
    #   ros2 launch mobile_robot launch_sim.launch.py nav2:=true            click "2D Goal Pose" in RViz
    #   ros2 launch mobile_robot launch_sim.launch.py world:=street_world.sdf
    args = [
        DeclareLaunchArgument('world', default_value='house_world.sdf',
                              description='world in worlds/: house_world.sdf (rooms, for SLAM), street_world.sdf, camera_world.sdf'),
        DeclareLaunchArgument('slam', default_value='true', description='build a map (slam_toolbox)'),
        DeclareLaunchArgument('nav2', default_value='false', description='Nav2: drive to goals clicked in RViz'),
        DeclareLaunchArgument('auto', default_value='false', description='start in autonomous wandering mode'),
        DeclareLaunchArgument('rviz', default_value='true', description='open RViz'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = no Gazebo window (drive and watch only from RViz; needs working server-side rendering)'),
        DeclareLaunchArgument('detector', default_value='true', description='lidar object detector (orange balls in RViz)'),
        DeclareLaunchArgument('cloud', default_value='true', description='floor-free 3D camera cloud for RViz'),
        DeclareLaunchArgument('webcam', default_value='false', description='also publish a real USB webcam (/webcam/image_raw)'),
        DeclareLaunchArgument('x', default_value='0.0', description='spawn x'),
        DeclareLaunchArgument('y', default_value='0.0', description='spawn y'),
    ]

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Our own worlds: the stock empty.sdf does not load the Sensors system, so camera and lidar would
    # never render. gui.config = Gazebo's standard window (mouse orbit/zoom) + camera preview + lidar viewer.
    gui_config = os.path.join(pkg_share, 'config', 'gui.config')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [
            TextSubstitution(text='-r '),
            PythonExpression(["'' if '", LaunchConfiguration('gui'), "' == 'true' else '-s --headless-rendering '"]),
            PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')]),
            TextSubstitution(text=f' --gui-config {gui_config}'),
        ]}.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')

    # camera.xacro: rgbd_camera with <topic>camera</topic> -> /camera/image, /depth_image, /points, /camera_info
    camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        remappings=[('/camera/image', '/camera/image_raw'),
                    ('/camera/depth_image', '/camera/depth/image_raw')],
        output='screen')

    lidar_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'], output='screen')

    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.05',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y')],
        output='screen')

    # The controller_manager only exists after Gazebo loaded the robot: give the spawners time.
    diff_drive_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["diff_cont", "--controller-manager-timeout", "120"], output='screen')
    joint_broad_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_broad", "--controller-manager-timeout", "120"], output='screen')

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
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'autonomy.launch.py')]),
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
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'webcam.launch.py')]),
        condition=IfCondition(LaunchConfiguration('webcam')))

    return LaunchDescription(args + [
        rsp, gazebo, clock_bridge, camera_bridge, lidar_bridge, spawn_entity,
        diff_drive_spawner, joint_broad_spawner,
        detector, cloud_filter, autonomy, rviz, webcam,
    ])
