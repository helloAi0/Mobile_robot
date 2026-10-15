import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'mobile_robot'
    pkg_share = get_package_share_directory(package_name)

    # ---- launch arguments --------------------------------------------------------------------
    #   ros2 launch mobile_robot launch_sim.launch.py world:=camera_world.sdf webcam:=true detector:=false
    world_arg = DeclareLaunchArgument(
        'world', default_value='street_world.sdf',
        description='World file inside the package worlds/ folder (street_world.sdf or camera_world.sdf)')
    webcam_arg = DeclareLaunchArgument(
        'webcam', default_value='false',
        description='true = also publish a real USB webcam as /webcam/image_raw (needs ros-jazzy-v4l2-camera)')
    relay_arg = DeclareLaunchArgument(
        'relay', default_value='true',
        description='true = forward a normal /cmd_vel (Twist) to /diff_cont/cmd_vel (TwistStamped) so any teleop works')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='true = also open RViz with rviz/camera.rviz (point cloud, lidar, images, detections)')
    detector_arg = DeclareLaunchArgument(
        'detector', default_value='true',
        description='true = run the simple lidar object detector (publishes /detections markers)')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Our own worlds: the stock empty.sdf does not load the Sensors system, so the camera and
    # lidar would never render. gui.config is Gazebo's standard window (mouse orbit/zoom, toolbars)
    # plus a camera preview and the lidar visualiser.
    gui_config = os.path.join(pkg_share, 'config', 'gui.config')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [
            TextSubstitution(text='-r '),
            PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')]),
            TextSubstitution(text=f' --gui-config {gui_config}'),
        ]}.items()
    )

    # Bridge the Gazebo clock to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # The sensor in camera.xacro is an rgbd_camera with <topic>camera</topic>, so Gazebo publishes
    # /camera/image, /camera/depth_image, /camera/points and /camera/camera_info. They are remapped
    # to the usual ROS names (/camera/image_raw, /camera/depth/image_raw).
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        remappings=[('/camera/image', '/camera/image_raw'),
                    ('/camera/depth_image', '/camera/depth/image_raw')],
        output='screen'
    )

    # Lidar (<topic>scan</topic> in lidar.xacro) -> ROS /scan (frame laser_frame)
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen'
    )

    # Spawn the entity using ros_gz_sim create node
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_bot',
                   '-z', '0.05'],
        output='screen'
    )

    # The controller_manager only exists after Gazebo has loaded the robot, so give the
    # spawners plenty of time to wait for it (default is too short on slow starts).
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager-timeout", "120"],
        output='screen',
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager-timeout", "120"],
        output='screen',
    )

    # Simple lidar detector: red sphere + "object x.x m" label in RViz (/detections)
    detector = Node(
        package=package_name,
        executable='object_detector.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('detector')),
    )

    # Removes the floor from the 3D camera cloud -> /camera/points_filtered (what RViz shows)
    cloud_filter = Node(
        package=package_name,
        executable='cloud_filter.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # /cmd_vel (Twist) -> /diff_cont/cmd_vel (TwistStamped), stamped with sim time
    relay = Node(
        package=package_name,
        executable='cmd_vel_relay.py',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('relay')),
    )

    # RViz with our config (on sim time, like everything else)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'camera.rviz')],
        parameters=[{'use_sim_time': True}],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Optional real webcam (only if you have one): /webcam/image_raw
    webcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'webcam.launch.py')]),
        condition=IfCondition(LaunchConfiguration('webcam')),
    )

    return LaunchDescription([
        world_arg,
        webcam_arg,
        detector_arg,
        relay_arg,
        rviz_arg,
        rsp,
        gazebo,
        clock_bridge,
        camera_bridge,
        lidar_bridge,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        detector,
        cloud_filter,
        relay,
        rviz,
        webcam,
    ])


# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch_ros.actions import Node

# def generate_launch_description():

#     package_name = 'mobile_robot'

#     rsp = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
#         )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
#     )

#     # joystick = IncludeLaunchDescription(
#     #     PythonLaunchDescriptionSource([os.path.join(
#     #         get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
#     #     )]), launch_arguments={'use_sim_time': 'true'}.items()
#     # )

#     # twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
#     # twist_mux = Node(
#     #     package="twist_mux",
#     #     executable="twist_mux",
#     #     parameters=[twist_mux_params, {'use_sim_time': True}],
#     #     remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel')]
#     # )

#     # Include the Gazebo Sim (Harmonic) launch file
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
#         launch_arguments={'gz_args': 'empty.sdf -r'}.items()
#     )

#     # Bridge the Gazebo camera topic to ROS 2 (using wildcard to catch namespaced topics)
#     camera_bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=['/my_bot/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
#         output='screen'
#     )

#     # Spawn the entity using ros_gz_sim create node
#     spawn_entity = Node(
#         package='ros_gz_sim',
#         executable='create',
#         arguments=['-topic', 'robot_description',
#                    '-entity', 'my_bot',
#                    '-z', '0.05'],
#         output='screen'
#     )

#     diff_drive_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["diff_cont"],
#     )

#     joint_broad_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_broad"],
#     )
    
#     # Bridge the Gazebo camera topic to ROS 2 (with correct entity prefix)
#     camera_bridge = Node(
#         package='ros_gz_bridge',
#         executable='parameter_bridge',
#         arguments=['/my_bot/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
#         output='screen'
#     )

#     return LaunchDescription([
#         rsp,
#         # joystick,
#         # twist_mux,
#         gazebo,
#         clock_bridge,
#         spawn_entity,
#         diff_drive_spawner,
#         joint_broad_spawner,
#         camera_bridge
#     ])