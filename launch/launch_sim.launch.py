import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'mobile_robot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Our own world: the stock empty.sdf does not load the Sensors system, so the camera
    # would never render. This one does, and also opens a camera preview panel.
    world_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'camera_world.sdf')

    # Include the Gazebo Sim (Harmonic) launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Bridge the Gazebo clock to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Bridge the Gazebo camera to ROS 2. The sensor in camera.xacro has <topic>camera/image_raw</topic>,
    # which Gazebo publishes as /camera/image_raw (NOT /my_bot/camera/image_raw, which is what
    # the old bridge listened to, so no image ever reached ROS or RViz).
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
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

    return LaunchDescription([
        rsp,
        gazebo,
        clock_bridge,
        camera_bridge,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner
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