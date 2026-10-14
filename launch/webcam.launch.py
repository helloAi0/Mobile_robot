"""Optional: publish a REAL USB / laptop webcam to ROS 2 as /webcam/image_raw.

Only for people who have a camera. Needs:  sudo apt install ros-jazzy-v4l2-camera
Run:   ros2 launch mobile_robot webcam.launch.py video_device:=/dev/video0
Then in RViz enable the "Webcam (optional)" Image display (rviz/camera.rviz) or add Image -> /webcam/image_raw.

A plain webcam has no depth, so it gives a colour image only (no point cloud). A depth camera
(Intel RealSense, Luxonis OAK-D, ...) uses its own driver package; point its topics at /camera/... to
reuse rviz/camera.rviz.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='Camera device (ls /dev/video*)'),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='webcam',
            namespace='webcam',                      # topics: /webcam/image_raw, /webcam/camera_info
            parameters=[{
                'video_device': video_device,
                'image_size': [640, 480],
                'camera_frame_id': 'webcam_optical',
            }],
            output='screen',
        ),

        # Gives the camera frame a place in the TF tree so RViz has no "frame does not exist" error
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0.2',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'webcam_optical'],
        ),
    ])
