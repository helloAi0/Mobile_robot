"""Look at the model without any simulator: sliders for the suspension and caster joints.

   ros2 launch mobile_robot_description view_robot.launch.py
Then in RViz add RobotModel (description topic /robot_description), fixed frame base_link.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rsp = os.path.join(get_package_share_directory('mobile_robot_description'), 'launch', 'rsp.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rsp), launch_arguments={'hardware': 'mock'}.items()),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2'),
    ])
