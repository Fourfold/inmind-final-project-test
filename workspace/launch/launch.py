import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detector',
            executable='object_detector',
        ),
        Node(
           package='distance_check',
           executable='distance_sensor_server',
        ),
        Node(
           package='distance_converter',
           executable='distance_converter',
        ),
        Node(
           package='main',
           executable='main_node',
        ),
    ])
