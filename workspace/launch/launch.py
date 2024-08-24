import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yellow_ball_finder',
            executable='yellow_ball_action_server',
        ),
        Node(
            package='yolov8_webcam',
            executable='yolov8_node',
        ),
        Node(
           package='distance_check',
           executable='distance_sensor_server',
        ),
    ])
