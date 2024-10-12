#! /usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur3_controller",
            executable="ur_predefined_movement",
            name="ur_predefined_movement",
        )
    ])
