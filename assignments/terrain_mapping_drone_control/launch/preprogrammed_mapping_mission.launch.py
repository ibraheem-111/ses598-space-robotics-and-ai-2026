#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='terrain_mapping_drone_control',
            executable='preprogrammed_mapping_landing',
            name='preprogrammed_mapping_landing',
            output='screen'
        )
    ])
