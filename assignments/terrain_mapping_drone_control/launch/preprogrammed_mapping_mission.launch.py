#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='terrain_mapping_drone_control',
            executable='preprogrammed_mapping_landing',
            name='preprogrammed_mapping_landing',
            output='screen',
            parameters=[{
                'cylinder_x': 5.0,
                'cylinder_y': 0.0,
                'orbit_radius': 2.5,
                'keepout_radius': 1.6,
                'takeoff_z': -2.5,
                'helix_top_z': -4.0,
                'turns_up': 1.5,
                'turn_period_sec': 18.0,
            }]
        )
    ])