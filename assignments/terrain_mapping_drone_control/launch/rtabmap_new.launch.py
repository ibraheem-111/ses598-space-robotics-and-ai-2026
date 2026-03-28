# Requirements:
#   A kinect for xbox 360
#   Install kinect_ros2 package (use this fork: https://github.com/matlabbe/kinect_ros2)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'OakD-Lite-Modify/base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'approx_sync':True}]

    remappings=[
          ('rgb/image', '/drone/front_rgb'),
          ('rgb/camera_info', '/drone/camera_info'),
          ('depth/image', '/drone/front_depth'),
          ('odom', '/rtabmap/odom')
          ]

    return LaunchDescription([
        # Optical rotation
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "kinect_rgb"]),

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Static TF publisher for camera to base link transform
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_to_base_link',
        #     arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'camera_link'],
        #     output='screen'
        # ),  

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap"),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'],
            namespace="rtabmap"),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap"),
    ])