# Requirements:
#   A kinect for xbox 360
#   Install kinect_ros2 package (use this fork: https://github.com/matlabbe/kinect_ros2)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'use_sim_time': LaunchConfiguration('use_sim_time'),
          'frame_id':'base_link',
          'odom_frame_id': 'odom',
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'subscribe_odom':True,
          'approx_sync':True,
        #   'topic_queue_size':1000,
        #   'sync_queue_size':1000,
        #   'odom_sensor_sync':True,
        #   'visual_odometry':False,
        #   'odom_frame_id': 'odom',
          }]

    remappings=[
          ('rgb/image', '/drone/front_rgb'),
          ('rgb/camera_info', '/drone/camera_info'),
          ('depth/image', '/drone/front_depth'),
          ('odom', '/rtabmap/odom')
          ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Static TF from robot base to camera frame used by image headers.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_oakd_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'OakD-Lite-Modify/base_link'],
            output='screen'
        ),

        # Node(
        #     package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        #     parameters=parameters,
        #     remappings=remappings,
        #     namespace="rtabmap"),

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