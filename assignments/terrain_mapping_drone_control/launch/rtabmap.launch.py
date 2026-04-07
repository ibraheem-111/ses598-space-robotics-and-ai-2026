from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo

parameters = [{
    'use_sim_time': LaunchConfiguration('use_sim_time'),

    # RTAB-Map parameters
    'frame_id': 'base_link',
    'subscribe_depth': True,
    'subscribe_rgb': True,
    'subscribe_rgbd': False,
    'approx_sync': False,
    'queue_size': 2,
    'sync_queue_size': 2,
    'odom_sensor_sync': False,

    # Odometry parameters
    'odom_frame_id': 'odom',
    # 'subscribe_odom_info': False,
    'odom_tf_angular_variance': 0.01,
    'odom_tf_linear_variance': 0.001,

    # Visual odometry parameters
    'visual_odometry': True,

    # Mapping parameters
    'grid_cell_size': 0.05,
    'grid_size': 20.0,
    'optimize_from_graph_end': True,
    'optimizer_iterations': 100,

    # Loop closure parameters
    'loop_closure_activated': True,
    'loop_closure_restriction_type': 0,
    'loop_closure_min_inliers': 1000,

    # Memory management
    'memory_management': True,
    'max_cloud_size': 50000,
    'min_cluster_size': 100,

    # Odom
    'publish_tf': True,
}]

remappings = [
    ('rgb/image', '/drone/front_rgb'),
    ('rgb/camera_info', '/drone/camera_info'),
    ('depth/image', '/drone/front_depth'),
    ('odom', '/rtabmap/odom'),
    ('imu', '/drone/imu'),
]

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        

        # Static TF publisher for camera to base link transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            # arguments=['0.1', '0', '0.05', str(-math.pi/2), '0', str(-math.pi/2), 'base_link', 'OakD-Lite-Modify/base_link'],
            # arguments=['0', '0', '0', str(-math.pi/2), '0', str(-math.pi/2), 'base_link', 'OakD-Lite-Modify/base_link'],
            # Keep translation from model.sdf; rotate to ROS optical-frame convention.
            arguments=['.12', '.03', '.242', '-1.57079632679', '0', '-1.57079632679', 'base_link', 'OakD-Lite-Modify/base_link'],
            output='screen'
        ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
            ),

        # RTAB-Map node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['-d'],
            parameters=parameters,
            # remappings=[
            #     # Camera topics
            #     ('rgb/image', '/drone/front_rgb'),
            #     ('depth/image', '/drone/front_depth'),
            #     ('rgb/camera_info', '/drone/front_rgb/camera_info'),
                
            #     # Odometry from PX4
            #     ('odom', '/rtabmap/odom'),
                
            #     # Output topics
            #     ('grid_map', 'map'),
            #     ('mapData', 'mapData'),
            #     ('mapPath', 'mapPath'),
            #     ('cloud_map', 'cloud_map')
            # ]
            remappings=remappings
        ),

        # RTAB-Map point cloud generation
        # Node(
        #     package='rtabmap_util',
        #     executable='point_cloud_xyz',
        #     name='point_cloud_xyz',
        #     # parameters=[{
        #     #     'use_sim_time': LaunchConfiguration('use_sim_time'),
        #     #     'decimation': 4,
        #     #     'voxel_size': 0.02,
        #     #     'max_depth': 4.0,
        #     #     'min_depth': 0.4
        #     # }],
        #     parameters=parameters,
        #     remappings=remappings
        # ),



        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),

        # Log info
        LogInfo(
            msg="RTAB-Map launched with drone configuration"
        )
    ]) 