from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('cart_pole_optimal_control').find('cart_pole_optimal_control')
    urdf_model_path = os.path.join(pkg_share, 'models', 'cart_pole', 'model.urdf')
    with open(urdf_model_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    # Create and return launch description
    return LaunchDescription([
        # Gazebo (headless mode)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Spawn robot in Gazebo
        TimerAction(
            period=2.5,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-file', urdf_model_path,
                        '-name', 'cart_pole',
                        '-allow_renaming', 'true'
                    ],
                    output='screen'
                ),
            ]
        ),

        # Direct topic bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            output='screen',
            arguments=[
                # Cart force command (ROS -> Gazebo)
                '/model/cart_pole/joint/cart_to_base/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
                # Joint states (Gazebo -> ROS)
                '/world/empty/model/cart_pole/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                # Clock (Gazebo -> ROS)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            ],
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,  # Increased update frequency
                'use_tf_static': True,
                'ignore_timestamp': True
            }]
        ),

        # Start state/control nodes shortly after model spawn
        TimerAction(
            period=3.2,
            actions=[
                Node(
                    package='cart_pole_optimal_control',
                    executable='state_republisher',
                    name='state_republisher',
                    output='screen'
                ),
                Node(
                    package='cart_pole_optimal_control',
                    executable='force_visualizer',
                    name='force_visualizer',
                    output='screen'
                ),
                Node(
                    package='cart_pole_optimal_control',
                    executable='lqr_controller',
                    name='lqr_controller',
                    output='screen'
                ),
            ]
        ),

        # Inject disturbances after controller has had time to settle
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='cart_pole_optimal_control',
                    executable='earthquake_force_generator',
                    name='earthquake_force_generator',
                    output='screen',
                    parameters=[{
                        'base_amplitude': 15.0,  # Strong force amplitude (realistic setting)
                        'frequency_range': [0.5, 4.0],  # Wide frequency range (realistic setting)
                        'update_rate': 50.0  # Update rate in Hz
                    }]
                ),
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'cart_pole.rviz')],
            parameters=[{
                'update_rate': 50.0  # Match the publish frequency
            }]
        )
    ]) 