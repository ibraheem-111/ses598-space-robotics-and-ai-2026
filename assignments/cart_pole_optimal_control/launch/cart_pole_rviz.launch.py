from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('cart_pole_optimal_control').find('cart_pole_optimal_control')
    urdf_model_path = os.path.join(pkg_share, 'models', 'cart_pole', 'model.urdf')
    with open(urdf_model_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    create_cart_pole = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_model_path,
            '-name', 'cart_pole',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    controller_group = [
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
        TimerAction(
            period=1.8,
            actions=[
                Node(
                    package='cart_pole_optimal_control',
                    executable='earthquake_force_generator',
                    name='earthquake_force_generator',
                    output='screen',
                    parameters=[{
                        'base_amplitude': 15.0,
                        'frequency_range': [0.5, 4.0],
                        'update_rate': 50.0
                    }]
                ),
            ]
        ),
    ]

    # Create and return launch description
    return LaunchDescription([
        # Gazebo (headless mode)
        ExecuteProcess(
            cmd=[
                'bash', '-lc',
                'if command -v ign >/dev/null 2>&1; then '
                'ign gazebo -r empty.sdf; '
                'elif command -v gz >/dev/null 2>&1; then '
                'gz sim -r empty.sdf; '
                'else '
                'echo "ERROR: Neither ign nor gz CLI found in PATH." >&2; '
                'exit 127; '
                'fi'
            ],
            output='screen'
        ),

        # Spawn robot in Gazebo
        TimerAction(
            period=2.5,
            actions=[
                create_cart_pole,
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
                'ignore_timestamp': False,
                'use_sim_time': True
            }]
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=create_cart_pole,
                on_exit=controller_group
            )
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'cart_pole.rviz')],
            parameters=[{
                'update_rate': 50.0,  # Match the publish frequency
                'use_sim_time': True
            }]
        )
    ]) 