from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    home_dir = os.environ['HOME']
    base_path = os.path.join(home_dir, 'ws_ses', 'src', 'terrain_mapping_drone_control', 'terrain_mapping_drone_control')

    return LaunchDescription([
        # Run aruco_tracker.py with log-level WARN to hush output
        # ExecuteProcess(
        #     cmd=[
        #         'python3', os.path.join(base_path, 'aruco_tracker.py'),
        #         '--ros-args', '--log-level', 'warn'
        #     ],
        #     output='log',  # suppresses logs to log file instead of screen
        # ),

        # # Run auto_detect_land.py with screen output
        # ExecuteProcess(
        #     cmd=['python3', os.path.join(base_path, 'auto_detect_land.py')],
        #     output='screen',
        # )
        Node(
            package='terrain_mapping_drone_control',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            ros_arguments=['--log-level', 'warn'],
        ),

        Node(
            package='terrain_mapping_drone_control',
            executable='auto_detect_land',
            name='auto_detect_land',
            output='screen',
            parameters=[{
                'orbit_center_x': 5.0,  
                'orbit_center_y': 0.0,
                'radius': 5.0, 
                'takeoff_altitude_ned': -3.0,
                'orbit_min_z_ned': -20.0,
                'orbit_climb_step_ned': -0.01, 
                'tangential_speed': 0.18,  # m/s
                'control_period': 0.05,   # 20 Hz
                'circle_entry_tolerance': 0.2,  # m
            }],
            # ros_arguments=['--log-level', 'debug'],  
        )
    ])
    """    self.declare_parameter('orbit_center_x', 0.0)          # odom frame
        self.declare_parameter('orbit_center_y', 0.0)          # odom frame
        self.declare_parameter('radius', 5.0)                  # meters
        self.declare_parameter('takeoff_altitude_ned', -5.0)   # PX4-style NED z
        self.declare_parameter('orbit_min_z_ned', -20.0)       # more negative = higher in NED
        self.declare_parameter('tangential_speed', 0.18)       # m/s around cylinder
        self.declare_parameter('vertical_speed_ned', float('nan'))
        self.declare_parameter('orbit_climb_step_ned', float('nan'))  # legacy alias, now interpreted as m/s
        self.declare_parameter('tangential_accel', 0.06)       # m/s^2 ramp for smooth start
        self.declare_parameter('vertical_accel', 0.02)         # m/s^2 ramp for smooth climb
        self.declare_parameter('control_period', 0.05)         # 20 Hz setpoint stream
        self.declare_parameter('circle_entry_speed', 0.30)     # m/s while joining circle
        self.declare_parameter('circle_entry_tolerance', 0.25) # meters
        self.declare_parameter('yaw_inward', True
"""