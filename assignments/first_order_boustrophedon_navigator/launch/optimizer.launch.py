from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        # Start our boustrophedon optimizer in SITL mode
        Node(
            package='first_order_boustrophedon_navigator',
            executable='boustrophedon_optimizer',
            name='lawnmower_optimizer',
            parameters=[{'sitl_mode': True}],
            output='screen'
        )
    ]) 