from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pp_controller',
            executable='pp_controller_node',
            name='pp_controller',
            output='screen',
            parameters=[{
                'lookahead_distance': 1.0,
                'max_linear_speed': 0.5,
                'max_angular_speed': 1.0
            }]
        )
    ]) 