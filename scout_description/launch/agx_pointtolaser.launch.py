from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/rslidar_points'),
                ('scan', '/scan')
            ],
            parameters=[
                {'target_frame': 'rslidar'},  # Frame ID of the LiDAR
                {'transform_tolerance': 0.01},
                {'min_height': -0.5},  # Min height (meters) for 2D slice
                {'max_height': 0.5},   # Max height (meters) for 2D slice
                {'angle_min': -3.14159},  # -180 degrees in radians
                {'angle_max': 3.14159},   # +180 degrees in radians
                {'angle_increment': 0.00873},  # ~0.5 degrees
                {'scan_time': 0.1},  # 10 Hz (match RS16 rotation rate)
                {'range_min': 0.3},  # Min range (meters)
                {'range_max': 100.0},  # Max range (meters)
                {'use_inf': True},  # Use infinity for out-of-range points
                {'inf_epsilon': 1.0}  # Distance threshold for infinity
            ]
        )
    ])