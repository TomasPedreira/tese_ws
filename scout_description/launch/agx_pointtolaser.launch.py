from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def generate_launch_description():
    qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST
    )

    return LaunchDescription([
        # RS-LiDAR-16 Driver
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen'
        ),
        # Static Transform: base_link -> rslidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rslidar']
        ),
        # PointCloud to LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/rslidar_points'),
                ('scan', '/scan')
            ],
            parameters=[
                {'target_frame': 'rslidar'},
                {'transform_tolerance': 0.01},
                {'min_height': -0.5},
                {'max_height': 0.5},
                {'angle_min': -3.14159},
                {'angle_max': 3.14159},
                {'angle_increment': 0.00873},
                {'scan_time': 0.1},
                {'range_min': 0.3},
                {'range_max': 100.0},
                {'use_inf': True},
                {'inf_epsilon': 1.0}
            ]
        ),
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'slam_toolbox/scan_topic': '/scan'},
                {'slam_toolbox/odom_topic': '/odom'},
                {'slam_toolbox/map_frame': 'map'},
                {'slam_toolbox/odom_frame': 'odom'},
                {'slam_toolbox/base_frame': 'base_link'},
                {'slam_toolbox/resolution': 0.05},
                {'slam_toolbox/max_laser_range': 100.0},
                {'slam_toolbox/minimum_travel_distance': 0.5},
                {'slam_toolbox/minimum_travel_heading': 0.436},
                {'qos_overrides./scan.reliability': 'best_effort'},
                {'qos_overrides./scan.durability': 'volatile'},
                {'qos_overrides./scan.depth': 10}
            ]
        ),
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {'qos_overrides./scan.reliability': 'best_effort'},
                {'qos_overrides./scan.durability': 'volatile'},
                {'qos_overrides./scan.depth': 10}
            ]
        )
    ])