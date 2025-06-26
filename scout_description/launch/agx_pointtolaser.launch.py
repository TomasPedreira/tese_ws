from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def generate_launch_description():
    # QoS profile for /scan
    qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST
    )

    return LaunchDescription([
        # Include rslidar_sdk launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rslidar_sdk'),
                    'launch',
                    'start.py'
                ])
            ])
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
                {'transform_tolerance': 0.1},  # Increased tolerance for TF
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
                {'scan_topic': '/scan'},
                {'odom_topic': '/odom'},
                {'map_frame': 'map'},
                {'odom_frame': 'odom'},
                {'base_frame': 'base_link'},
                {'resolution': 0.05},
                {'max_laser_range': 100.0},
                {'minimum_travel_distance': 0.5},
                {'minimum_travel_heading': 0.436},
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