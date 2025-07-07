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
                {'target_frame': 'lidar_link'},
                {'transform_tolerance': 0.1},  # Increased for TF robustness
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
        # Include slam_toolbox online_async_launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'params_file': PathJoinSubstitution([
                    FindPackageShare('scout_description'),
                    'config',
                    'slam_toolbox_best_effort.yaml'
                ])
            }.items(),
        )
    ])