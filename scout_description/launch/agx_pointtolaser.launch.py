from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def generate_launch_description():
    qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rslidar_sdk'),
                    'launch',
                    'start.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'use_system_time': 'true',
                'timestamp_mode': 'system',
                'use_lidar_time': 'false',
                'time_offset': '0.0'
            }.items()
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                {'use_sim_time': False},  # Explicitly set
                {'target_frame': 'lidar_link'},
                {'transform_tolerance': 2.0},
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
            ],
            remappings=[
                ('cloud_in', '/rslidar_points'),
                ('scan', '/scan')
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': PathJoinSubstitution([
                    FindPackageShare('scout_description'),
                    'config',
                    'slam_toolbox.yaml'
                ]),
                'use_sim_time': 'false'  # Force system time
            }.items()
        )
    ])