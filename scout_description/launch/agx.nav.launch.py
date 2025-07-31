import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import SetEnvironmentVariable, ExecuteProcess, LogInfo
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)

from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO
import tf_transformations
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy



def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='scout_description').find('scout_description')
    default_model_path = os.path.join(pkg_share, 'urdf/scout_v2/scout_v2_trailer.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/navigation_config.rviz')

    # QoS profile for lidar
    qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST
    )

    # Scout base node for real robot
    scout_base_node = launch_ros.actions.Node(
        package='scout_base',
        executable='scout_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'port_name': launch.substitutions.LaunchConfiguration('port_name'),                
                'odom_frame': launch.substitutions.LaunchConfiguration('odom_frame'),
                'base_frame': launch.substitutions.LaunchConfiguration('base_frame'),
                'odom_topic_name': launch.substitutions.LaunchConfiguration('odom_topic_name'),
                'is_scout_mini': launch.substitutions.LaunchConfiguration('is_scout_mini'),
                'is_omni_wheel': launch.substitutions.LaunchConfiguration('is_omni_wheel'),
                'auto_reconnect': launch.substitutions.LaunchConfiguration('auto_reconnect'),
                'simulated_robot': launch.substitutions.LaunchConfiguration('simulated_robot'),
                'control_rate': launch.substitutions.LaunchConfiguration('control_rate'),
        }])

    # Robot state publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')]), 'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Joint state publisher
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[LaunchConfiguration('model')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # RViz node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Robot localization for real robot
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/agx_ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Trailer hitch publisher
    trailer_pub = launch_ros.actions.Node(
        package="scout_description",
        executable="test_pub_hitch.py",
        name="test_pub_hitch",
        output="screen",
    )

    # Lidar launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rslidar_sdk'),
                'launch',
                'humble_start.py'
            ])
        ]),

    )

    # Pointcloud to laserscan conversion
    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            {'use_sim_time': False},  # Explicitly set for real robot
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
    )
    
    # Navigation localization
    nav_loc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "nav2_bringup",
            "localization_launch.py",
            f"map:={os.path.join(pkg_share, 'maps/basement.yaml')}",
            f"params_file:={os.path.join(pkg_share, 'config/amcl_params.yaml')}",
            "use_sim_time:=False",
        ],
        output="screen",
    )
    
    # Navigation planner
    nav_nav = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "nav2_bringup",
            "navigation_launch.py",
            f"params_file:={os.path.join(pkg_share, 'config/straight_planner.yaml')}",
            "use_sim_time:=False",
        ],
        output="screen",
    )

    return launch.LaunchDescription([
        # Launch arguments
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
        
        # Scout base arguments
        launch.actions.DeclareLaunchArgument(name='port_name', default_value='can0',
                                            description='CAN bus name, e.g. can0'),
        launch.actions.DeclareLaunchArgument(name='odom_frame', default_value='odom',
                                            description='Odometry frame id'),
        launch.actions.DeclareLaunchArgument(name='base_frame', default_value='base_link',
                                            description='Base link frame id'),
        launch.actions.DeclareLaunchArgument(name='odom_topic_name', default_value='odom',
                                            description='Odometry topic name'),
        launch.actions.DeclareLaunchArgument(name='is_scout_mini', default_value='false',
                                            description='Scout mini model'),
        launch.actions.DeclareLaunchArgument(name='is_omni_wheel', default_value='false',
                                            description='Scout mini omni-wheel model'),
        launch.actions.DeclareLaunchArgument(name='auto_reconnect', default_value='true',
                                            description='Attempts to re-establish CAN command mode'),
        launch.actions.DeclareLaunchArgument(name='simulated_robot', default_value='false',
                                            description='Whether running with simulator'),
        launch.actions.DeclareLaunchArgument(name='control_rate', default_value='50',
                                            description='Control loop update rate'),

        # Nodes
        scout_base_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        robot_localization_node,
        rviz_node,
        trailer_pub,
        lidar_launch,
        pointcloud_to_laserscan_node,
        nav_loc,
        nav_nav
    ])