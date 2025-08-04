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

def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result

    return on_output

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='scout_description').find('scout_description')
    default_model_path = os.path.join(pkg_share, 'urdf/scout_v2/scout_v2_trailer.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/navigation_config.rviz')
    gz_models_path = os.path.join(pkg_share, 'models')
    # default_world_path=os.path.join(pkg_share, 'world/indoor_2.world')
    default_world_path=os.path.join(pkg_share, 'world/wallworldV2.world')

    # Add headless argument declaration
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )
    
    # Create the Gazebo commands
    gazebo_cmd = [
        'gazebo',
        '--verbose',
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
        LaunchConfiguration('world')
    ]
    
    gzserver_cmd = [
        'gzserver',
        '--verbose',
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
        LaunchConfiguration('world')
    ]
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')]), 'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[LaunchConfiguration('model')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    # spawn_entity = launch_ros.actions.Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'scout_v2', '-topic', 'robot_description'],
    #     output='screen'
    # )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'scout_v2',
            '-topic', 'robot_description',
            '-x', '0.0',    # X position
            '-y', '0.0',    # Y position 
            '-z', '0.4',    # Z position
            '-R', '0.0',    # Roll in radians
            '-P', '0.0',    # Pitch in radians
            '-Y', '1.57',   # Yaw in radians (this is 90 degrees in radians)
        ],
        output='screen'
    )
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    trailer_pub = launch_ros.actions.Node(
        package="scout_description",
        executable="test_pub_hitch.py",
        name="test_pub_hitch",
        output="screen",
    )
    nav_loc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "nav2_bringup",
            "localization_launch.py",
            # f"map:={os.path.join(pkg_share, 'maps/wallworldV2.yaml')}",
            f"map:={os.path.join(pkg_share, 'maps/basement.yaml')}",
            f"params_file:={os.path.join(pkg_share, 'config/amcl_params.yaml')}",
            "use_sim_time:=True",
        ],
        output="screen",
    )
    nav_nav = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "nav2_bringup",
            "navigation_launch.py",
            # "params_file:=/home/tomas/tt_ws/src/tese_ws/scout_description/config/nav2_params.yaml",
            f"params_file:={os.path.join(pkg_share, 'config/straight_planner.yaml')}",
            "use_sim_time:=True",
        ],
        output="screen",
    )

    # Timer actions for delayed start
    nav_loc_timer = TimerAction(
        period=20.0,
        actions=[nav_loc]
    )
    
    nav_nav_timer = TimerAction(
        period=40.0,
        actions=[nav_nav]
    )
  

  

    return launch.LaunchDescription([
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=gz_models_path,
        ),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='world', default_value=default_world_path,
                                            description='Absolute path to world sdf file'),                                            
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        headless_arg,
        launch.actions.ExecuteProcess(
            cmd=gazebo_cmd,
            output='screen',
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('headless'))
        ),
        launch.actions.ExecuteProcess(
            cmd=gzserver_cmd,
            output='screen',
            condition=launch.conditions.IfCondition(LaunchConfiguration('headless'))
        ),

        joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz_node,
        trailer_pub,
        nav_loc_timer,
        nav_nav_timer
    ])