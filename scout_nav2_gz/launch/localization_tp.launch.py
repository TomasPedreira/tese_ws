import launch
import os
import launch_ros
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO

# Create event handler that waits for an output message and then returns actions
def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result

    return on_output


# Launch the robot and the navigation stack with localization

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="scout_nav2_gz"
    ).find("scout_nav2_gz")
    
    diff_drive_loaded_message = "Successfully loaded controller diff_drive_base_controller into state active"
    ekf_ready_message = "Registering sensor"
    navigation_ready_message = "Creating bond timer"
    default_model_path = os.path.join(pkg_share, "urdf/scout_v2/scout_v2.xacro")

    run_headless = LaunchConfiguration("run_headless")
    world = LaunchConfiguration("world")
    use_trailer = LaunchConfiguration("use_trailer")
    model = LaunchConfiguration("model")

    # Including launch files with execute process
    bringup = ExecuteProcess(
        name="launch_bringup",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("scout_nav2_gz"),
                    "launch",
                    "display.launch.py",
                ]
            ),
            "use_rviz:=false",
            ["run_headless:=", run_headless],
            ["world:=", world],
            "use_localization:=true",  # Enable localization instead of SLAM
            ["use_trailer:=", use_trailer],
            ["model:=", model],
        ],
        shell=False,
        output="screen",
    )


    rviz_node = Node(
        condition=IfCondition(NotSubstitution(run_headless)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    localization_nav = ExecuteProcess(
        name="launch_localization_nav",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "localization_launch.py",
                ]
            ),
            "use_sim_time:=True",
            
            ["map:=/home/tomas/tt_ws/src/tese_ws/scout_nav2_gz/maps/tp_indoor_map.yaml"],

        ],
        output="screen",
    )




    waiting_localization_nav = RegisterEventHandler(
        OnProcessIO(
            target_action=bringup,
            on_stdout=on_matching_output(
                ekf_ready_message,
                [
                    LogInfo(
                        msg="EKF ready. Starting navigation..."
                    ),
                    localization_nav,
                ],
            ),
        )
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=[FindPackageShare("scout_nav2_gz"), "/config/nav2_params.yaml"],
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=[
                    FindPackageShare("scout_nav2_gz"),
                    "/rviz/navigation_config.rviz",
                ],
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in headless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="world",
                default_value=[
                    FindPackageShare("scout_nav2_gz"),
                    "/world/ign_indoor/ign_indoor_features.sdf",
                ],
                description="Absolute path to the world file",
            ),
            DeclareLaunchArgument(
                name="use_trailer", 
                default_value="False",
                description="Use the robot model with a trailer",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            bringup,
            rviz_node,
            # waiting_localization_nav,
        ]
    )
