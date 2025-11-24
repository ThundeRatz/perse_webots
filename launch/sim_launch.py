import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import math


def launch_setup(context, *args, **kwargs):
    odometry_variables = LaunchConfiguration("odometry_variables").perform(context)
    gui = LaunchConfiguration("gui").perform(context) == "true"
    perse_bringup = get_package_share_directory("perse_bringup")
    perse_properties = os.path.join(perse_bringup, "config", "perse_properties.yaml")

    if odometry_variables == "":
        odometry_variables = os.path.join(
            perse_bringup, "config", "odometry_variables.yaml"
        )

    package_dir = get_package_share_directory("perse_webots")

    robot_description_path = os.path.join(package_dir, "resource", "perse.urdf")
    webots = WebotsLauncher(
        world=os.path.join(package_dir, "worlds", "vehicle.wbt"),
        gui=gui,
    )

    perse_driver = WebotsController(
        robot_name="perse_webots",
        parameters=[
            {
                "robot_description": robot_description_path,
                "odometry_variables": odometry_variables,
                "encoder_constant": 1 / 14,
            },
            perse_properties,
        ],
        respawn=True,
    )

    tf_transforms = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_map_broadcaster",
            # Args: x y z yaw pitch roll frame_id child_frame_id
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_lidar_broadcaster",
            arguments=["0.077", "0", "0.21", "0", "0", "0", "base_link", "lidar_link"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_camera_broadcaster",
            arguments=[
                "0.31",
                "0",
                "0.18",
                str(-math.pi / 2),
                "0",
                str(-math.pi / 2),
                "base_link",
                "camera_link",
            ],
            output="screen",
        ),
    ]

    return [
        webots,
        perse_driver,
        *tf_transforms,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("odometry_variables", default_value=""),
        DeclareLaunchArgument("gui", default_value="true"),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
