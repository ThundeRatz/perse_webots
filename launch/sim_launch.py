import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import math


def launch_setup(context, *args, **kwargs):
    odometry_variables = LaunchConfiguration("odometry_variables").perform(context)
    perse_bringup = get_package_share_directory("perse_bringup")
    perse_properties = os.path.join(perse_bringup, "config", "perse_properties.yaml")

    if odometry_variables == "":
        odometry_variables = os.path.join(
            perse_bringup, "config", "odometry_variables.yaml"
        )

    package_dir = get_package_share_directory("perse_webots")

    robot_description_path = os.path.join(package_dir, "resource", "perse.urdf")
    webots = WebotsLauncher(world=os.path.join(package_dir, "worlds", "vehicle.wbt"))

    perse_driver = WebotsController(
        robot_name="perse_webots",
        parameters=[
            {
                "robot_description": robot_description_path,
                "odometry_variables": odometry_variables,
                "encoder_constant": 1 / 14,
            },
            perse_properties
        ],
        respawn=True,
    )

    return [
        webots,
        perse_driver,
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
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
