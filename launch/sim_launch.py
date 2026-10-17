import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

# from webots_ros2_driver.webots_controller import WebotsController
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import math

import os

from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from webots_ros2_driver.utils import controller_protocol, controller_ip_address


class WebotsController(ExecuteProcess):
    def __init__(
        self,
        output="screen",
        respawn=False,
        remappings=[],
        namespace="",
        parameters=[],
        robot_name="",
        port="1234",
        **kwargs,
    ):
        webots_controller = os.path.join(
            get_package_share_directory("webots_ros2_driver"),
            "scripts",
            "webots-controller",
        )

        protocol = controller_protocol()
        ip_address = controller_ip_address() if (protocol == "tcp") else ""

        robot_name_option = [] if not robot_name else ["--robot-name=" + robot_name]
        ip_address_option = [] if not ip_address else ["--ip-address=" + ip_address]

        ros_arguments = []
        for item in remappings:
            key, value = item
            remap = f"{key}:={value}"
            ros_arguments.append("-r")
            ros_arguments.append(remap)
        if namespace:
            remap = f"__ns:=/{namespace}"
            ros_arguments.append("-r")
            ros_arguments.append(remap)
        for item in parameters:
            if isinstance(item, dict):
                for key, value in item.items():
                    parameter = [
                        f"{key}:=",
                        (
                            value
                            if isinstance(value, Substitution)
                            else TextSubstitution(text=str(value))
                        ),
                    ]
                    ros_arguments.append("-p")
                    ros_arguments.append(parameter)
        file_parameters = [item for item in parameters if isinstance(item, str)]

        ros_args = ["--ros-args"] if ros_arguments else []
        params_file = ["--params-file"] + (
            ";--params-file;".join(file_parameters).split(";")
            if file_parameters
            else []
        )

        node_name = "webots_controller" + (("_" + robot_name) if robot_name else "")
        super().__init__(
            output=output,
            cmd=[
                webots_controller,
                *robot_name_option,
                ["--protocol=", protocol],
                *ip_address_option,
                ["--port=", port],
                "ros2",
                *ros_args,
                *ros_arguments,
                *params_file,
            ],
            name=node_name,
            respawn=respawn,
            # Set WEBOTS_HOME to package directory to load correct controller library
            additional_env={"WEBOTS_HOME": get_package_prefix("webots_ros2_driver")},
            **kwargs,
        )

    def execute(self, context: LaunchContext):
        return super().execute(context)

    def _shutdown_process(self, context, *, send_sigint):
        return super()._shutdown_process(context, send_sigint=send_sigint)


def launch_setup(context, *args, **kwargs):
    perse_bringup = get_package_share_directory("perse_bringup")
    perse_webots = get_package_share_directory("perse_webots")

    default_mission_points = os.path.join(perse_bringup, "config", "mission_points.yaml")
    mission_points = LaunchConfiguration("mission_points").perform(context) or default_mission_points
    gui = LaunchConfiguration("gui").perform(context) == "true"

    topics = os.path.join(perse_bringup, "config", "topics.yaml")
    perse_properties = os.path.join(perse_bringup, "config", "perse_properties.yaml")
    perse_driver_parameters = os.path.join(perse_webots, "config", "perse_driver.yaml")

    robot_description_path = os.path.join(perse_webots, "resource", "perse.urdf")
    webots = WebotsLauncher(
        world=os.path.join(perse_webots, "resource", "worlds", "circle_arena.wbt"),
        gui=gui,
    )

    perse_driver = WebotsController(
        robot_name="perse_webots",
        parameters=[
            {
                "robot_description": robot_description_path,
                "mission_points": mission_points,
                "encoder_constant": 1 / 14,
            },
            perse_driver_parameters,
            perse_properties,
            topics
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
        DeclareLaunchArgument("mission_points", default_value=""),
        DeclareLaunchArgument("gui", default_value="true"),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
