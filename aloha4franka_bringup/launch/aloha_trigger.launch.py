import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    this_pkg_name = "aloha4franka_bringup"
    description_pkg_name = "aloha4franka_description"
    urdf_file = "urdf/aloha_trigger.urdf.xacro"

    namespace = LaunchConfiguration("namespace")
    device = LaunchConfiguration("device")
    uncontrolled = LaunchConfiguration("uncontrolled")

    args = [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Change the namespace of the gripper.",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="/dev/ttyUSB0",
            description="Select the device location.",
        ),
        DeclareLaunchArgument(
            "uncontrolled",
            default_value="false",
            description="Whether or not we use torque control.",
        ),
    ]

    urdf_path = os.path.join(
        get_package_share_directory(description_pkg_name), urdf_file
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_path,
            " ",
            "device:=",
            device,
            " ",
            "uncontrolled:=",
            uncontrolled,
        ]
    )

    controller_config = os.path.join(
        get_package_share_directory(this_pkg_name), "config", "controllers.yaml"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_config,
            {"robot_description": robot_description_content},
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    trigger_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trigger_effort_controller", "-c", "controller_manager"],
        output="screen",
    )
    trigger_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trigger_state_broadcaster", "-c", "controller_manager"],
        output="screen",
    )
    # reboot_server = Node(
    #     package="aloha4franka_bringup",
    #     executable="reboot_server",
    #     output="screen",
    # )

    nodes = [
        controller_manager,
        trigger_controller_spawner,
        trigger_state_broadcaster_spawner,
        # reboot_server,
    ]

    return LaunchDescription(
        [
            *args,
            GroupAction(
                actions=[
                    PushRosNamespace(namespace=namespace),
                    GroupAction(
                        actions=[
                            PushRosNamespace(namespace="trigger"),
                            *nodes,
                        ]
                    ),
                ]
            ),
        ]
    )
