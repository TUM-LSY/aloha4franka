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
    urdf_file = "urdf/aloha_gripper.urdf.xacro"

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

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_path,
        " ",
        "device:=",
        device,
        " ",
        "uncontrolled:=",
        uncontrolled,
    ])

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

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "-c", "controller_manager"],
        output="screen",
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_tf_static": True,
                "publish_frequency": 30.0,
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory(description_pkg_name),
                "rviz",
                "view_config.rviz",
            ),
        ],
    )

    nodes = [
        controller_manager,
        gripper_controller_spawner,
        joint_state_broadcaster_spawner,
        robot_state_publisher,
        # rviz,
    ]

    return LaunchDescription([
        *args,
        GroupAction(
            actions=[
                PushRosNamespace(namespace=namespace),
                *nodes,
            ]
        ),
    ])
