import os

import launch_ros.descriptions
from launch import LaunchDescription
from launch.events import Shutdown
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    this_pkg_name = "aloha4franka_bringup"
    description_pkg_name = "aloha4franka_description"
    urdf_file = "urdf/aloha_franka.urdf.xacro"

    urdf_path = os.path.join(
        get_package_share_directory(description_pkg_name), urdf_file
    )
    robot_description_content = Command(["xacro ", urdf_path])

    controller_config = os.path.join(
        get_package_share_directory(this_pkg_name), "config", "controllers.yaml"
    )

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                controller_config,
                {
                    "robot_description": launch_ros.descriptions.ParameterValue(
                        robot_description_content, value_type=str
                    ),
                },
            ],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
            on_exit=Shutdown(),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_position_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": launch_ros.descriptions.ParameterValue(
                        robot_description_content, value_type=str
                    ),
                    "use_tf_static": True,
                    "publish_frequency": 50.0,
                }
            ],
        ),
        Node(
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
        ),
        # Add tf2_tools to visualize the TF tree
        # Node(
        #     package='tf2_tools',
        #     executable='view_frames',
        #     name='view_frames'
        # )
    ])
