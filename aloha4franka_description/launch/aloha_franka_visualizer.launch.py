import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.descriptions
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "aloha4franka_description"
    urdf_file = "urdf/aloha_franka.urdf.xacro"

    urdf_path = os.path.join(get_package_share_directory(pkg_name), urdf_file)
    robot_description_content = Command(["xacro ", urdf_path])

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
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
                    "publish_frequency": 30.0,
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
                    get_package_share_directory(pkg_name), "rviz", "view_config.rviz"
                ),
            ],
        ),
    ])
