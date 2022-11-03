from struct import pack
from setuptools import Command
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF via xacro
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("arm_robot"),
            "",
            "arm.rviz",
        ]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_robot"),
                    "arm.urdf.xacro",
                ]
            )
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        robot_state_pub_node,
        rviz2_node
    ]

    return LaunchDescription(nodes)