import os
from setuptools import Command
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource 

def generate_launch_description():
    ## Gazebo Stuff
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
 
 
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'arm_sensor'],
                    output='screen')


    # Get URDF via xacro
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("arm_robot"),
            "",
            "arm_sensor.rviz",
        ]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_robot"),
                    "arm_sensor.urdf.xacro",
                ]
            )
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str), "use_sim_time": True}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="log"
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
        rviz2_node,
        robot_joint_state_publisher,
        gazebo,
        spawn_entity
    ]

    return LaunchDescription(nodes)