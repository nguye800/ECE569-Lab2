from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            "rrbot.urdf.xacro",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["-d", "rrbot.rviz"],
    )

    return LaunchDescription(
        [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
