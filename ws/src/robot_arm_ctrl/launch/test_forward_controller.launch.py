from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value='info',
            choices=["debug", "info", "warn", "error", "fatal"],
            description="Sets the log level.",
        )
    )
    logLevel = LaunchConfiguration("log_level")

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("robot_arm_ctrl"),
            "config",
            "robot_arm_forward_publisher.yaml",
        ]
    )

    hand_goals = PathJoinSubstitution(
        [
            FindPackageShare("robot_arm_ctrl"),
            "config",
            "robot_hand_forward_publisher.yaml",
        ]
    )

    nodes = [
        Node(
            package="robot_arm_ctrl",
            executable="forward_position.py",
            name="forward_arm",
            parameters=[position_goals],
            arguments=['--ros-args', '--log-level', logLevel],
            output="both",
        ),
        Node(
            package="robot_arm_ctrl",
            executable="forward_position.py",
            name="forward_hand",
            parameters=[hand_goals],
            arguments=['--ros-args', '--log-level', logLevel],
            output="both",
        )
    ]

    return LaunchDescription(declared_arguments + nodes)