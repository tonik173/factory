from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value='info',
            choices=["debug", "info", "warn", "error", "fatal"],
            description="Sets the log level.",
        ))
    logLevel = LaunchConfiguration("log_level")
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish-state",
            default_value='True',
            description="Publishes state values over Sparkplug",
        ))
    publishState = LaunchConfiguration("publish-state")

    trajectory_publisher = Node(
        package="robot_arm_ctrl",
        executable="trajectory_publisher.py",
        name="trajectory_publisher",
        arguments=['--ros-args', '--log-level', logLevel],
        parameters=[
            {'publish-state': publishState}
        ],
        output="both", 
    )

    return LaunchDescription(declared_arguments + [trajectory_publisher])
