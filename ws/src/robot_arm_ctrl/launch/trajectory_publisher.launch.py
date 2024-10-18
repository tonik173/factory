from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    trajectory_publisher = Node(
        package="robot_arm_ctrl",
        executable="trajectory_publisher.py",
        name="trajectory_publisher",
        arguments=['--ros-args', '--log-level', 'info'],
        output="both", 
    )

    return LaunchDescription([trajectory_publisher])