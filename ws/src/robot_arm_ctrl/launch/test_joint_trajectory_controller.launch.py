from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EqualsSubstitution
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
        ))
    declared_arguments.append(    
        DeclareLaunchArgument(
            "positions",
            default_value='demo',
            choices=["demo", "pick"],
            description="Input set.",
        )
    )
    logLevel = LaunchConfiguration("log_level")
    input_set = LaunchConfiguration("positions")

    nodes = []
    choices=["demo", "pick"]
    for choice in choices:
        print(choice)
        nodes += [
            Node(
                package="robot_arm_ctrl",
                executable="joint_trajectory_position.py",
                name="joint_trajectory_arm",
                parameters=[PathJoinSubstitution([FindPackageShare("robot_arm_ctrl"),"config", "jt-" + choice + "-arm.yaml"])],
                arguments=['--ros-args', '--log-level', logLevel],
                output="both",
                condition=IfCondition(EqualsSubstitution(input_set, choice))
            ),
            Node(
                package="robot_arm_ctrl",
                executable="joint_trajectory_position.py",
                name="joint_trajectory_hand",
                parameters=[PathJoinSubstitution([FindPackageShare("robot_arm_ctrl"),"config", "jt-" + choice + "-hand.yaml"])],
                arguments=['--ros-args', '--log-level', logLevel],
                output="both",
                condition=IfCondition(EqualsSubstitution(input_set, choice))
            ),
        ]

    return LaunchDescription(declared_arguments + nodes)
"controller_name", "joint_trajectory_controller"