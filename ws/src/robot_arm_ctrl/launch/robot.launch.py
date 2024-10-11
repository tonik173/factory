from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'hardware_driver' parameter is to 'mock'.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_driver",
            default_value="real",
            choices=["real", "mock", "gazebo"],
            description="Driver option. Uses the RobotArmHardwareDriver as default.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_type",
            default_value="forward",
            choices=["forward", "joint-trajectory"],
            description="Robot controller type to start (for both, hand and arm).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
                If changed than also joint names in the controllers' configuration have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value='info',
            choices=["debug", "info", "warn", "error", "fatal"],
            description="Sets the log level.",
        )
    )

    # Initialize/fetch arguments
    gui = LaunchConfiguration("gui")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    prefix = LaunchConfiguration("prefix")
    controller_type = LaunchConfiguration("controller_type")
    hardware_driver = LaunchConfiguration("hardware_driver")
    logLevel = LaunchConfiguration("log_level")

    # Get URDF via xacro, applies the parameters
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_arm_ctrl"),
                    "urdf",
                    "robot_arm.xacro",
                ]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "hardware_driver:=",
            hardware_driver,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_arm_ctrl"),
            "config",
            "controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_arm_ctrl"), "rviz", "robot_arm.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        arguments=['--ros-args', '--log-level', logLevel],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=['--ros-args', '--log-level', logLevel],
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, '--ros-args', '--log-level', logLevel],
        condition=IfCondition(gui),
    )

    # support nodes
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', logLevel],
    )
    fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fts_broadcaster", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', logLevel],
    )

    # Controllers 
    robot_controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_position_controller", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', logLevel],
            condition=IfCondition(EqualsSubstitution(controller_type, "forward"))
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["forward_hand_controller", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', logLevel],
            condition=IfCondition(EqualsSubstitution(controller_type, "forward"))
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', logLevel],
            condition=IfCondition(EqualsSubstitution(controller_type, "joint-trajectory"))
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_hand_controller", "--controller-manager", "/controller_manager", '--ros-args', '--log-level', logLevel],
            condition=IfCondition(EqualsSubstitution(controller_type, "joint-trajectory"))
        )
    ]

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node, LogInfo(msg="RViz node started")],
        )
    )

    # Delay controllers start after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[TimerAction(period=1.0, actions=[controller])]
                )
            )
        ]

    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                    on_exit=[TimerAction(period=5.0, actions=[LogInfo(msg="System running :-)")])]
                )
            )
    ]

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        fts_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments 
                             + nodes 
                             + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner)