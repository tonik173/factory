from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    RegisterEventHandler, 
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EqualsSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Declare launch arguments
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
            Used only if 'hardware_driver' parameter is set to 'mock'.",
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

    # Initialize/fetch Arguments
    gui = LaunchConfiguration("gui")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    prefix = LaunchConfiguration("prefix")
    controller_type = LaunchConfiguration("controller_type")
    logLevel = LaunchConfiguration("log_level")

    # robot controller simulation file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_arm_ctrl"),
            "config",
            "controllers.yaml",
        ]
    )

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
            "hardware_driver:=gazebo",
            " ",
            "simulation_controllers:=",
            robot_controllers,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_arm_ctrl"), "rviz", "robot_arm.rviz"]
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
    
    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch", "/gz_sim.launch.py"]
        ),
        launch_arguments={"ign_args": " -r -v 3 empty.sdf"}.items(),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot_arm",
        arguments=["-name", "RobotArm", "-topic", "robot_description", '--ros-args', '--log-level', logLevel],
        output="screen",
    )

    # support
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

    # Controllers: launches either the forward or joint-trajectory controller for both, the arm and hand. 
    robot_controller_spawners = [
        # Forward controllers
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
        # Joint-trajectory controller
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

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_spawn_robot,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_fts_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo_spawn_robot,
            on_start=[
                TimerAction(
                    period=1.5,
                    actions=[fts_broadcaster_spawner],
                ),
            ],
        )
    )

    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[TimerAction(period=3.0, actions=[controller],),],
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
        gazebo,
        gazebo_spawn_robot,
        robot_state_pub_node,
        rviz_node,
        delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        delay_fts_broadcaster_spawner_after_ros2_control_node,
    ]

    return LaunchDescription(declared_arguments 
                             + nodes
                             + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner)