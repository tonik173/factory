import os
import launch
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch_ros.substitutions import FindPackageShare

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
    package_dir = get_package_share_directory('robot_arm_ctrl')
    robot_xacro_path = os.path.join(package_dir, 'urdf', 'robot_arm.xacro')
    control_params = os.path.join(package_dir, 'config', 'controllers.yaml')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'robot_arm.rviz')
    
    # RViz2
    robot_description_content = xacro.process_file(robot_xacro_path).toxml()
    robot_description = {"robot_description": robot_description_content}
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=['--ros-args'],
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, '--ros-args'],
    )

    # Webots
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'factory.wbt'),
        ros2_supervisor=True
    )

    # Controllers 
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    robot_controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            output='screen',
            arguments=["joint_trajectory_position_controller"] + controller_manager_timeout,
            parameters=[
                {'use_sim_time': True},
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output='screen',
            arguments=["joint_trajectory_hand_controller"] + controller_manager_timeout,
            parameters=[
                {'use_sim_time': True},
            ],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=['joint_state_broadcaster'] + controller_manager_timeout,
            parameters=[
                {'use_sim_time': True},
            ],
        ),
#        Node(
#            package="controller_manager",
#            executable="spawner",
#            arguments=["fts_broadcaster", "--controller-manager", "/controller_manager"],
#        )
    ]

    # Webots ros2 controller manager
    robot_driver = WebotsController(
        robot_name='robot_arm',
        parameters=[
            {'robot_description': robot_xacro_path,
             'xacro_mappings': ['hardware_driver:=webots'],
             'use_sim_time': True,
             'set_robot_state_publisher': True},
            control_params
        ],
        respawn=True
    )

    # Wait for the simulation to be ready to start the tools and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start=robot_controller_spawners
    )

    return LaunchDescription([
        webots,
        webots._supervisor,

        robot_state_pub_node,
        rviz_node,

        robot_driver,
        waiting_nodes,

        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])