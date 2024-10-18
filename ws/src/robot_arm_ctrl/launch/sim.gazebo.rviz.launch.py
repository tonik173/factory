import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_dir = get_package_share_directory('robot_arm_ctrl')
    robot_xacro_path = os.path.join(package_dir, 'urdf', 'robot_arm.xacro')
    control_params = os.path.join(package_dir, 'config', 'controllers.yaml')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'robot_arm.rviz')

    robot_description_content = xacro.process_file(robot_xacro_path, mappings={'hardware_driver' : 'gazebo', 'simulation_controllers':control_params}).toxml()
    robot_description = {"robot_description": robot_description_content}
    
    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=['--ros-args'],
        output="both",
        parameters=[robot_description],
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, '--ros-args'],
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 3 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "rrbot_system_position",
            "-allow_renaming",
            "true",
        ],
    )

    # Controllers 
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_param_file = ["--param-file", control_params]
    joint_trajectory_position_controller_node = Node(
            package="controller_manager",
            executable="spawner",
            output='screen',
            arguments=["joint_trajectory_position_controller"] + controller_manager_param_file + controller_manager_timeout,
            parameters=[
                {'use_sim_time': True},
            ],
    ) 
    joint_trajectory_hand_controller_node = Node(
            package="controller_manager",
            executable="spawner",
            output='screen',
            arguments=["joint_trajectory_hand_controller"] + controller_manager_param_file + controller_manager_timeout,
            parameters=[
                {'use_sim_time': True},
            ],
    )
    node_robot_state_publisher = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        parameters=[
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub_node,
        gz_spawn_entity,
        node_robot_state_publisher,
        joint_trajectory_position_controller_node,
        joint_trajectory_hand_controller_node,
        rviz_node,
    ])