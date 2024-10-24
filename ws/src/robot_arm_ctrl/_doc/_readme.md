### Prerequisits

sudo apt install pip
python3 -m pip install pysparkplug
https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.htm

```sh
cd <repo-path>/ws
colcon build --packages-select robot_arm_ctrl
source install/setup.bash                                           ### always execute in new terminal
```

### Display robot in RViz

Launches RViz2 with robot arm and the Joint State Publisher.
If this works, the robot description in the URDF file is correct.

```sh
ros2 launch robot_arm_ctrl display.launch.py
```

To start the robot with the ros2_controller, launch

### Forward controller

Moves arm and hand by opening two separate test nodes

```sh
ros2 launch robot_arm_ctrl robot.launch.py                          # default: controller_type:=forward
ros2 launch robot_arm_ctrl sim.gz.rviz.launch.py log_level:=debug   # default: log_level:=info
# in a second terminal
ros2 launch robot_arm_ctrl test_forward_controller.launch.py
```

### Joint trajectory controller

Moves arm and hand simultaneously

```sh
ros2 launch robot_arm_ctrl robot.launch.py controller_type:=joint-trajectory log_level:=info
ros2 launch robot_arm_ctrl sim.gz.rviz.launch.py controller_type:=joint-trajectory log_level:=info
ros2 launch robot_arm_ctrl sim.webots.rviz.launch.py   # for webots
ros2 launch robot_arm_ctrl sim.gazebo.rviz.launch.py   # for gazebo
# in a second terminal
ros2 launch robot_arm_ctrl test_joint_trajectory_controller.launch.py log_level:=info
```

### Trajectory publisher

Open another terminal and run

```sh
ros2 launch robot_arm_ctrl trajectory_publisher.launch.py
```

### ros2_control insights

```sh
ros2 launch robot_arm_ctrl trajectory_publisher.launch.py
```

### ros2 control commands

Play with the ros2 controll. 
Open another terminal and run

```sh
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 control list_hardware_components -v
ros2 control set_controller_state joint_trajectory_position_controller active
```

#### Replace controller

```sh
ros2 control load_controller joint_trajectory_position_controller --set-state inactive
ros2 control switch_controllers --activate joint_trajectory_position_controller --deactivate forward_position_controller
ros2 launch robot_arm_ctrl test_joint_trajectory_controller.launch.py
```

#### Other commands

```sh
ros2 control view_controller_chains
ros2 run rqt_controller_manager rqt_controller_manager 
```

There is a rqt, controller manager plugin.


### References

- [ROSCON2022 ros_control2 workshop](https://github.com/ros-controls/roscon2022_workshop)
- [Mixin c++ and python](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/)
- [RRBot Example](https://control.ros.org/master/doc/ros2_control_demos/example_1/doc/userdoc.html)