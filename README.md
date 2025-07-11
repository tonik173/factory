# factory

## About

Installs

- ROS2 Jazzy Jalisco
- ros2_control
- Gazebo Harmonic

## How to launch the docker image

```sh
cd factory/docker/arm-multipass-jazzy
docker compose build
docker compose up -d
docker exec -itd arm-multipass-jazzy-ros terminator

# remove container
docker compose down
docker container rm -f arm-multipass-rolling-ros-1
```

## Build the robot arm example

```sh
cd src/ws
colcon build --symlink-install
```

### Display robot in RViz

Launches RViz with robot arm and the Joint State Publisher. If this works, the robot description in the URDF file is correct.

```sh
source install/setup.bash 
ros2 launch robot_arm_ctrl display.launch.py
```

