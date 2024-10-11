### About

Installs

- ROS2 Jazzy Jalisco
- ros2_control
- Gazebo Harmonic

### How to launch the docker image

```sh
cd /home/kaan/github/factory/docker/arm-multipass-jazzy
docker image build -t arm-multipass-jazzy-ros -f Dockerfile .
docker compose up -d
docker exec -itd arm-multipass-jazzy-ros terminator

# remove container
docker compose down
docker container rm -f arm-multipass-rolling-ros-1
```

### Cleanup

```sh
./clean.sh
```
