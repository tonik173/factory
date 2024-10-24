# https://hub.docker.com/r/arm64v8/ros/tags
FROM arm64v8/ros:jazzy   

# initialize setup
RUN apt-get update

# install apps
RUN apt-get install -y \
 vim \ 
 wget \
 terminator \
 iputils-ping \
 apt-utils \
 iproute2 \
 pip

 # install ros additions
RUN apt-get update
RUN apt-get install -y ros-jazzy-teleop-twist-keyboard
RUN apt-get install -y ros-jazzy-rviz2
RUN apt-get install -y ros-jazzy-rqt*
RUN apt-get install -y ros-jazzy-xacro
RUN apt-get install -y ros-jazzy-joint-state-publisher-gui 

# install ros2_control
RUN apt-get update
RUN apt-get install -y ros-jazzy-ros2-control
RUN apt-get install -y ros-jazzy-ros2-controllers

# install Gazebo
RUN apt-get update
RUN apt-get install -y ros-jazzy-ros-gz

# Did work for a while then stopped working
RUN apt-get update
RUN apt-get install -y ros-jazzy-gz-ros2-control
# alternative: source local build in ws_jazzy

# webots connector
RUN apt-get update
RUN apt-get install -y ros-jazzy-webots-ros2

# Sparkplug
RUN apt-get update
RUN pip3 install --break-system-packages mqtt_spb_wrapper
RUN pip3 install --break-system-packages pandas

# install cleanup
RUN rm -rf /var/lib/apt/lists/*

# user setup
ARG USERNAME
ARG USERID
ARG GROUPID
RUN groupadd --gid ${GROUPID} ${USERNAME} \
  && useradd -s /bin/bash --uid ${USERID} --gid ${GROUPID} -m ${USERNAME} \
  && mkdir /home/${USERNAME}/.config && chown ${USERID}:${GROUPID} /home/${USERNAME}/.config

# setup sudo
RUN apt-get update \
 && apt-get install -y sudo \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && rm -rf /var/lib/apt/lists/*

# copy files and folders from host to container
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

ENV MACOS_HOST=${MACOS_HOST}
ENV MQTT_BROKER_HOST=${MQTT_BROKER_HOST}
ENV MQTT_BROKER_PORT=${MQTT_BROKER_PORT}

ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
