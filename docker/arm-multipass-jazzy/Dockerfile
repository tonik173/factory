# https://hub.docker.com/r/arm64v8/ros/tags
FROM arm64v8/ros:jazzy-perception-noble    

# initialize setup
RUN apt-get update

# install apps
RUN apt-get install -y \
 vim \ 
 wget \
 terminator \
 iputils-ping \
 apt-utils \
 iproute2

# install cleanup
RUN rm -rf /var/lib/apt/lists/*

# user setup
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


ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
