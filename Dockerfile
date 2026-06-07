# dora Docker

## Before fresh building run "docker create volume dora-vol"

FROM ros:kilted AS base

SHELL ["/bin/bash", "-c"]

ENV WORK_DIR=/opt/ros

# installing common programs
RUN apt-get update && \
    apt-get upgrade -y && \
    \
    apt-get install -y \
    software-properties-common nano curl btop tree unzip \
    python3 python3-pip 

# cloning base repos:
RUN cd /opt/ros && git clone -b Nav2_feature --depth=1 https://github.com/legokor/dora-ros.git
RUN cd /opt/ros/dora-ros/ros2_ws/src/ && git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git

# make our lives easier
RUN echo \
    $'alias py=python3\n' \
    $'alias c=clear\n' \
        >> /root/.bashrc

# timezones
RUN echo "Europe/Budapest" > /etc/timezone
RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime

# setup ros environment in shell
RUN echo 'source ${WORK_DIR}/${ROS_DISTRO}/setup.bash' >> /root/.bashrc

# starts controller with rplidar
FROM base AS dora

RUN source ${WORK_DIR}/dora-ros/scripts/build.sh

# build if running in CI, run on container start
CMD ["/bin/bash", "-l", "${WORK_DIR}/dora-ros/scripts/run.sh"]

FROM base AS dev

RUN cd ${WORK_DIR}

EXPOSE 22

# removed neovim becuase nobody uses neovim.