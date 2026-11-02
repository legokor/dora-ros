# dora Docker

## Before fresh building run "docker create volume dora-vol"

FROM ros:kilted AS base

SHELL ["/bin/bash", "-c"]

WORKDIR ~

# timezones
RUN echo "Europe/Budapest" > /etc/timezone
RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime

RUN apt-get update && \
    apt-get upgrade -y --no-install-recommends && \
\
    apt-get install -y --no-install-recommends \
        ranger neovim curl btop tree unzip python3-pip \
\
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-state-publisher \
        # rplidar package is not maintained :/ \
        # ros-${ROS_DISTRO}-rplidar-ros \
\
    # clean up filesystem \
    && rm -rf /var/lib/apt/lists/*

# cloning base repos:
RUN git clone -b Nav2_feature --depth=1 https://github.com/legokor/dora-ros.git
RUN git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git dora-ros/ros2_ws/src/

# make our lives easier
RUN echo "source $HOME/dora-ros/scripts/bashrcExtension.bash" >> .bashrc

# starts controller with rplidar
FROM base AS dora

CMD /bin/bash -l -c "${WORK_DIR}/dora-ros/scripts/build-and-run.sh"

# Starts devcontainer
FROM base AS dev

CMD bash

EXPOSE 22
