# dora Docker

FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

## update and install packages
RUN apt-get update && \
    apt-get upgrade -y && \
\
    apt-get install -y \
        ranger neovim curl btop tree \
        bash-comletion python3-serial unzip \
\
        ros-dev-tools \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-state-publisher \
        # rplidar package is not maintained :/ \
        # ros-${ROS_DISTRO}-rplidar-ros \
\
    # clean up filesystem \
    && rm -rf /var/lib/apt/lists/*

# make our lives easier
RUN echo \
    $'export EDITOR=nvim\n' \
    $'alias py=python3\n' \
    $'alias c=clear\n' \
        >> /root/.bashrc

# timezones
RUN echo "Europe/Budapest" > /etc/timezone
RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime

# setup ros environment in shell
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc

# ros copy workspace
RUN cd /root/ && git clone https://github.com/legokor/dora-ros.git

# RPLIDAR
RUN cd /root/dora-ros/ros2_ws/src/ && \
    git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git

RUN source /root/dora-ros/scripts/build.sh

CMD cd /root/dora-ros/ros2_ws/ && \
    source /root/dora-ros/ros2_ws/src/install/setup.bash && \
    ros2 launch controller launch_dora.xml

