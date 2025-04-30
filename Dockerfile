# dora Docker

FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

## update and install packages
RUN apt-get update && \
    apt-get upgrade -y && \
\
    apt-get install -y \
        ranger neovim curl btop tree \
\
        ros-dev-tools \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-rplidar-ros \
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

# ros copy workspace
COPY ./ros2_ws/* /root/ros2_ws/

# setup ros environment in shell
RUN echo 'source /root/ros2_ws/install/setup.bash' >> /root/.bashrc

# build our packages
WORKDIR /root/ros2_ws/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

CMD "cd /root/ros2_ws/ && ros2 launch controller src/controller/launch/launch_dora.xml"
