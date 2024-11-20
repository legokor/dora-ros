# dora Docker

FROM ubuntu:jammy

SHELL ["/bin/bash", "-c"]

RUN locale  # check for UTF-8

RUN apt update
RUN apt upgrade

RUN apt install locales

RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN locale  # verify settings

RUN echo "Europe/Budapest" > /etc/timezone
RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update
RUN apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update
RUN apt install -y ros-dev-tools

RUN apt install -y ros-rolling-ros-base

RUN echo 'source /opt/ros/rolling/setup.bash' >> ~/.bashrc

RUN apt install -y ros-rolling-rviz2 libogre-next-dev ros-rolling-rviz-ogre-vendor

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

WORKDIR /root/ros2_ws
RUN colcon build --symlink-install
RUN echo 'source /root/ros2_ws/install/setup.bash' >> ~/.bashrc

WORKDIR /root/ros2_ws
