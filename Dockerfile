# dora Docker

FROM ubuntu:jammy

SHELL ["/bin/bash", "-c"]

RUN apt update -y
RUN apt upgrade -y

RUN apt install -y locales ranger neovim curl

RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN echo 'export EDITOR=nvim' >> /root/.bashrc

RUN echo "Europe/Budapest" > /etc/timezone
RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update
RUN apt install -y ros-dev-tools

RUN apt install -y ros-rolling-ros-base

# RUN echo 'export ROS_DOMAIN_ID=0' >> /root/.bashrc
RUN echo 'source /opt/ros/rolling/setup.bash' >> /root/.bashrc

# RUN apt install -y ros-rolling-rviz2 libogre-next-dev ros-rolling-rviz-ogre-vendor ros-rolling-qt-gui

# RPLIDAR
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git

# our ros node
RUN mkdir -p /root/ros2_ws/src/controller/src /root/ros2_ws/src/controller/include/
COPY ./CMakeLists.txt ./package.xml /root/ros2_ws/src/controller/
COPY ./src/* /root/ros2_ws/src/controller/src/
COPY ./include/* /root/ros2_ws/src/controller/include/controller/
COPY res/* /root/ros2_ws/

WORKDIR /root/ros2_ws/

RUN echo 'source /root/ros2_ws/install/setup.bash' >> /root/.bashrc
RUN source /opt/ros/rolling/setup.bash && colcon build

# ENTRYPOINT /root/entry.sh
