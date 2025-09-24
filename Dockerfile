# dora Docker
## spaces at the end of lines for appeasing the lsp gods

FROM ros:kilted AS base

SHELL ["/bin/bash", "-c"]

## update and install packages
RUN apt-get update && \
    apt-get upgrade -y && \
\
    apt-get install -y \
        ranger neovim curl btop tree unzip python3-pip \
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
RUN echo 'source /root/dora-ros/ros2_ws/src/install/setup.bash' >> /root/.bashrc 

# ros copy workspace
RUN cd /root/ && git clone --depth=1 https://github.com/legokor/dora-ros.git 

# RPLIDAR
RUN cd /root/dora-ros/ros2_ws/src/ && \
    git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git 

RUN source /root/dora-ros/scripts/build.sh 

# build if running in CI, run on container start
CMD ["/bin/bash", "-lc", "/root/dora-ros/scripts/${__DORA_CI_ACTION:-run}.sh"]

# rviz multistage
FROM base AS rviz

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-${ROS_DISTRO}-rviz2 && \
    rm -rf /var/lib/apt/lists/* 

# start rviz on container start
CMD ["/bin/bash", "-lc", "rviz2"]

FROM rviz AS dev

# we remove neovim because we need the newest version for development
RUN apt-get remove -y neovim && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install luarocks ripgrep clangd && \
    rm -rf /var/lib/apt/lists/* 

# install newest neovim appimage from github releases
RUN cd /tmp && \
    curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.appimage && \
    chmod u+x nvim-linux-x86_64.appimage && \
    ./nvim-linux-x86_64.appimage --appimage-extract && \
    mv squashfs-root /nvim-squashfs-root && \
    ln -s /nvim-squashfs-root/AppRun /usr/bin/nvim 

