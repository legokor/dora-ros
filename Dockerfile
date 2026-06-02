# dora Docker
## spaces at the end of lines for appeasing the lsp gods

ARG GIT_BRANCH=main

# base sets up ROS environment
FROM ros:kilted AS base

SHELL ["/bin/bash", "-c"]

## update and install packages
RUN apt-get update && \
	apt-get upgrade -y && \
	\
	apt-get install -y \
	ranger neovim curl btop tree unzip python3-pip nano\
	\
	ros-dev-tools \
	ros-${ROS_DISTRO}-xacro \
	ros-${ROS_DISTRO}-joint-state-publisher
# rplidar package is not maintained :/ \
# ros-${ROS_DISTRO}-rplidar-ros \
# \
# clean up filesystem: not in this stage, as the package lists are needed for later stages \
# && rm -rf /var/lib/apt/lists/*

# make our lives easier
RUN echo \
	$'export EDITOR=nvim\n' \
	$'alias py=python3\n' \
	$'alias c=clear\n' \
	>> /root/.bashrc

# timezones
RUN echo "Europe/Budapest" > /etc/timezone
RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime

# auto setup ros environment in shell
# at this point, this file does not exist
RUN echo 'source /root/dora-ros/ros2_ws/src/install/setup.bash' >> /root/.bashrc

FROM base AS builder

ARG GIT_BRANCH
# ros copy workspace
# RUN cd /root/ && git clone --depth=1 https://github.com/legokor/dora-ros.git

# only copy the active branch
RUN cd /root/ && git clone --branch ${GIT_BRANCH} --depth=1  https://github.com/legokor/dora-ros.git

# RPLIDAR copy
RUN cd /root/dora-ros/ros2_ws/src/ && \
	git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git

# build the project
RUN source /root/dora-ros/scripts/build.sh

# build if running in CI, run on container start
CMD ["/bin/bash", "-l", "/root/dora-ros/scripts/${__DORA_CI_ACTION:-run}.sh"]

# Bare minimum(?) for running the robot
# This shouldn't include neovim, probably ros-dev-tools, needs to be looked into
FROM base AS prod

# copy the correct script
COPY --from=builder /root/dora-ros/scripts/no-build-run.sh /root/dora-ros/scripts/no-build-run.sh
# copy the result of building
COPY --from=builder /root/dora-ros/ros2_ws/src/install /root/dora-ros/ros2_ws/src/install

# run automatically
CMD ["/bin/bash", "-l", "/root/dora-ros/scripts/no-build-run.sh"]

# rviz multistage
FROM builder AS rviz

# Meg lehet spórolni az update és upgrade-et, ha nem removeoljuk a package liste-t a base-ben
RUN	apt-get install -y ros-${ROS_DISTRO}-rviz2

# start rviz on container start
CMD ["/bin/bash", "-lc", "rviz2"]

FROM builder AS dev

# we remove neovim because we need the newest version for development
RUN apt-get remove -y neovim && \
	apt-get install -y bash-completion luarocks ripgrep clangd

# install newest neovim appimage from github releases
RUN cd /tmp && \
	curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.appimage && \
	chmod u+x nvim-linux-x86_64.appimage && \
	./nvim-linux-x86_64.appimage --appimage-extract && \
	mv squashfs-root /nvim-squashfs-root && \
	ln -s /nvim-squashfs-root/AppRun /usr/bin/nvim

RUN cd /root/dora-ros/ \
	&& git remote set-url origin git@github.com:legokor/dora-ros.git\
	&& git fetch --unshallow # because of shallow copy, we need to fetch everything



