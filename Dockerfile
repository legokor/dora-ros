# dora Docker
## spaces at the end of lines for appeasing the lsp gods

# default argument values
ARG DORA_CI_ACTION=run
ARG GIT_BRANCH=master

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

# builds the project
FROM base AS builder

# copy only the active branch
ARG GIT_BRANCH
RUN cd /root/ && git clone --branch ${GIT_BRANCH} --depth=1  https://github.com/legokor/dora-ros.git

# RPLIDAR copy
RUN cd /root/dora-ros/ros2_ws/src/ && \
	git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git

# build the project
RUN source /root/dora-ros/scripts/build.sh

# build if running in CI, run on container start
ARG DORA_CI_ACTION
CMD ["/bin/bash", "-l", "/root/dora-ros/scripts/${DORA_CI_ACTION}.sh"]

# Bare minimum(?) for running the robot
# This shouldn't include neovim, probably ros-dev-tools, needs to be looked into
FROM base AS prod

# copy the correct script
COPY --from=builder /root/dora-ros/scripts/no-build-run.sh /root/dora-ros/scripts/no-build-run.sh
# copy the result of building
COPY --from=builder /root/dora-ros/ros2_ws/src/install /root/dora-ros/ros2_ws/src/install

# in prod, remove the apt list
# this will be better done in the future (building the prod stage from a base ros image)
RUN rm -rf /var/lib/apt/lists/*

# run automatically
CMD ["/bin/bash", "-l", "/root/dora-ros/scripts/no-build-run.sh"]

# rviz multistage
# base-ből, mert build majd ezeknek a csomagjainak a letöltése után lesz
# vagy majd a base-be belerakunk minden csomagot? meg kell nézni előtte a new.Dockerfile-os cachelést
FROM base AS rviz

# Meg lehet spórolni az update és upgrade-et, ha nem removeoljuk a package list-et a base-ben
RUN	apt-get install -y ros-${ROS_DISTRO}-rviz2

# start rviz on container start
CMD ["/bin/bash", "-lc", "rviz2"]

# IN PROGRESS:
# lehúzza az egész repot, lebuildeli, és beállítja a github repot
FROM rviz AS dev

# we remove neovim because we need the newest version for development
RUN apt-get remove -y neovim && \
	apt-get install -y bash-completion luarocks ripgrep clangd \
	# dev programs
	# done in rviz stage
	# ros-${ROS_DISTRO}-rviz2 \
	ros-${ROS_DISTRO}-rqt \
	ros-${ROS_DISTRO}-rqt-common-plugins \
	# experimental programs
	ros-${ROS_DISTRO}-navigation2 \
	ros-${ROS_DISTRO}-nav2-bringup \
	ros-${ROS_DISTRO}-slam-toolbox

# install newest neovim appimage from github releases
RUN cd /tmp && \
	curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.appimage && \
	chmod u+x nvim-linux-x86_64.appimage && \
	./nvim-linux-x86_64.appimage --appimage-extract && \
	mv squashfs-root /nvim-squashfs-root && \
	ln -s /nvim-squashfs-root/AppRun /usr/bin/nvim

# -- BUILD STAGE --

# copy only the active branch
ARG GIT_BRANCH
RUN cd /root/ && git clone --branch ${GIT_BRANCH} --depth=1  https://github.com/legokor/dora-ros.git

# RPLIDAR copy
RUN cd /root/dora-ros/ros2_ws/src/ && \
		git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git

# build the project
RUN source /root/dora-ros/scripts/build.sh

# -- BUILD STAGE --

# git elérés része
# nagyon work in progress, mert autentikáció kell
RUN cd /root/dora-ros/ \
	# fix depth=1 and single branch copying
	&& git config remote.origin.fetch "+refs/heads/*:refs/remotes/origin/*" \
	# because of shallow copy, we need to fetch everything
	&& git fetch --unshallow \
	&& git remote set-url origin git@github.com:legokor/dora-ros.git

# start from the working folder
WORKDIR /root/dora-ros

# no need to run a command
CMD []
