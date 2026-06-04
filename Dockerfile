# dora Docker
## spaces at the end of lines for appeasing the lsp gods

# default argument values
ARG DORA_CI_ACTION=run
ARG GIT_BRANCH=master

# base sets up ROS environment
FROM ros:kilted AS base

SHELL ["/bin/bash", "-c"]

#HARDCODEOLVA A ROS_DISTRO A CSOMAGNEVEKBE, MAJD JAVÍTANI?
RUN bash -e <<'EOF'
# Base csomagok
MY_DEBS="ranger neovim curl btop tree unzip python3-pip nano ros-kilted-xacro ros-kilted-joint-state-publisher"
(
  # Csomagok kiírása
  for pkg in $MY_DEBS; do echo "$pkg"; done

  # Szortírozás + dedup
) | sort -u > /tmp/base_debs.txt
EOF

RUN bash -e <<'EOF'
# Rviz csomagok
MY_DEBS="ros-kilted-rviz2"
(
  # Csomagok kiírása
  for pkg in $MY_DEBS; do echo "$pkg"; done

  # Szortírozás + dedup
) | sort -u > /tmp/rviz_debs.txt
EOF

RUN bash -e <<'EOF'
# Dev csomagok
MY_DEBS="bash-completion luarocks ripgrep clangd ros-kilted-rqt ros-kilted-rqt-common-plugins ros-kilted-navigation2 ros-kilted-nav2-bringup ros-kilted-slam-toolbox"
(
  # Csomagok kiírása
  for pkg in $MY_DEBS; do echo "$pkg"; done

  # Szortírozás + dedup
) | sort -u > /tmp/dev_debs.txt
EOF


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

# rviz multistage
# base-ből, mert build majd ezeknek a csomagjainak a letöltése után lesz
# vagy majd a base-be belerakunk minden csomagot? meg kell nézni előtte a new.Dockerfile-os cachelést
FROM base AS rviz

# base és rviz függőségek letöltése
COPY --from=base /tmp/base_debs.txt /tmp/base_debs.txt
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=base,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=base,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && < /tmp/base_debs.txt xargs apt-get install -y

COPY --from=base /tmp/rviz_debs.txt /tmp/rviz_debs.txt
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=base,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=base,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && < /tmp/rviz_debs.txt xargs apt-get install -y

# start rviz on container start
CMD ["/bin/bash", "-lc", "rviz2"]

# IN PROGRESS:
# lehúzza az egész repot, lebuildeli, és beállítja a github repot
FROM rviz AS dev

# dev függőségek letöltése
COPY --from=base /tmp/dev_debs.txt /tmp/dev_debs.txt
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=base,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=base,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && < /tmp/dev_debs.txt xargs apt-get install -y

# we remove neovim because we need the newest version for development
RUN apt-get remove -y neovim

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
