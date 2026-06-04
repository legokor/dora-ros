ARG FROM_IMAGE=ros:rolling
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher
ARG OVERLAY_WS

# overwrite defaults to persist minimal cache
RUN rosdep update --rosdistro $ROS_DISTRO && \
    cat <<EOF > /etc/apt/apt.conf.d/docker-clean && apt-get update
APT::Install-Recommends "false";
APT::Install-Suggests "false";
EOF

# clone overlay source
WORKDIR $OVERLAY_WS/src
RUN cat <<EOF | vcs import .
repositories:
  ros2/demos:
    type: git
    url: https://github.com/ros2/demos.git
    version: ${ROS_DISTRO}
EOF

RUN bash -e <<'EOF'
# Saját csomagok
MY_DEBS="nano git tmux htop"

(
  # Rosdep auto függőség felderítés. Most még egybe a kettő (nincs külön exec/build, fel kéne címkézni)
  rosdep install -y \
    --from-paths \
      ros2/demos/demo_nodes_cpp \
      ros2/demos/demo_nodes_py \
    --ignore-src \
    --reinstall \
    --simulate \
    | grep 'apt-get install' \
    | awk '{gsub(/'\''/,"",$4); print $4}'

  # Saját függőségek hozzáadása
  for pkg in $MY_DEBS; do echo "$pkg"; done

  # Listába rakás és nevek elmentése
) | sort -u > /tmp/packages.txt
EOF

# multi-stage for building
FROM $FROM_IMAGE AS builder
ARG OVERLAY_WS

# install build dependencies
COPY --from=cacher /tmp/build_debs.txt /tmp/build_debs.txt
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=cacher,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=cacher,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    < /tmp/build_debs.txt xargs apt-get install -y

# build overlay source
WORKDIR $OVERLAY_WS
COPY --from=cacher $OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --packages-select \
        demo_nodes_cpp \
        demo_nodes_py \
      --mixin release

# multi-stage for running
FROM $FROM_IMAGE-ros-core AS runner
ARG OVERLAY_WS

# install exec dependencies
COPY --from=cacher /tmp/exec_debs.txt /tmp/exec_debs.txt
RUN --mount=type=cache,target=/etc/apt/apt.conf.d,from=cacher,source=/etc/apt/apt.conf.d \
    --mount=type=cache,target=/var/lib/apt/lists,from=cacher,source=/var/lib/apt/lists \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    < /tmp/exec_debs.txt xargs apt-get install -y

# setup overlay install
ENV OVERLAY_WS=$OVERLAY_WS
COPY --from=builder $OVERLAY_WS/install $OVERLAY_WS/install
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run launch file
CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener_launch.py"]
