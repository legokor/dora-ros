# dora Docker

## Before fresh building run "docker create volume dora-vol"

FROM ros:kilted AS base

SHELL ["/bin/bash", "-c"]

ENV WORK_DIR=~

# installing common programs
RUN apt-get update && \
    apt-get upgrade -y && \
    \
    apt-get install -y \
    software-properties-common nano curl btop tree unzip neovim \
    python3 python3-pip 

# cloning base repos:
RUN cd ${WORK_DIR} && git clone -b Nav2_feature --depth=1 https://github.com/legokor/dora-ros.git
RUN cd ${WORK_DIR}/dora-ros/ros2_ws/src/ && git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git

# make our lives easier
RUN echo ${WORK_DIR}/dora-ros/scripts/bashrcExtension.txt >> /root/.bashrc

# timezones
RUN echo "Europe/Budapest" > /etc/timezone
RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime

# starts controller with rplidar
FROM base AS dora

CMD /bin/bash -l -c "${WORK_DIR}/dora-ros/scripts/build-and-run.sh"

FROM base AS dev

CMD /bin/bash -l -c "exec /bin/bash"

EXPOSE 22
