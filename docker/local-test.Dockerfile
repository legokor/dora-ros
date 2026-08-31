# dora Docker

## Before fresh building run "docker create volume dora-vol"
# Experiment: See if copying helps online build time

FROM ros:kilted AS base

SHELL ["/bin/bash", "-c"]

WORKDIR root

# timezones
RUN echo "Europe/Budapest" > /etc/timezone
RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime

# installing common programs
RUN apt-get update && \
    apt-get upgrade -y && \
    \
    apt-get install -y \
    software-properties-common nano curl btop tree unzip neovim foot \
    python3 python3-pip 

# Copying repos
# Warning: This experiment dockerfile will NOT contain the rplidar repo unless previously cloned
COPY --chmod=755 .. .

# make our lives easier
RUN echo "source /root/scripts/bashrcExtenstion.bash" >> .bashrc

# starts controller with rplidar
FROM base AS dora

CMD /bin/bash -l -c "${WORK_DIR}/dora-ros/scripts/build-and-run.sh"

FROM base AS dev

# GUI dependencies

RUN apt-get install -y \
	dbus dbus-x11 dconf-service dconf-cli at-spi2-core x11-utils

EXPOSE 22
