# Dockerfile for Dora

# docker-compose 1.17 workaround for selecting target image.
ARG BUILD_TARGET=base # Default target

FROM ros:kilted AS base

SHELL ["/bin/bash", "-c"]

WORKDIR /root

# timezones
RUN echo "Europe/Budapest" > /etc/timezone
RUN ln -fs /usr/share/zoneinfo/Europe/Budapest /etc/localtime

# Copying repos
# Warning: This experiment dockerfile will NOT contain the rplidar repo unless previously cloned
COPY .. ./dora-ros

# Make our lives easier
RUN echo "source $HOME/dora-ros/scripts/bashrcExtension.bash" >> .bashrc

# Starts controller with rplidar
FROM base AS dora

CMD /bin/bash -l -c "$HOME/dora-ros/scripts/build-and-run.sh"

FROM base AS dev

CMD bash

# Without defining target in compose, the last defined image will be used.
FROM ${BUILD_TARGET} AS final
