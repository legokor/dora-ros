FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-jazzy-rviz2 && \
    rm -rf /var/lib/apt/lists/*

RUN echo source /opt/ros/jazzy/setup.bash >> /root/.bashrc

CMD ["/bin/bash", "-lc", "rviz2"]
