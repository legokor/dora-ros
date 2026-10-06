#!/bin/bash

## Helper script to install ros2 on native ubuntu and wsl
# Don't call with sudo because the ROS_DISTRO env-var will be set to root instead of the user

# Ros version:
export ROS_DISTRO=kilted

# Adding Personal Package Archives due to Ubuntu's repos lacking behind
sudo add-apt-repository -y ppa:zhangsongcui3371/fastfetch  # Can't live without fastfetch.

# In case they are missing.
basePackages=(
	software-properties-common nano curl btop tree unzip
    python3 python3-pip
)

packages=(
	python3-rosdep  # automatic ROS dependency installer
    ros-dev-tools   # Dev tools
    ros-${ROS_DISTRO}-desktop   # Ros2 for developtment
    ros-${ROS_DISTRO}-xacro   # XML macro runner for URDF files.
    ros-${ROS_DISTRO}-joint-state-publisher   # Publishes join states like how many degrees a wheel has rotated
    ros-${ROS_DISTRO}-robot-state-publisher   # Publishes join states like how many degrees a wheel has rotated
    ros-${ROS_DISTRO}-navigation2   # Nav2 packages
    ros-${ROS_DISTRO}-nav2-bringup   # Launch files for Nav2
    ros-${ROS_DISTRO}-slam-toolbox   # SLAM packages
    ros-${ROS_DISTRO}-rviz2   # Rviz2
    ros-${ROS_DISTRO}-rviz-default-plugins   # Helpful plugins
    ros-${ROS_DISTRO}-rqt   # Analytics panel for ROS data
    ros-${ROS_DISTRO}-rqt-common-plugins  # Better logging, settings etc...
    ros-${ROS_DISTRO}-joint-state-publisher-gui   # Allows manual joint configuration
    # rplidar package is not maintained :/ 
    # ros-${ROS_DISTRO}-rplidar-ros
    
    # Development tools (VsCode should connect automatically to WSL)
	fastfetch  # EXTREMELY IMPORTANT
	
	# GUI tools and themes for setting Rviz outlook you have to manually set the theme in qt5ct/qt6ct
	fuzzel  # Application launcher for files with .desktop entries
	apwal   # Application launcher which allows you to wire commands to launch icons easily 
	qt5ct   # Qt5 Control panel
	qt6ct   # Qt6 Control panel
	gnome-themes-extra-data   # Adwaita-dark theme
	adwaita-qt   # Matching qt5 theme for Adwaita-dark
	adwaita-qt6  # Matching qt6 theme for Adwaita-dark
)

# Installing base packages
sudo apt update
sudo apt upgrade -y
sudo apt install -y "${basePackages[@]}"

# Setting up ros install:
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Installing
sudo apt update
sudo apt install ros-dev-tools "${packages[@]}"

# Sourcing bash to apply downloaded theme to QT via environment vars
source $HOME/dora-ros/scripts/bashrcExtension.bash

# Gsettings to apply gtk theme and icon
gsettings set org.gnome.desktop.interface gtk-theme "Adwaita-dark"

