#!/bin/bash

## This script installs runtime packages outside container building
# Volumes keep downloaded packages, so this script should be ran once when creating new volumes
# It should take about 10 mins to finish over wifi.

# Adding Personal Package Archives due to Ubuntu's repos lacking behind
add-apt-repository -y ppa:ubuntuhandbook1/geany  # For built in color themes
add-apt-repository -y ppa:zhangsongcui3371/fastfetch  # Can't live without fastfetch.

packages=(
	# ROS packages and dev tools	
	python3-rosdep  # automatic ROS dependency installer
    ros-dev-tools   # Dev tools
    ros-${ROS_DISTRO}-xacro   # XML macro runner for URDF files.
    ros-${ROS_DISTRO}-joint-state-publisher   # Publishes join states like how many degrees a wheel has rotated
    ros-${ROS_DISTRO}-navigation2   # Nav2 packages
    ros-${ROS_DISTRO}-nav2-bringup   # Launch files for Nav2
    ros-${ROS_DISTRO}-slam-toolbox   # SLAM packages
    ros-${ROS_DISTRO}-rviz2   # Rviz2
    ros-${ROS_DISTRO}-rviz-default-plugins   # Helpful plugins
    ros-${ROS_DISTRO}-rqt   # Analytics panel for ROS data
    ros-${ROS_DISTRO}-rqt-common-plugins  # Better logging, settings etc...
    # rplidar package is not maintained :/ 
    # ros-${ROS_DISTRO}-rplidar-ros
    
    # Development tools
    geany   # Lightweight GUI code editor
	geany-plugins   # Plugins like LSP client
	thunar   # File explorer
	alacritty   # Terminal emulator
	fastfetch # EXTREMELY IMPORTANT
	
	# GUI tools and themes for setting Rviz outlook you have to manually set the theme in qt5ct/qt6ct
	qt5ct   # Qt5 Control panel
	qt6ct   # Qt6 Control panel
	gnome-themes-extra-data   # Adwaita-dark theme
	adwaita-qt   # Matching qt5 theme for Adwaita-dark
	adwaita-qt6  # Matching qt6 theme for Adwaita-dark
	dbus dbus-x11 dconf-service dconf-cli at-spi2-core x11-utils # Various GUI dependencies
)

apt-get update
apt-get upgrade -y
apt-get install -y "${packages[@]}"
	
# Switching to downloaded themes by setting env vars.
export GTK_THEME="Adwaita-dark"
export QT_QPA_PLATFORMTHEME=qt5ct
