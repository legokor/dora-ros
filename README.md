# Dora ROS

## Setting up

Clone repo to get compose file:
```bash
git clone git@github.com:legokor/dora-ros.git
```

<!-- TODO: use compose to set LiDar permissions -->
<!-- Comment: Don't you only need read permission? -->
Permission to access the LiDar:
```bash
sudo chmod 777 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

## Docker setup

### Dora Container

Starting the container:

### Development Container

You can now also run launch-devcon.sh on linux systems to launch the development container.
On Windows 11, you can run WSL2, then run the script from there.
Windows 10 Home systems require a virtual machine due to Microsoft policy.

### Run rviz in the development container

The script should attach your X11 session to the container.
However, run install.sh in the container first to install ogre libraries and rviz.

## ROS commands

### Stop lidar motor

```bash
ros2 service call /stop_motor std_srvs/srv/Empty {}
```

### Moving the robot

```bash
ros2 topic pub -1 /dora/cmd_vel geometry_msgs/msg/Twist '{ linear: { x: 0, y: 0 }, angular: { z: 0 } }'
```

The teleop_control package provides WASD control through PyGame. In the ros2-ws folder run:

```bash
colcon build
source install/setup.bash
ros2 launch teleop_control launchKeyMovement.xml
```

