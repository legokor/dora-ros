# Dora ROS

## Setting up

Clone repo to get compose file:
```bash
git clone git@github.com:legokor/dora-ros.git
cd dora-ros
```

Or just download it:
```bash
wget https://github.com/legokor/dora-ros/raw/refs/heads/master/docker-compose.yml
```

<!-- TODO: use compose to set LiDar permissions -->
Permission to access the LiDar:
```bash
sudo chmod 777 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

### Start container

```bash
docker-compose up -d
```
By default this starts everything.

### Connect to container

```bash
docker exec -it dora-ros bash
```

### Restart container

```bash
docker start dora-ros
```

### Stop container

```bash
docker stop dora-ros
```

## ROS commands

### Stop lidar motor

```bash
ros2 service call /stop_motor std_srvs/srv/Empty {}
```

## Run rviz on another machine

If you're running rviz in a Docker container you need to give it access to your X server:
```bash
xhost +local:
```

Running like this gives the container access to your X11 session and starts `rviz`:
```bash
docker run --rm -itv /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY ghcr.io/legokor/dora-ros-rviz:latest
```
