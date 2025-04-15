# Dora ROS

## Setting up

```bash
git clone git@github.com:legokor/dora-ros.git

cd dora-ros

sudo chmod 777 /dev/ttyUSB0

docker-compose up -d
```

### Connect

```bash
docker exec -it dora-ros bash
```

### Run Again

```bash
docker start dora-ros
docker exec -it dora-ros bash
```

### Stop

```bash
docker stop dora-ros
```

## ROS commands

### Start lidar node
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### Stop lidar motor

```bash
ros2 service call /stop_motor std_srvs/srv/Empty {}
```

## rviz on another machine

If you're running rviz in a Docker container you need to give it access to your X server:
```bash
xhost +local:
```

```bash
# TODO: create dora.rviz
rviz2 -d dora.rviz
```
