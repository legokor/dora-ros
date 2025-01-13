# Dora ROS

## Running

### Set up

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

## rviz

```bash
rviz2 -d valami.rviz
```

## Stop motor

```bash
# in docker
ros2 service call /stop_motor std_srvs/srv/Empty {}
```
