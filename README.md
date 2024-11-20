# Dora ROS

## Set up

```bash
git clone git@github.com:legokor/dora-ros.git

cd dora-ros

docker-compose up
```

## Run

```bash
sudo chmod 777 /dev/ttyUSB0

docker start doraros_ros2_1

docker exec -it doraros_ros2_1 bash
```
