import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

## end of ROS imports

from typing import Protocol, override
from dataclasses import dataclass, field

from threading import Thread
from math import isfinite
from time import sleep

@dataclass
class Lidar():
    # measured in meters, in range [range_min, range_max]
    ranges: list[float]
    range_min: float
    range_max: float

    # measured in radians
    angle_start: float
    angle_end: float
    angle_increment: float

    # time between measurements in seconds
    time_increment: float

    # time since last scan in seconds
    scan_time: float

class DORA(Protocol):
    """DORA the explorer

    Attributes:
        vel_x (float): Horizontal speed in x
        vel_y (float): Horizontal speed in y
        vel_w (float): Rotational speed
    """

    vel_x: float
    vel_y: float
    vel_w: float

    lidar: Lidar

    def motor(self, x: float, y: float, w: float):
        """Sends a speed command to the robot.

        :param x: Horizontal speed in x
        :param y: Horizontal speed in y
        :param w: Rotational speed
        """

class ROSWrapper(DORA, Node):
    vel_x: float = 0
    vel_y: float = 0
    vel_w: float = 0

    lidar: Lidar = Lidar([], -1, -1, -1, -1, -1, -1, -1)

    def __init__(self):
        super().__init__('ROSWrapper')
        self.vel_pub = self.create_publisher(Twist, '/dora/cmd_vel', 10)

        def got_vel(v: Twist):
            self.vel_x, self.vel_y, self.vel_w = v.linear.x, v.linear.y, v.angular.z

        self.vel_sub = self.create_subscription(Twist, '/dora/odom', got_vel, 10)

        def got_lidar(s: LaserScan):
            ranges = [f for f in s.ranges if isfinite(f)]
            range_min = min(ranges)
            range_max = max(ranges)
            self.lidar = Lidar(s.ranges.tolist(), range_min, range_max, s.angle_min, s.angle_max, s.angle_increment, s.time_increment, s.scan_time)

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', got_lidar, 10)

    @override
    def motor(self, x: float, y: float, w: float):
        v = Twist(
            linear=Vector3(x=float(x), y=float(y), z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(w))
        )

        self.vel_pub.publish(v)

def main():
    with rclpy.init():
        node = ROSWrapper()

        def spin_node():
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                # print('spin interrupted')
                pass
            except ExternalShutdownException:
                # print('spin ext shutdown')
                pass

        t = Thread(target=spin_node)
        t.start()

        try:
            mi_main(node)
        except KeyboardInterrupt:
            # print('mi_main interrupted')
            pass
        except:
            # print('mi_main exception')
            pass

        try:
            # print('thread joining')
            t.join()
        except:
            # print('join exception')
            pass

        # print('thread joined')

def mi_main(dora: DORA):
    while True: # cv.waitKey(1):
        dora.motor(0, 0, 1)
        print(f'{dora.vel_x=}, {dora.vel_y=}, {dora.vel_w=} {dora.lidar.range_min=} {dora.lidar.range_max=}')
        sleep(1)

if __name__ == '__main__':
    main()

