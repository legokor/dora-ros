#!/root/dora-ros/asd/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

## end of ROS imports

from typing import Protocol, override
from dataclasses import dataclass

from threading import Thread
from math import isfinite
from time import sleep

import cv2
import numpy as np
from ultralytics import YOLO

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
        print('empty motor')

class ROSWrapper(Node, DORA):
    vel_x: float = 0
    vel_y: float = 0
    vel_w: float = 0

    lidar: Lidar = Lidar([], -1, -1, -1, -1, -1, -1, -1)

    def __init__(self):
        super().__init__('ROSWrapper')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        def got_vel(v: Twist):
            self.vel_x, self.vel_y, self.vel_w = v.linear.x, v.linear.y, v.angular.z

        self.vel_sub = self.create_subscription(Twist, '/odom', got_vel, 10)

        def got_lidar(s: LaserScan):
            ranges = [f for f in s.ranges if isfinite(f)]
            range_min = min(ranges)
            range_max = max(ranges)
            self.lidar = Lidar(
                s.ranges.tolist(),
                range_min, range_max,
                s.angle_min, s.angle_max,
                s.angle_increment,
                s.time_increment,
                s.scan_time
            )

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', got_lidar, 10)

    @override
    def motor(self, x: float, y: float, w: float):
        v = Twist(
            linear=Vector3(x=float(x), y=float(y), z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=float(w))
        )

        print(f'motor: {x=} {y=} {w=} {self.vel_pub}')

        self.vel_pub.publish(v)

def main():
    with rclpy.init():
        print('rclpy inited')
        node = ROSWrapper()
        print('node created')

        def spin_node():
            try:
                print('spinning node')
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
            print('mi_main')
            mi_main(node)
        except KeyboardInterrupt:
            # print('mi_main interrupted')
            pass
        except err:
            # print('mi_main exception')
            raise err

        node.motor(0,0,0)

        try:
            # print('thread joining')
            t.join()
        except:
            # print('join exception')
            pass

        # print('thread joined')

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)

import time

def mi_main(dora: DORA):
    tolerance = 50       # pixels around center considered “aligned”
    kp = 0.01            # proportional gain for turning kb 0.005 - 0.1
    rotation_speed = 0.2 # search rotation speed
    ball_centered = False  # once True, robot stays still

    while True: # cv.waitKey(1):
        # Get camera frame
        ret, frame = cap.read()
        print('cap read')

        if not ret:
            print('not ret...')
            break

        # Resize for YOLO 
        frame_resized = cv2.resize(frame, (512, 512))
        print('resized')
        frame_resized = cv2.flip(frame_resized, 0)
        print('flipped')

        results = model.predict(frame_resized, conf=0.5, verbose=False) #valószínűleg lassú TODO: optimalizálás
        print('predicted')
        ball_detected = len(results[0].boxes) > 0

        if not ball_centered:
            if ball_detected:
                dora.motor(0, 0, 0)
                # Get ball position
                box = results[0].boxes.xyxy[0]
                name = model.names[int(results[0].boxes.cls[0])]
                x1, y1, x2, y2 = map(int, box)
                cx = (x1 + x2) // 2
                error = cx - 256  # center of 512-wide image

                print(f"{name} detected at x={cx}, error={error}")

                # Draw visuals
                cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame_resized, (cx, (y1 + y2)//2), 4, (0, 0, 255), -1)
                cv2.putText(frame_resized, name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                cv2.imwrite(f'detect-{time.time()}.jpg', frame_resized)

                # Rotate until centered
                if abs(error) <= tolerance:
                    dora.motor(0, 0, 0)
                    ball_centered = True
                    print("Ball centered! Robot goes straight.")
                else:
                    turn = kp * error * rotation_speed
                    dora.motor(0, 0, turn)
            else:
                # Search for ball
                dora.motor(0, 0, rotation_speed)
                print("Searching for ball...")
        else:
            # Ball centered once, go straight
            dora.motor(0, 1, 0)
            print("Ball was centered previously. Robot goes straight.")
        
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    print('main...?')
    main()

