#!/usr/bin/env python3

import pygame
import rclpy
import math
import os
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from ament_index_python.packages import get_package_share_directory


from geometry_msgs.msg import Twist

publishRate = 20    #Hz

class TeleopKeyPublisher(Node):
    def __init__(self):
        super().__init__("teleop_key_publisher")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Velocity data
        self.linSpeed = 0.1
        self.linAccel = 0.01
        self.linLimit = (0, 1)
        self.angSpeed = 0.1
        self.angAccel = 0.02
        self.angLimit = (-math.pi/4, math.pi/4)

        # Parameter for accelerational input
        parameter_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                                      description='Defies whether accelerational key input is considered')
        self.declare_parameter("use_accelerational_keys", True, parameter_descriptor)

    def onKeyPressed(self):
        useAccelKeys = self.get_parameter("use_accelerational_keys").get_parameter_value().bool_value
        keys = pygame.key.get_pressed()
        msg = Twist()

        # Movement keys

        if (keys[pygame.K_w]):
            msg.linear.x = self.linSpeed
    
        if (keys[pygame.K_s]):
            msg.linear.x = -self.linSpeed

        if (keys[pygame.K_q]):
            msg.linear.y = self.linSpeed

        if (keys[pygame.K_e]):
            msg.linear.y = -self.linSpeed

        if (keys[pygame.K_a]):
            msg.angular.z = self.angSpeed

        if (keys[pygame.K_d]):
            msg.angular.z = -self.angSpeed
    
        # Accelerational keys

        if (useAccelKeys):
            if (keys[pygame.K_r]):
                if ((self.linSpeed + self.linAccel) <= self.linLimit[1]):
                    self.linSpeed += self.linAccel

            if (keys[pygame.K_f]):
                if ((self.linSpeed - self.linAccel) >= self.linLimit[0]):
                    self.linSpeed -= self.linAccel

            if (keys[pygame.K_t]):
                if ((self.angSpeed + self.angAccel) <= self.angLimit[1]):
                    self.angSpeed += self.angAccel

            if (keys[pygame.K_g]):
                if ((self.angSpeed - self.angAccel) >= self.angLimit[0]):
                    self.angSpeed -= self.angAccel

        self.publisher.publish(msg)


def main(args=None):
    pygame.init()
    screen = pygame.display.set_mode((600, 450))
    pygame.display.set_caption('Pls adjatok alkohol az enyemet elitam plsssss')

    rclpy.init(args=args)
    teleop_node = TeleopKeyPublisher()
    clock = pygame.time.Clock()

    # Reading in image for background
    package_share_path = get_package_share_directory('teleop_control')
    image_path = os.path.join(package_share_path, "images", "listening.png")
    bg = pygame.image.load(os.path.join(image_path))

    running = True
    while running:
        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        teleop_node.onKeyPressed()

        # Allowing rclpy to update data
        rclpy.spin_once(teleop_node, timeout_sec=0)

        # Pygame updates
        pygame.display.update()
        clock.tick(publishRate)
        screen.blit(bg, (0,0))

    pygame.quit()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
