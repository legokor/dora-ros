# Needed for build (?)
...

import pygame
import rclpy
import math
from rclpy.node import Node

from geometry_msgs.msg import Twist

class TeleopKeyPublisher(Node):
    def __init__(self):
        super.__init__("teleop_key_publisher")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Velocity data
        self.linSpeed = 0
        self.linAccel = 0.1
        self.linLimit = (0, 1)
        self.angSpeed = 0
        self.angAccel = 0.2
        self.angLimit = (-math.pi/4, math.pi/4)

    def onKeyPressed(self):
        keys = pygame.key.get_pressed()
        msg = Twist()

        # Movement keys

        if (keys[pygame.K_w]):
            msg.linear.x = self.linSpeed
    
        if (keys[pygame.K_s]):
            msg.linear.x = -self.linSpeed

        if (keys[pygame.K_a]):
            msg.linear.y = self.linSpeed

        if (keys[pygame.K_d]):
            msg.linear.y = -self.linSpeed

        if (keys[pygame.K_q]):
            msg.angular.z = self.angSpeed

        if (keys[pygame.K_e]):
            msg.linear.z = -self.angSpeed
    
        # Accelerational keys

        if (keys[pygame.K_r]):
            if ((self.linSpeed + self.linAccel) in range(self.linLimit)):
                self.linSpeed += self.linAccel

        if (keys[pygame.K_f]):
            if ((self.linSpeed + self.linAccel) in range(self.linLimit)):
                self.linSpeed -= self.linAccel

        if (keys[pygame.K_t]):
            if ((self.angSpeed + self.angAccel) in range(self.angLimit)):
                self.angSpeed += self.angAccel

        if (keys[pygame.K_g]):
            if ((self.angSpeed + self.angAccel) in range(self.angLimit)):
                self.angSpeed -= self.angAccel

        self.publisher.publish(msg)


def main(args=None):
    pygame.init()
    screen = pygame.display.set_mode((300, 300))
    pygame.display.set_caption('Pls adjatok alkohol az enyemet elitam plsssss')

    rclpy.init(args=args)
    teleop_node = TeleopKeyPublisher()
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        teleop_node.onKeyPressed()
        pygame.display.update()
        clock.tick(60)

    pygame.quit()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
