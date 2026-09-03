#!/usr/bin/env python3

import pygame
import rclpy
import os
import time
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from ament_index_python.packages import get_package_share_directory

# Msg import
from teleop_control.msg import KeyInputMsg
from teleop_control.msg import MouseInputMsg


# Responsible for transforming keyboard and mouse input
# into ROS2 messages using Pygame.
class TeleopKeyPublisher(Node):

    # Msg variables (Generating only once is enough)
    Mouse_msg = MouseInputMsg()
    Key_msg = KeyInputMsg()

    def __init__(self):
        # Node and publisher decleration
        super().__init__("teleop_key_publisher")
        self.keyPublisher = self.create_publisher(
                            KeyInputMsg, "key_teleop_control", 10)
        self.mousePublisher = self.create_publisher(
                            MouseInputMsg, "mouse_teleop_control", 10)
        self.publishRate = 1/10    # 1/Hz = sec
        # Due to the program's logic, the actual publish rate is a bit slower

    # Transforms pygame keyboard input to KeyInputMsg and publishes it
    def onKeyPressed(self, keyEvent):

        # Selecting pressed key
        self.Key_msg.key = keyEvent.key

        # Selecting modifier keys
        # event.mod is a bitmask and each bit preresents a modifier key.
        self.Key_msg.shift = bool(keyEvent.mod & pygame.KMOD_SHIFT)
        self.Key_msg.ctrl = bool(keyEvent.mod & pygame.KMOD_CTRL)
        self.Key_msg.alt = bool(keyEvent.mod & pygame.KMOD_ALT)

        # Publishing
        self.keyPublisher.publish(self.Key_msg)

    # Transforms pygame mouse input to MouseInputMsg and publishes it
    def onMousePressed(self, mouseEvent):
        # Generating msg
        msg = MouseInputMsg()

        # Selecting pressed mouse buttons and mousewheel scroll
        match mouseEvent.button:
            case 1:
                msg.mouse_left = True
            case 2:
                msg.mouse_middle = True
            case 3:
                msg.mouse_right = True
            case 4:
                msg.mouse_wheel_down = True
            case 5:
                msg.mouse_wheel_up = True

        # Publishing
        self.mousePublisher.publish(msg)

    def onMouseLifted(self, mouseEvent):
        # Generating msg
        msg = MouseInputMsg()

        # Selecting pressed mouse buttons and mousewheel scroll
        match mouseEvent.button:
            case 1:
                msg.mouse_left = True
            case 2:
                msg.mouse_middle = True
            case 3:
                msg.mouse_right = True
            case 4:
                msg.mouse_wheel_down = True
            case 5:
                msg.mouse_wheel_up = True

        # Publishing
        self.mousePublisher.publish(msg)


def main(args=None):

    # Initializing PyGame
    pygame.init()
    screen = pygame.display.set_mode((600, 450))
    pygame.display.set_caption('Kattints a képre és nyomj wasd-ot')
    # Setting repeat events for event-based keyhold (faster than polling)
    pygame.key.set_repeat(100)

    # Initializing TeleopNode
    rclpy.init(args=args)
    teleop_node = TeleopKeyPublisher()

    # Reading in image for background
    package_share_path = get_package_share_directory('teleop_control')
    image_path = os.path.join(package_share_path, "images", "listening.png")
    bg = pygame.image.load(image_path)

    # Main event loop
    while rclpy.ok():

        # Events
        for event in pygame.event.get():
            match event.type:
                case pygame.QUIT:
                    rclpy.try_shutdown()
                case pygame.KEYDOWN:
                    teleop_node.onKeyPressed(event)
                case pygame.MOUSEBUTTONDOWN:
                    teleop_node.onMousePressed(event)

        # Allowing rclpy to update data
        rclpy.spin_once(teleop_node, timeout_sec=0)

        # Updating display to allow resizing
        pygame.display.update()
        screen.blit(bg, (0, 0))

        # Process sleep in seconds
        time.sleep(teleop_node.publishRate)

    pygame.quit()
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
