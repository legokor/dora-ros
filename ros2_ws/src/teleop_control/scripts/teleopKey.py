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

# Msg import
from teleop_control.msg import KeyInputMsg
from teleop_control.msg import MouseInputMsg

# Responsible for transforming keyboard and mouse input into ROS2 messages using Pygame.
class TeleopKeyPublisher(Node):
	
    def __init__(self):
		# Node and publisher decleration
        super().__init__("teleop_key_publisher")
        self.keyPublisher = self.create_publisher(KeyInputMsg, "key_teleop_control", 10)
        self.mousePublisher = self.create_publisher(MouseInputMsg, "mouse_teleop_control", 10)
        self.publishRate = 1/10    # 1/Hz = sec
        # Due to the program's logic, the actual publish rate is a bit slower

        
	# Transforms pygame keyboard input to KeyInputMsg and publishes it
    def onKeyPressed(self, keyEvent):

		# Generating msg
		msg = KeyInputMsg()
		
		# Selecting pressed key
		msg.key = keyEvent.key
		
		# Selecting modifier keys
		# event.mod is a bitmask and each bit preresents a modifier key. Pressed = 1
		msg.shift = bool(keyEvent.mod & pygame.KMOD_SHIFT)
        msg.ctrl = bool(keyEvent.mod & pygame.KMOD_CTRL)
        msg.alt = bool(keyEvent.mod & pygame.KMOD_ALT)
				
		# Publishing
		self.keyPublisher.publish(msg)
		
	# Transforms pygame mouse input to MouseInputMsg and publishes it
	def onMousePressed(self, mouseEvent):
		# Generating msg
		msg = MouseInput()
		
		# Selecting pressed mouse buttons and mousewheel scroll
		match mouseEvent.button:
			case 1:
				msg.MouseLeft = True;
			case 2:
				msg.MouseMiddle = True;
			case 3:
				msg.MouseRight = True;
			case 4:
				msg.MouseWheelDown = True;
			case 5:
				msg.MouseWheelUp = True;
		
		# Publishing
		self.mousePublisher.pulish(msg)
		

def main(args=None):
	
	# Initializing PyGame
    pygame.init()
    screen = pygame.display.set_mode((600, 450))
    pygame.display.set_caption('Kattints a képre és nyomj wasd-ot')
    pygame.key.set_repeat(100) #Setting repeat events for event-based keyhold (faster than polling)

	# Initializing TeleopNode
    rclpy.init(args=args)
    teleop_node = TeleopKeyPublisher()

    # Reading in image for background
    package_share_path = get_package_share_directory('teleop_control')
    image_path = os.path.join(package_share_path, "images", "listening.png")
    bg = pygame.image.load(os.path.join(image_path))

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
        
        # Process sleep in seconds
        time.sleep(teleop_node.publishRate)

    pygame.quit()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
