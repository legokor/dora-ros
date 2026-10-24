#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from teleop_control.msg import KeyInputMsg
from nav2_msgs.srv import ManageLifecycleNodes


class Nav2TeleopNode(Node):

    def __init__(self):
        super().__init__('nav2_teleop')
        self.timeout = self.declare_parameter(
            'timeout_sec', 30)
        self.callCooldown = self.declare_parameter(
            'call_cooldown_sec', 3)
        self.keySubscriber = self.create_subscription(
            KeyInputMsg, 'key_teleop_control', self.subscriptionCallback, 10)
        self.nav2Client = self.create_client(
            ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes')

        # For delay between service calls
        self.lastActive = time.time()

        # Waiting for lifecycle manager
        self.get_logger().info('Waiting for lifecycle_manager ...')
        if not self.nav2Client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError('Lifecycle manager is not available.')
        self.get_logger().info('Connected to lifecycle manager')

        # Predefining request:
        self.nav2Request = ManageLifecycleNodes.Request()

        # Making an array of lifecycle types:
        # (They are simply numbers from 0 to 6,
        # but it might change in later updates)
        self.lifecycles = (
            ManageLifecycleNodes.Request.STARTUP,
            ManageLifecycleNodes.Request.SHUTDOWN,
            ManageLifecycleNodes.Request.RESET,
            ManageLifecycleNodes.Request.PAUSE,
            ManageLifecycleNodes.Request.RESUME,
            ManageLifecycleNodes.Request.CONFIGURE,
            ManageLifecycleNodes.Request.CLEANUP
        )

    def subscriptionCallback(self, msg):
        available = (time.time() - self.lastActive) > self.callCooldown
        # +1 at range call because the end of range is not inclusive
        if not msg.ctrl or not available or msg.key not in range(0, 7):
            return

        self.nav2Request.command = self.lifecycles(msg.key)
        self.nav2Client.call(self.nav2Request, self.timeout)
        self.lastActive = time.time()


def main(args=None):
    # Init
    try:
        with rclpy.init(args=args):
            nav2Node = Nav2TeleopNode()

            rclpy.spin(nav2Node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
