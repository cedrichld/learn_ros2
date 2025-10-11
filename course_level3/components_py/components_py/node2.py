#!/usr/bin/env python3
from rclpy.node import Node
import time

class Node2(Node):
    def __init__(self):
        super().__init__("node2")
        self.timer4_ = self.create_timer(1.0, self.callback_timer4)
        self.timer5_ = self.create_timer(1.0, self.callback_timer5)

    def callback_timer4(self):
        time.sleep(2.0)
        self.get_logger().info("cb 4")

    def callback_timer5(self):
        time.sleep(2.0)
        self.get_logger().info("cb 5")