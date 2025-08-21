#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import time


class Node1(Node):
    def __init__(self):
        super().__init__("node1")
        self.timer1_ = self.create_timer(1.0, self.callback_timer1)
        self.timer2_ = self.create_timer(1.0, self.callback_timer2)
        self.timer3_ = self.create_timer(1.0, self.callback_timer3)

    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1")

    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 2")

    def callback_timer3(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3")

class Node2(Node):
    def __init__(self):
        super().__init__("node2")
        self.timer1_ = self.create_timer(1.0, self.callback_timer4)
        self.timer2_ = self.create_timer(1.0, self.callback_timer5)

    def callback_timer4(self):
        time.sleep(2.0)
        self.get_logger().info("cb 4")

    def callback_timer5(self):
        time.sleep(2.0)
        self.get_logger().info("cb 5")


def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()    
    node2 = Node2()    
    executor = SingleThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()