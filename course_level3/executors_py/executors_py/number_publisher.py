#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from example_interfaces.msg import Int64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.number_ = 1
        self.publish_frequency_ = 1.0
        self.number_publisher_ = self.create_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number)
        self.get_logger().info("Number publisher has been started.")

        # Main callbacks in ROS2:
        # - timers
        # - subscribers
        # - service servers
        # - action servers
        # - futures (in clients)


    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
