#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class MyCustomNode(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")
        self.hw_status_pub_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hw_status) 
        self.get_logger().info("Hw status publisher has been started")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.are_motors_ready = True
        msg.temperature = 41.2
        msg.debug_message = "Nothing"
        self.hw_status_pub_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()