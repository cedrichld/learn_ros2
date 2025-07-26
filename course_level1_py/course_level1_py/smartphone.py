import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("smartphone")
        self.sub_ = self.create_subscription(String, "robot_news", self.news_sub, 10)
        self.get_logger().info("Smartphone has been started!")

    def news_sub(self, msg: String):
        msg.data = "I heard: [" + msg.data + "]"
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()