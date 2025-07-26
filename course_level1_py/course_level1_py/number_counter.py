import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.sub_ = self.create_subscription(Int64, "number", self.callback_number, 10)
        self.pub_ = self.create_publisher(Int64, "number_count", 10)
        self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        self.counter_ = 0
        self.get_logger().info("Number Counter has been started!")

    def callback_number(self, msg: Int64):
        self.counter_ += msg.data
        new_msg = Int64()
        new_msg.data = self.counter_
        self.pub_.publish(new_msg)

    def callback_reset_counter(self, request: SetBool.Request, response: SetBool.Response):
        if (request.data): 
            self.counter_ = 0
            response.success = True
            response.message = "Counter reset to 0"
            return response
        response.success = False
        response.message = "Counter not reset to 0"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()