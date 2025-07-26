#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from functools import partial
    
class ResetCounterClientNode(Node):
    def __init__(self):
        super().__init__("reset_counter_client")
        self.client_ = self.create_client(SetBool, "reset_counter")

    def call_reset_counter(self, data):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for SetBool Server")
        
        request = SetBool.Request()
        request.data = data

        future = self.client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_reset_counter, request=request))

    def callback_call_reset_counter(self, future, request):
        response = future.result()
        self.get_logger().info("Received Request [" 
            + str(request.data) + "]. " + str(response.message))
    
    
def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterClientNode()
    node.call_reset_counter(True)
    # rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()