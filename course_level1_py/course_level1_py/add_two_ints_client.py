#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial
    
class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")
        
    def call_add_two_ints(self, a, b):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Add Two Ints server...")
    
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client_.call_async(request)
        # Adding the partial so we can pass in the request
        future.add_done_callback(partial(self.callback_call_add_two_ints, request=request))

    def callback_call_add_two_ints(self, future, request):
        response = future.result()
        self.get_logger().info(str(request.a) + " + " +
                               str(request.b) + " = " + str(response.sum))
        # time
    
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    node.call_add_two_ints(2, 7)
    node.call_add_two_ints(4, 8)
    node.call_add_two_ints(5, 1)
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()