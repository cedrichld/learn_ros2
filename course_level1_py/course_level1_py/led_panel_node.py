#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import PowerLed
from my_robot_interfaces.msg import LedInformation
    
class PowerLedServerNode(Node):
    def __init__(self):
        super().__init__("power_led_server")
        self.declare_parameter("led_states", [0, 0, 0])

        self.server_ = self.create_service(
            PowerLed, "set_led", self.callback_set_led)
        self.led_panel_state_pub_ = self.create_publisher(
            LedInformation, "led_panel_state", 10)
        # self.led_panel_timer_ = self.create_timer(
        #     5.0, self.publish_led_panel_state)
        self.get_logger().info("Set Led Server has been started!")
        self.led_status = self.get_parameter("led_states").value

    def callback_set_led(self, request: PowerLed.Request, response: PowerLed.Response):
        led_number = request.led_number
        state = request.state
        
        if led_number >= len(self.led_status) or \
                led_number < 0 or state not in (0, 1):
            response.success = False
            return response
        
        self.led_status[led_number] = state
        self.publish_led_panel_state()
        response.success = True
        self.get_logger().info("Requested LED #" + str(led_number) + ": " +
                               str(state) + ". Success: " + str(response.success))
        return response
    
    def publish_led_panel_state(self):
        msg = LedInformation()
        msg.led_status = self.led_status
        self.led_panel_state_pub_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = PowerLedServerNode()    
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()