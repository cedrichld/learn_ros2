#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import PowerLed
from my_robot_interfaces.msg import BatteryPercentage
from functools import partial
    
class BatteryClientNode(Node):
    def __init__(self):
        super().__init__("battery_client")
        self.client_ = self.create_client(PowerLed, "set_led")
        self.get_logger().info("Battery Client has been started!")

        # Following edouard:
        self.battery_state_ = "full"
        self.last_time_battery_state_changed_ = self.get_current_time_seconds()
        self.battery_timer_ = self.create_timer(0.1, self.check_battery_state)

        # My own implementation:
        self.percentage_pub_ = self.create_publisher(BatteryPercentage, "battery_percentage", 10)
        self.percentage_pub_timer_ = self.create_timer(1.0, self.publish_battery_percentage)
        self.battery_percentage = 100.0 # float from 0 to 100
        self.charging = False
    
    def get_current_time_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1000000000.0
    
    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state_ == "full":
            if time_now - self.last_time_battery_state_changed_ > 4.0:
                self.last_time_battery_state_changed_ = time_now
                self.battery_state_ = "empty"
                self.get_logger().info("Battery is empty! Charging...")
                self.call_set_led(2, True)
        else:
            if time_now - self.last_time_battery_state_changed_ > 6.0:
                self.last_time_battery_state_changed_ = time_now
                self.battery_state_ = "full"
                self.get_logger().info("Battery is now full!")
                self.call_set_led(2, False)
            
    def call_set_led(self, led_number: float, state: int):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /set_led server...")
    
        request = PowerLed.Request()
        request.led_number = led_number
        request.state = state

        future = self.client_.call_async(request)
        # Adding the partial so we can pass in the request
        future.add_done_callback(partial(self.callback_call_set_led, request=request))

    def callback_call_set_led(self, future, request):
        response = future.result()
        self.get_logger().info("LED number: " + str(request.led_number) + ", state: " +
                               str(request.state) + " = " + str(response.success))
    
    # First personal implementation - not as realistic
    def battery_charge_discharge(self):
        rate = 100.0 / 6.0 if self.charging else -100.0 / 4.0
        pct = self.battery_percentage + rate
        
        self.battery_percentage = max(0.0, min(100.0, pct))
        if self.battery_percentage in (0.0, 100.0):
            self.charging = (self.battery_percentage == 0.0)
            self.call_set_led(2, self.charging)
    
    # First personal implementation - not as realistic
    def publish_battery_percentage(self):
        self.check_battery_state()
        msg = BatteryPercentage()
        msg.battery_percentage = self.battery_percentage
        self.percentage_pub_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = BatteryClientNode()
    # node.battery_charge_discharge()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()