#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from my_robot_interfaces.msg import TurtleSpawnInfo # Custom
from rclpy.parameter import Parameter
from functools import partial

from random import uniform
import math
    
class TurtleSpawnClient(Node):
    def __init__(self):
        super().__init__("turtle_spawn")
        self.declare_parameter("spawn_period", 0.3)
        spawn_period_ = self.get_parameter("spawn_period").value
        self.add_post_set_parameters_callback(self.parameters_callback)

        self.spawn_client_= self.create_client(Spawn, "/spawn")
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /spawn server...")

        self.pub_info = self.create_publisher(TurtleSpawnInfo, "new_spawn_info", 10)
        self.spawn_timer_ = self.create_timer(spawn_period_, self.randomize_spawn)
        self.get_logger().info("Started turtle_spawn node, spawn_period=" + str(spawn_period_))
        
    def randomize_spawn(self):
        pose = Pose()
        pose.x = uniform(1.0, 10.0)
        pose.y = uniform(1.0, 10.0)
        pose.theta = uniform(-math.pi, math.pi)
        self.call_spawn(pose)
        
    def call_spawn(self, pose: Pose):
    
        request = Spawn.Request()
        request.x = pose.x
        request.y = pose.y
        request.theta = pose.theta

        future = self.spawn_client_.call_async(request)
        # Adding the partial so we can pass in the request
        future.add_done_callback(partial(self.callback_call_spawn, request=request))

    def callback_call_spawn(self, future, request):
        try: 
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Spawn call failed: {e}")
            return
        info = TurtleSpawnInfo()
        info.x, info.y, info.theta, info.name = request.x, request.y, request.theta, response.name
        self.get_logger().info("'" + str(info.name) + "' spawned: x=" + str(info.x) + ", y=" +
                               str(info.y) + ", theta=" + str(info.theta))

        self.pub_info.publish(info)
    
    def parameters_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "spawn_period":
                new_period = float(param.value)
                if new_period <= 0.0:
                    self.get_logger().warn("spawn_period must be > 0")
                    continue
                self.spawn_timer_.cancel()
                self.spawn_timer_ = self.create_timer(new_period, self.randomize_spawn)
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnClient()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()