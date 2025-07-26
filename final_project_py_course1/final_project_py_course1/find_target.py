#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import TurtleSpawnInfo # Custom
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from functools import partial

from math import hypot
# from typing import Union
    
class FindTargetNode(Node):
    def __init__(self):
        super().__init__("find_target")
        self.get_logger().info("Find Target Started.")

        self.spawn_sub_ = self.create_subscription(TurtleSpawnInfo, "new_spawn_info", self.callback_spawn_sub, 5)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 1)
        self.target_pub_ = self.create_publisher(Pose, "target_turtle", 1)
        self.kill_client_= self.create_client(Kill, "/kill")
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /kill server...")
        
        self.turtle_pose_: Pose = None
        self.targets_ = {}
        self.current_target_name_ = ""
        self.current_target_pose_: Pose = None
        self.threshold_distance_ = 0.5
    
    def callback_spawn_sub(self, new_target: TurtleSpawnInfo):
        self.targets_[new_target.name] = {"x": new_target.x, "y": new_target.y}
        self.single_target_compare(new_target)

    def callback_pose(self, msg: Pose):
        ''' Subscription callback to the pose of turtle1 '''
        self.turtle_pose_ = msg
        if len(self.targets_) > 0 and self.current_target_pose_ != None: 
            if self.target_dist(self.current_target_pose_) < self.threshold_distance_:
                if self.current_target_name_:
                    self.targets_.pop(self.current_target_name_, None)
                    self.call_kill(self.current_target_name_)
                    self.find_new_target()
                else:
                    self.get_logger().warn("No current_target_name_ although self.targets_ > 0")

    def target_dist(self, target: Pose | TurtleSpawnInfo): # either Pose or TurtleSpawnInfo
        if self.turtle_pose_ == None:
            return
        dx = target.x - self.turtle_pose_.x
        dy = target.y - self.turtle_pose_.y
        return hypot(dx, dy)
        
    def single_target_compare(self, new_target: TurtleSpawnInfo):
        def _temp():
            self.current_target_name_ = new_target.name
            self.current_target_pose_.x = new_target.x
            self.current_target_pose_.y = new_target.y

            self.target_pub_.publish(self.current_target_pose_)
            self.get_logger().info(f"New target -> {self.current_target_name_} "
                f"({self.current_target_pose_.x:.2f}, {self.current_target_pose_.y:.2f})")
            
        if self.current_target_name_ == "":
            self.current_target_pose_ = Pose()
            _temp() 
        elif self.turtle_pose_ != None:
            if self.target_dist(new_target) < self.target_dist(self.current_target_pose_):
                _temp()

    def find_new_target(self):
        if not self.targets_:
            self.current_target_name_ = ""
            self.current_target_pose_: Pose = None
            self.get_logger().info("All targets cleared.")
            return 
        
        def distance(p):
            return hypot(p["x"] - self.turtle_pose_.x,
                         p["y"] - self.turtle_pose_.y)
        
        closest_name, closest_pose = min(
            self.targets_.items(), 
            key=lambda kv: hypot(kv[1]["x"] - self.turtle_pose_.x,
                                 kv[1]["y"] - self.turtle_pose_.y)
        )

        self.current_target_name_ = closest_name
        self.current_target_pose_.x = closest_pose["x"]
        self.current_target_pose_.y = closest_pose["y"]

        self.target_pub_.publish(self.current_target_pose_)
        self.get_logger().info(f"New target -> {closest_name} "
            f"({self.current_target_pose_.x:.2f}, {self.current_target_pose_.y:.2f})")

    def call_kill(self, turtle_name: str):
    
        request = Kill.Request()
        request.name = turtle_name

        future = self.kill_client_.call_async(request)
        # Adding the partial so we can pass in the request
        future.add_done_callback(partial(self.callback_call_kill, request=request))

    def callback_call_kill(self, future, request):
        try: 
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Kill call failed: {e}")
            return
        self.get_logger().info("Killed '" + str(request.name) + "'")

def main(args=None):
    rclpy.init(args=args)
    node = FindTargetNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()