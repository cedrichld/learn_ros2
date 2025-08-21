#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

class MoveRobotServerNode(LifecycleNode):
    def __init__(self):
        super().__init__("move_robot_server_node")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.position = 50 # Initial position
        self.move_robot_server_ = None
        self.server_activated_ = False
        
        self.get_logger().info("IN Action Server constructor.")

    # Create ROS2 communications, connect to HW
    def on_configure(self, previous_state: LifecycleState):
        self.declare_parameter("action_name", rclpy.Parameter.Type.STRING)
        self.action_name = self.get_parameter("action_name").value
        self.get_logger().info("IN on_configure")
        self.move_robot_server_ = ActionServer(
            self, 
            MoveRobot, 
            self.action_name, 
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        return TransitionCallbackReturn.SUCCESS
    
    # Activate/Enable HW
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        self.server_activated_ = True
        return super().on_activate(previous_state)
    
    # Activate/Disable HW
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.server_activated_ = False
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.goal_handle_.abort()
        return super().on_deactivate(previous_state)
    
    # Destroy ROS2 communications, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.undeclare_parameter("action_name")
        self.move_robot_server_.destroy() # same as destroy_lifecycle but for other than publisher
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown")
        self.move_robot_server_.destroy()
        self.server_activated_ = False
        return TransitionCallbackReturn.SUCCESS


    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Received a goal.")

        if not self.server_activated_:
            self.get_logger().warn("Rejecting the goal: Node not activated yet.")
            return GoalResponse.REJECT
        
        # Validate the goal request
        if goal_request.target_position not in range(0, 101) or goal_request.velocity <= 0 \
            or not self.server_activated_:
            self.get_logger().warn("Rejecting the goal: Out of bounds.")
            return GoalResponse.REJECT
        
        # Policy: Preempt exesiting goal when receiving new goal
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.goal_handle_.abort()

        self.get_logger().info("Accepting the goal.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a Cancel Request.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle): 
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        # Get request from goal
        target_position = goal_handle.request.target_position
        velocity = goal_handle.request.velocity if (self.position - target_position) < 0 else -goal_handle.request.velocity

        # Execute the action 
        self.get_logger().info("Executing the goal.")
        feedback = MoveRobot.Feedback()
        result = MoveRobot.Result()

        while rclpy.ok():
            # Inside forloop so need to check if goal is still active (if replaced by preempt goal for example)
            if not goal_handle.is_active:                
                result.message = "Preempted by another goal, or node deactivated"
                result.final_position = self.position
                return result
            if goal_handle.is_cancel_requested:
                result.final_position = self.position
                if target_position == self.position:
                    result.message = "Success"
                else:
                    result.message = "Goal Canceled"
                    goal_handle.canceled()
                return result
            
            # Update position
            if self.position == target_position:
                result.final_position = self.position
                result.message = "Success"
                goal_handle.succeed()
                return result
            if abs(velocity) > abs(target_position - self.position):
                self.position = target_position
            else:
                self.position += velocity
            
            if self.position < 0 or self.position > 100:
                self.get_logger().error("Position out of bounds.")
                result.final_position = self.position
                goal_handle.abort()
                return result
            
            self.get_logger().info(str(self.position))
            feedback.current_position = self.position
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
