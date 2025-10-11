#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server_node")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []
        self.count_until_server_ = ActionServer(
            self, 
            CountUntil, 
            "count_until", 
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action Server has been started.")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received a goal.")

        # Policy: Refuse new goal if current goal is still active
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("A goal is already active, rejecting new goal")
        #         return GoalResponse.REJECT
        
        # Validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal.")
            return GoalResponse.REJECT
        
        # Policy: Preempt exesiting goal when receiving new goal
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Abort current goal and accept new goal.")
        #         self.goal_handle_.abort()


        self.get_logger().info("Accepting the goal.")
        return GoalResponse.ACCEPT

    # Policy: Add goals to a queue
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a Cancel Request.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle): 
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action 
        self.get_logger().info("Executing the goal.")
        feedback = CountUntil.Feedback()
        counter = 0
        result = CountUntil.Result()
        for _ in range(target_number):
            # Inside forloop so need to check if goal is still active (if replaced by preempt goal for example)
            if not goal_handle.is_active:
                result.result_number = counter
                self.process_next_goal_in_queue()
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal.")
                goal_handle.canceled()
                result.result_number = counter
                self.process_next_goal_in_queue()
                return result
            counter += 1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        ## Set final state FIRST  and return result AFTERWARDS (different from C++)
        # Once done, FIRST set goal final state
        goal_handle.succeed()

        # and THEN send the result
        result.result_number = counter
        self.process_next_goal_in_queue()
        return result
    
    def process_next_goal_in_queue(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
