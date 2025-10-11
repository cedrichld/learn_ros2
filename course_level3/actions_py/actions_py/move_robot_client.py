#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import MoveRobot
from std_msgs.msg import Bool


class MoveRobotClientNode(Node):
    def __init__(self):
        super().__init__("move_robot_client_node")
        self.move_robot_client_ = ActionClient(
            self,
            MoveRobot,
            "move_robot")
        self.cancel_move_sub_ = self.create_subscription(
            Bool, "cancel_move", self.callback_cancel_move, 10)

    def send_goal(self, target_position, velocity):
        # Wait for the server
        self.move_robot_client_.wait_for_server()

        # Create a goal
        goal = MoveRobot.Goal()
        goal.target_position = target_position
        goal.velocity = velocity

        # Send the goal
        self.get_logger().info("Sending goal")
        self.move_robot_client_. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)
        
    def callback_cancel_move(self, msg: Bool):
        if msg.data:
            self.cancel_goal()          

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.get_logger().info("Sending a cancel request")
            self.goal_handle_.cancel_goal_async()

    def goal_response_callback(self, future):
        self.goal_handle_ : ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted.")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal got rejected.")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled") # putting STATUS_CANCELED may not be read by client. action may still succeed from CANCELING state though
        self.get_logger().info("Result: " + str(result.final_position))

    def goal_feedback_callback(self, feedback_msg: MoveRobot.Feedback):
        position = feedback_msg.feedback.current_position
        self.get_logger().info("Got Feedback: " + str(position))


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClientNode()
    # node.send_goal(0, 10)
    node.send_goal(70, 1)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
