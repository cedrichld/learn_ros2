#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.declare_parameter("managed_node_names", rclpy.Parameter.Type.STRING_ARRAY)
        self.node_names = self.get_parameter("managed_node_names").value
        self.client = {}
        for node_name in self.node_names:
            service_change_state_name = "/" + node_name + "/change_state"
            self.client[node_name] = self.create_client(ChangeState, service_change_state_name)

    def change_state(self, transition: Transition):
        # Wait for all services
        for node_name in self.node_names:
            self.client[node_name].wait_for_service()
        
        # Fire all requests concurrently
        future = {}
        for node_name in self.node_names:
            request = ChangeState.Request()
            request.transition = transition
            future[node_name] = self.client[node_name].call_async(request)
        
        # Wait for all to complete
        while not all(f.done() for f in future.values()):
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Success log
        for node_name, f in future.items():
            ## Could wait until all are configured, try to configure again in while loop (for HW)
            ok = bool(f.result() and getattr(f.result(), "success", False))
            self.get_logger().info(f"{node_name}: '{transition.label}' {'OK' if ok else 'FAILED'}")

    def initialization_sequence(self):
        # Unconfigured to Inactive
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)
        self.get_logger().info("Configuring OK, now inactive")

        # Inactive to Active
        self.get_logger().info("Trying to switch to activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition)
        self.get_logger().info("Activating OK, now active")

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleNodeManager()
    node.initialization_sequence()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
