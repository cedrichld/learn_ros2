import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

### - - UTILITIES - - ###
def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class PIAccumulator:
    def __init__(self, limit: float):
        self.sum = 0.0
        self.limit = abs(limit)

    def update(self, error: float, dt: float) -> float:
        self.sum += error * dt
        self.sum = max(-self.limit, min(self.sum, self.limit))
        return self.sum

    def reset(self):
        self.sum = 0.0
        

class PidTurtleNode(Node):
    def __init__(self):
        super().__init__("pid_turtle")

        ### - - Timer Settings - - ###
        self.declare_parameter("dt", 0.05)
        self.dt = self.get_parameter("dt").value
        
        ### - - PID Parameters for distance and angle - - ###
        PID_default_gains = {
            "P_d": 3.5, "I_d": 0.05, "D_d": 0.05,
            "P_angle": 20.0, "I_angle": 0.001, "D_angle": 0.01,
        }
        for a, b in PID_default_gains.items():
            self.declare_parameter(a, b)
        self.gains = {
            "distance": {
                "P" : self.get_parameter("P_d").value,
                "I" : self.get_parameter("I_d").value,
                "D" : self.get_parameter("D_d").value,
            },
            "angle": {
                "P" : self.get_parameter("P_angle").value,
                "I" : self.get_parameter("I_angle").value,
                "D" : self.get_parameter("D_angle").value,
            }
        }
        self.add_post_set_parameters_callback(self.parameters_callback)
        self.timer_ = self.create_timer(self.dt, self.publish_cmd)
        
        ### - - Set Publishers and Subscribers - - ##
        self.pub_cmd_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        self.sub_pose_ = self.create_subscription(Pose, "/turtle1/pose", 
                                                  self.callback_pose, 1)
        self.sub_target_ = self.create_subscription(Pose, "target_turtle", 
                                    self.callback_target, 1)

        ## - - State Variables - - ##        
        self.pose_: Pose = None # x, y, theta
        self.target_: Pose = None # x, y, theta
        # self.target_.x, self.target_.y = 5.544444561004639, 5.544444561004639

        self.err_d_prev = 0.0
        self.err_angle_prev = 0.0

        ## - - Filtered derivatives - - ##
        self.d_err_d_filt = 0.0
        self.d_err_angle_filt = 0.0
        self.d_alpha = 0.15 # 0 < alpha < 1 (lower stronger smoothing)

        ## - - Integral Accumulators - - ##
        self.I_d = PIAccumulator(limit=2.0)
        self.I_angle = PIAccumulator(limit=1.0)

        ## - - Velocities to publish and thresholds - - ##
        self.v_ = 0.0
        self.w_ = 0.0
        self.threshold_d = 0.01
        self.threshold_angle = 0.01

        self.get_logger().info("Turtle PID has been started with P_d: " + str(self.gains["distance"]["P"]) + 
                               ", I_d: " + str(self.gains["distance"]["I"]) + ", D_d: " + str(self.gains["distance"]["D"])
                                + "; P_angle: " + str(self.gains["angle"]["P"]) + ", I_angle: " + 
                                str(self.gains["angle"]["I"]) + ", D_angle: " + str(self.gains["angle"]["D"]))

    def compute_errors(self):
        ''' Find the distance error to target '''
        dx = self.target_.x - self.pose_.x
        dy = self.target_.y - self.pose_.y
        err_d = math.hypot(dx, dy)

        desired_theta = math.atan2(dy, dx)
        err_angle = wrap_angle(desired_theta - self.pose_.theta)

        return err_d, err_angle

    def pid_step(self):
        ''' Compute velocity an omega based on PID Controller Logic '''
        if self.pose_ == None or self.target_ == None:
            return

        err_d, err_angle = self.compute_errors()
        
        if (err_d < self.threshold_d):
            ## Check if at target
            self.v_ = 0
            self.err_d_prev = 0
            self.I_d.reset()
            if (err_angle < self.threshold_angle):
                self.w_ = 0
                self.err_angle_prev = 0
                self.I_angle.reset()
        else:
            ## Distance loop
            P_d = self.gains["distance"]["P"] * err_d
            I_d_term = self.gains["distance"]["I"] * self.I_d.update(err_d, self.dt)

            d_raw_d = (err_d - self.err_d_prev) / self.dt
            self.d_err_d_filt = (self.d_alpha * d_raw_d +
                                (1.0 - self.d_alpha) * self.d_err_d_filt)
            D_d = self.gains["distance"]["D"] * self.d_err_d_filt

            self.v_ = P_d + I_d_term + D_d
            self.err_d_prev = err_d

            ## Angle loop
            P_angle = self.gains["angle"]["P"] * err_angle
            I_angle_term = self.gains["angle"]["I"] * self.I_angle.update(err_angle, self.dt)

            d_raw_angle = (err_angle - self.err_angle_prev) / self.dt
            self.d_err_angle_filt = (self.d_alpha * d_raw_angle +
                                (1.0 - self.d_alpha) * self.d_err_angle_filt)
            D_angle = self.gains["angle"]["D"] * self.d_err_angle_filt

            self.w_ = P_angle + I_angle_term + D_angle
            self.err_angle_prev = err_angle

    def callback_pose(self, msg: Pose):
        ''' Subscription callback to the pose of turtle1 '''
        self.pose_ = msg

    def callback_target(self, msg: Pose):
        self.target_ = msg
        self.I_d.reset()
        self.I_angle.reset()
        self.get_logger().info("Going to target: " + str((msg.x, msg.y)))

    def publish_cmd(self):
        ''' Publish velocity commands '''
        self.pid_step()
        msg = Twist()
        msg.linear.x = float(self.v_)
        msg.angular.z = float(self.w_)
        self.pub_cmd_.publish(msg)

    # Used if we want to change parameter during runtime
    def parameters_callback(self, params: list[Parameter]): 
        name_map = {
            "P_d": ("distance", "P"),
            "I_d": ("distance", "I"),
            "D_d": ("distance", "D"),
            "P_angle":("angle", "P"),
            "I_angle":("angle", "I"),
            "D_angle":("angle", "D"),
        }
        for p in params:
            if p.name in name_map:
                group, term = name_map[p.name]
                self.gains[group][term] = p.value
                self.get_logger().info(f"{p.name} changed to {p.value}")
            elif p.name == "dt":
                new_period = float(p.value)
                if new_period <= 0.0:
                    self.get_logger().warn("spawn_period must be > 0")
                    continue
                self.dt = new_period
                self.timer_.cancel()
                self.timer_ = self.create_timer(self.dt, self.publish_cmd)
                self.get_logger().info(f"{p.name} changed to {p.value}")

def main(args=None):
    rclpy.init(args=args)
    node = PidTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()