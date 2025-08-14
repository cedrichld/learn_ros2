#include <cmath>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/target_array.hpp"
#include "my_robot_interfaces/msg/target.hpp"
#include "my_robot_interfaces/srv/kill_turtle.hpp"
#include "turtlesim/msg/pose.hpp" // Pose
#include "geometry_msgs/msg/twist.hpp" // cmd_vel

using namespace std::chrono_literals;
using namespace std::placeholders;
    
class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller"), target(nullptr), pose(nullptr)
    {
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 1,
            std::bind(&TurtleControllerNode::callbackPose, this, _1));
        targets_sub_ = this->create_subscription<my_robot_interfaces::msg::TargetArray>(
            "/targets", 1,
            std::bind(&TurtleControllerNode::callbackTargetFinder, this, _1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 1);
        kill_turtle_client_ = this->create_client<my_robot_interfaces::srv::KillTurtle>("/kill_turtle");
        RCLCPP_INFO(this->get_logger(), "Turtle Controller started");
    }

    void callKillTurtle(std::string turtle_name)
    {
        while (!kill_turtle_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        RCLCPP_INFO(this->get_logger(), "Requesting to kill %s", turtle_name.c_str());

        auto request = std::make_shared<my_robot_interfaces::srv::KillTurtle::Request>();
        request->turtle_name = turtle_name;

        kill_turtle_client_->async_send_request(request, 
            std::bind(&TurtleControllerNode::callbackCallKillTurtle, this, _1));
    }
    
private:
    void callbackPose(const turtlesim::msg::Pose::SharedPtr turtle_pose)
    {
        pose = turtle_pose;
        TurtleControllerNode::turtleController();
    }
    void callbackTargetFinder(const my_robot_interfaces::msg::TargetArray::ConstSharedPtr target_array)
    {
        if (!pose) {
            target.reset();
            return;
        }
        bool has_target = false;
        double min_dist = std::numeric_limits<double>::infinity();

        for (const auto &t : target_array->targets) {
            const double dist = std::hypot(t.x - pose->x, t.y - pose->y);
            if (dist < min_dist) {
                min_dist = dist;
                if (!target) {
                    target = std::make_shared<my_robot_interfaces::msg::Target>(t);
                } else {
                    *target = t;
                }
                has_target = true;
            }
        }

        if (!has_target) {
            // shared pointer for target set to None - same as target = nullptr
            target.reset();
        }
    }

    double wrap_angle(double angle) {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0) angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    void turtleController()
    {
        auto cmd_vel = geometry_msgs::msg::Twist();
        if (!target) {
            // return no velocity
            cmd_pub_->publish(cmd_vel);
        } else {
            const double dist = std::hypot(target->x - pose->x, target->y - pose->y);
            const double angle = TurtleControllerNode::wrap_angle(
                std::atan2(target->y - pose->y, target->x - pose->x) - pose->theta);

            cmd_vel.linear.x = 8.0 * dist;
            cmd_vel.angular.z = 45.0 * angle;
            
            cmd_pub_->publish(cmd_vel);

            if (dist < 0.1) {
                callKillTurtle(target->name);
            }
        }
    }

    void callbackCallKillTurtle(rclcpp::Client<my_robot_interfaces::srv::KillTurtle>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Turtle killed");
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<my_robot_interfaces::msg::TargetArray>::SharedPtr targets_sub_;
    my_robot_interfaces::msg::Target::SharedPtr target;
    turtlesim::msg::Pose::SharedPtr pose;

    rclcpp::Client<my_robot_interfaces::srv::KillTurtle>::SharedPtr kill_turtle_client_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}