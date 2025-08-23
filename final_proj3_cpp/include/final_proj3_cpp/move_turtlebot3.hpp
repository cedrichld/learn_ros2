#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_robot_interfaces/action/move_turtle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
// #include "lifecycle_msgs/srv/get_state.hpp" 
#include "example_interfaces/msg/bool.hpp"

using MoveTurtlebot3 = my_robot_interfaces::action::MoveTurtle;
using MoveTurtlebot3GoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtlebot3>;

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace final_proj3_cpp {

class MoveTurtlebot3Server : public rclcpp::Node
{
public:
    MoveTurtlebot3Server(const rclcpp::NodeOptions &options);

private:

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtlebot3::Goal> goal);
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<MoveTurtlebot3GoalHandle> goal_handle);
    void handle_accepted_callback(const std::shared_ptr<MoveTurtlebot3GoalHandle> goal_handle);
    void execute_goal(const std::shared_ptr<MoveTurtlebot3GoalHandle> goal_handle);    
    void callback_turtlebot3pose(const nav_msgs::msg::Odometry pose);
    void callback_lifecycle_state(const example_interfaces::msg::Bool state);

    rclcpp_action::Server<MoveTurtlebot3>::SharedPtr move_turtlebot3_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    std::shared_ptr<MoveTurtlebot3GoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr turtlebot3_pose_sub_;
    nav_msgs::msg::Odometry turtlebot3_pose_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr turtlebot3_cmd_pub_;
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr lifecycle_state_sub_;
    bool activated_;
};  

}  // namespace final_proj3_cpp