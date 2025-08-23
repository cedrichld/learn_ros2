#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp" 

#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "std_srvs/srv/empty.hpp"

#include "my_robot_interfaces/action/move_turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;
using LifecycleCallbackReturn = 
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace final_proj3_cpp {

class TurtleController : public rclcpp_lifecycle::LifecycleNode
{
public:
    TurtleController(const rclcpp::NodeOptions &options);

    // Spawn Turtle
    void callTurtleSpawn(const double x, const double y, const double theta, const std::string name);
    void callTurtleKill(const std::string name);
    void callClearBackground();

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

private:
    // Spawn Turtle
    void callbackCallTurtleSpawn(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future);

    std::string turtle_name_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_bg_client_;
    

    // Action: Control Turtle 
    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtle::Goal> goal);
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);
    void handle_accepted_callback(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);
    void execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);    
    void callback_turtlepose(const turtlesim::msg::Pose pose);
        
    rclcpp_action::Server<MoveTurtle>::SharedPtr move_turtle_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_;
    turtlesim::msg::Pose turtle_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_cmd_pub_;
    bool activated_;

    rclcpp::SubscriptionOptions sub_options_;
    rclcpp::PublisherOptions pub_options_;
};  

}  // namespace final_proj3_cpp