#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_robot_interfaces/action/move_turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "lifecycle_msgs/srv/get_state.hpp" 
#include "example_interfaces/msg/bool.hpp"

using MoveTurtle = my_robot_interfaces::action::MoveTurtle;
using MoveTurtleGoalHandle = rclcpp_action::ServerGoalHandle<MoveTurtle>;

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace final_proj3_cpp {

class MoveTurtleServer : public rclcpp::Node
{
public:
    MoveTurtleServer(const rclcpp::NodeOptions &options);

    // void callTurtleSpawnKillStateClient()
    // {
    //     while (!turtle_spawn_kill_state_client_->wait_for_service(1s)) {
    //         RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
    //     }

    //     auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    //     turtle_spawn_kill_state_client_->async_send_request(request, std::bind(&MoveTurtleServer::callbackcallTurtleSpawnKillStateClient, this, _1));
    // }

private:

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtle::Goal> goal);
    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);
    void handle_accepted_callback(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);
    void execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle);    
    void callback_turtlepose(const turtlesim::msg::Pose pose);
    void callback_lifecycle_state(const example_interfaces::msg::Bool state);
    // void callbackcallTurtleSpawnKillStateClient(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future)
    // {
    //     auto response = future.get();
    //     if (response->current_state.label == "active") {
    //         activated_ = true;
    //     } else {
    //         activated_ = false;
    //     }
    // }

    rclcpp_action::Server<MoveTurtle>::SharedPtr move_turtle_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    std::shared_ptr<MoveTurtleGoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_;
    turtlesim::msg::Pose turtle_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_cmd_pub_;
    std::string turtle_name_;

    // rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr turtle_spawn_kill_state_client_;
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr lifecycle_state_sub_;
    bool activated_;
};  

}  // namespace final_proj3_cpp