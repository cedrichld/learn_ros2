#include "final_proj3_cpp/move_turtle.hpp"

namespace final_proj3_cpp {

MoveTurtleServer::MoveTurtleServer(const rclcpp::NodeOptions &options) : Node("move_turtle_server_node", options)
{
    this->declare_parameter("turtle_name", "controlled_turtle");
    turtle_name_ = this->get_parameter("turtle_name").as_string();

    // Double check we want reentrant
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
        this,
        "move_turtle", 
        std::bind(&MoveTurtleServer::goal_callback, this, _1, _2),
        std::bind(&MoveTurtleServer::cancel_callback, this, _1),
        std::bind(&MoveTurtleServer::handle_accepted_callback, this, _1),
        rcl_action_server_get_default_options(),
        cb_group_
    );

    rclcpp::SubscriptionOptions sub_options_;
    sub_options_.callback_group = cb_group_;

    rclcpp::PublisherOptions pub_options_;
    pub_options_.callback_group = cb_group_;

    turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/" + turtle_name_ + "/pose", 10, std::bind(&MoveTurtleServer::callback_turtlepose, this, _1), sub_options_);
    turtle_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + turtle_name_ + "/cmd_vel", 10, pub_options_);

    // turtle_spawn_kill_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/turtle_spawn_kill_client/get_state");
    lifecycle_state_sub_ = this->create_subscription<example_interfaces::msg::Bool>(
        "lifecycle_state", 1, std::bind(&MoveTurtleServer::callback_lifecycle_state, this, _1), sub_options_);
    activated_ = false;
    RCLCPP_INFO(this->get_logger(), "Move Turtle ActionServer started.");
}

// void callTurtleSpawnKillStateClient()
// {
//     while (!turtle_spawn_kill_state_client_->wait_for_service(1s)) {
//         RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
//     }

//     auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

//     turtle_spawn_kill_state_client_->async_send_request(request, std::bind(&MoveTurtleServer::callbackcallTurtleSpawnKillStateClient, this, _1));
// }

rclcpp_action::GoalResponse MoveTurtleServer::goal_callback(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtle::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received a goal.");

    bool stop_options = (!activated_) || ((fabs(goal->cmd_lin_x) > 3.0) ||  
            (fabs(goal->cmd_ang_z) > 3.0) || (goal->duration <= 0.0));
    // If goal is not within requested format
    if (stop_options) {
        RCLCPP_INFO(this->get_logger(), "Rejecting the goal - node deactivated or goal parameters out of range.");
        return rclcpp_action::GoalResponse::REJECT;  
    }
    // Policy: Preempt exisiting goal when receiving a new goal
    {
        // Cannot run both mutex at the same time
        std::lock_guard<std::mutex> lock(mutex_);

        if (goal_handle_) {
            if (goal_handle_->is_active()) {
                RCLCPP_INFO(this->get_logger(), "Abort current goal and accept new goal.");
                preempted_goal_id_ = goal_handle_->get_goal_id();
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
        }
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveTurtleServer::cancel_callback(
    const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received Cancel Request.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveTurtleServer::handle_accepted_callback(
    const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing the goal.");
    execute_goal(goal_handle);
}

void MoveTurtleServer::execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
{
    {
        // Not to run both mutex simultaneously
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }
    
    // Get request from goal
    double cmd_lin_x = goal_handle->get_goal()->cmd_lin_x;
    double cmd_ang_z = goal_handle->get_goal()->cmd_ang_z;
    auto duration  = rclcpp::Duration::from_seconds(goal_handle->get_goal()->duration);

    geometry_msgs::msg::Twist turtle_cmd_vel;
    turtle_cmd_vel.linear.x = cmd_lin_x;
    turtle_cmd_vel.angular.z = cmd_ang_z;

    // Execute the action
    auto result = std::make_shared<MoveTurtle::Result>();
    auto feedback = std::make_shared<MoveTurtle::Feedback>();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));  
    const auto start = now();
    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
        if (now() - start >= duration) {
            turtle_cmd_pub_->publish(geometry_msgs::msg::Twist()); // Stop the turtle
            result->position = {turtle_pose_.x, turtle_pose_.y};
            result->message = "Goal Succeeded";
            goal_handle->succeed(result);
            return;
        }
        
        if (!activated_) {
            result->position = {turtle_pose_.x, turtle_pose_.y};
                result->message = "Turtle Spawn/Kill Server deactivated";
                goal_handle->abort(result);
                return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                result->position = {turtle_pose_.x, turtle_pose_.y};
                result->message = "Goal Preempted";
                goal_handle->abort(result);
                return;
            }
        }

        if (goal_handle->is_canceling()) {
            result->position = {turtle_pose_.x, turtle_pose_.y};
            result->message = "Goal Canceled";
            goal_handle->canceled(result);
            return;
        }
        
        turtle_cmd_pub_->publish(turtle_cmd_vel);

        feedback->percent_completion = ((now().seconds() - start.seconds()) / duration.seconds()) * 100.0;
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }   
}

void MoveTurtleServer::callback_turtlepose(const turtlesim::msg::Pose pose) 
{
    turtle_pose_ = pose;
}

void MoveTurtleServer::callback_lifecycle_state(const example_interfaces::msg::Bool state)
{
    activated_ = state.data;
}
// void callbackcallTurtleSpawnKillStateClient(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future)
// {
//     auto response = future.get();
//     if (response->current_state.label == "active") {
//         activated_ = true;
//     } else {
//         activated_ = false;
//     }
// }

} // namespace final_proj3_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(final_proj3_cpp::MoveTurtleServer)


// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MoveTurtleServer>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();
//     rclcpp::shutdown();
//     return 0;
// }