#include "final_proj3_cpp/move_turtlebot3.hpp"

namespace final_proj3_cpp {

MoveTurtlebot3Server::MoveTurtlebot3Server(const rclcpp::NodeOptions &options) : Node("move_turtlebot3_server_node", options)
{
    // Double check we want reentrant
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    move_turtlebot3_server_ = rclcpp_action::create_server<MoveTurtlebot3>(
        this,
        "move_turtlebot3", 
        std::bind(&MoveTurtlebot3Server::goal_callback, this, _1, _2),
        std::bind(&MoveTurtlebot3Server::cancel_callback, this, _1),
        std::bind(&MoveTurtlebot3Server::handle_accepted_callback, this, _1),
        rcl_action_server_get_default_options(),
        cb_group_
    );

    rclcpp::SubscriptionOptions sub_options_;
    sub_options_.callback_group = cb_group_;

    rclcpp::PublisherOptions pub_options_;
    pub_options_.callback_group = cb_group_;

    turtlebot3_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>( 
        "/odom", 10, std::bind(&MoveTurtlebot3Server::callback_turtlebot3pose, this, _1), sub_options_);
    turtlebot3_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10, pub_options_);

    lifecycle_state_sub_ = this->create_subscription<example_interfaces::msg::Bool>(
        "lifecycle_state", 1, std::bind(&MoveTurtlebot3Server::callback_lifecycle_state, this, _1), sub_options_);
    activated_ = false;
    RCLCPP_INFO(this->get_logger(), "Move Turtlebot3 ActionServer started.");
}

rclcpp_action::GoalResponse MoveTurtlebot3Server::goal_callback(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveTurtlebot3::Goal> goal)
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

rclcpp_action::CancelResponse MoveTurtlebot3Server::cancel_callback(
    const std::shared_ptr<MoveTurtlebot3GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received Cancel Request.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveTurtlebot3Server::handle_accepted_callback(
    const std::shared_ptr<MoveTurtlebot3GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing the goal.");
    execute_goal(goal_handle);
}

void MoveTurtlebot3Server::execute_goal(const std::shared_ptr<MoveTurtlebot3GoalHandle> goal_handle)
{
    {
        // Not to run both mutex simultaneously
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }
    
    // Get request from goal
    double cmd_lin_x = goal_handle->get_goal()->cmd_lin_x;
    double cmd_ang_z = goal_handle->get_goal()->cmd_ang_z;
    double duration  = goal_handle->get_goal()->duration;

    geometry_msgs::msg::TwistStamped turtlebot3_cmd_vel;
    turtlebot3_cmd_vel.twist.linear.x = cmd_lin_x;
    turtlebot3_cmd_vel.twist.angular.z = cmd_ang_z;

    // Execute the action
    auto result = std::make_shared<MoveTurtlebot3::Result>();
    auto feedback = std::make_shared<MoveTurtlebot3::Feedback>();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    rclcpp::WallRate loop_rate(20.0);
    const auto t0 = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        const double curr_t =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        if (curr_t >= duration) break;
        
        if (!activated_) {
            result->position = {turtlebot3_pose_.pose.pose.position.x, turtlebot3_pose_.pose.pose.position.y};
                result->message = "Turtlebot3 Spawn/Kill Server deactivated";
                goal_handle->abort(result);
                return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                result->position = {turtlebot3_pose_.pose.pose.position.x, turtlebot3_pose_.pose.pose.position.y};
                result->message = "Goal Preempted";
                goal_handle->abort(result);
                return;
            }
        }

        if (goal_handle->is_canceling()) {
            result->position = {turtlebot3_pose_.pose.pose.position.x, turtlebot3_pose_.pose.pose.position.y};
            result->message = "Goal Canceled";
            goal_handle->canceled(result);
            return;
        }
        
        turtlebot3_cmd_pub_->publish(turtlebot3_cmd_vel);

        feedback->percent_completion = (curr_t / duration) * 100.0;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }

    turtlebot3_cmd_pub_->publish(geometry_msgs::msg::TwistStamped()); // Stop the turtlebot3
    result->position = {turtlebot3_pose_.pose.pose.position.x, turtlebot3_pose_.pose.pose.position.y};
    result->message = "Goal Succeeded";
    goal_handle->succeed(result);
}

void MoveTurtlebot3Server::callback_turtlebot3pose(const nav_msgs::msg::Odometry pose) 
{
    turtlebot3_pose_ = pose;
}

void MoveTurtlebot3Server::callback_lifecycle_state(const example_interfaces::msg::Bool state)
{
    activated_ = state.data;
}

} // namespace final_proj3_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(final_proj3_cpp::MoveTurtlebot3Server)