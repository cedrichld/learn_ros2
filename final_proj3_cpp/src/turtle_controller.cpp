#include "final_proj3_cpp/turtle_controller.hpp"

namespace final_proj3_cpp {

TurtleController::TurtleController(const rclcpp::NodeOptions &options) : LifecycleNode("turtle_controller", options)
{
    this->declare_parameter("turtle_name", "controlled_turtle");
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    sub_options_.callback_group = cb_group_;
    pub_options_.callback_group = cb_group_;
    activated_ = false;
    RCLCPP_INFO(this->get_logger(), "Move Turtle ActionServer started.");
}


// Lifecycle Node
LifecycleCallbackReturn TurtleController::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "Configuring...");
    turtle_name_ = this->get_parameter("turtle_name").as_string();

    spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
    kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
    clear_bg_client_ = this->create_client<std_srvs::srv::Empty>("/clear");
    move_turtle_server_ = rclcpp_action::create_server<MoveTurtle>(
        this,
        "move_" + turtle_name_, 
        std::bind(&TurtleController::goal_callback, this, _1, _2),
        std::bind(&TurtleController::cancel_callback, this, _1),
        std::bind(&TurtleController::handle_accepted_callback, this, _1),
        rcl_action_server_get_default_options(),
        cb_group_
    );
    turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/" + turtle_name_ + "/pose", 10, std::bind(&TurtleController::callback_turtlepose, this, _1), sub_options_);
    turtle_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + turtle_name_ + "/cmd_vel", 10, pub_options_);
    
    callClearBackground();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    callTurtleSpawn(5.0, 5.0, 0.0, turtle_name_);

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "Activating...");
    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    activated_ = true;

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleController::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(this->get_logger(), "Deactivating...");   
    activated_ = false;

    // deactivate action server
    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "Cleaning up...");    
    
    TurtleController::callTurtleKill(turtle_name_);
    TurtleController::callClearBackground();

    spawn_client_.reset();
    kill_client_.reset();
    clear_bg_client_.reset();
    move_turtle_server_.reset();
    turtle_pose_sub_.reset();
    turtle_cmd_pub_.reset();

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state; 
    RCLCPP_INFO(this->get_logger(), "Shutting down...");

    TurtleController::callTurtleKill(turtle_name_);
    TurtleController::callClearBackground();

    spawn_client_.reset();
    kill_client_.reset();
    clear_bg_client_.reset();
    move_turtle_server_.reset();
    turtle_pose_sub_.reset();
    turtle_cmd_pub_.reset();

    return LifecycleCallbackReturn::SUCCESS;
}


// Turtle Spawn, Kill and ClearBG
void TurtleController::callTurtleSpawn(const double x, const double y, 
    const double theta, const std::string name)
{
    while(!spawn_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for SPAWN server...");
    }

    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;
    request->name = name;

    spawn_client_->async_send_request(request, 
        std::bind(&TurtleController::callbackCallTurtleSpawn, this, _1));        
}

void TurtleController::callTurtleKill(const std::string name)
{
    while(!kill_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for KILL server...");
    }

    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = name;

    kill_client_->async_send_request(request);
}

void TurtleController::callClearBackground()
{
    while (!clear_bg_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for CLEAR server...");
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    clear_bg_client_->async_send_request(request);
}

void TurtleController::callbackCallTurtleSpawn(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
{
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "'%s' spawned.", response->name.c_str());
}


rclcpp_action::GoalResponse TurtleController::goal_callback(
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

rclcpp_action::CancelResponse TurtleController::cancel_callback(
    const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received Cancel Request.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtleController::handle_accepted_callback(
    const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing the goal.");
    execute_goal(goal_handle);
}

void TurtleController::execute_goal(const std::shared_ptr<MoveTurtleGoalHandle> goal_handle)
{
    {
        // Not to run both mutex simultaneously
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }
    
    // Get request from goal
    geometry_msgs::msg::Twist turtle_cmd_vel;
    turtle_cmd_vel.linear.x = goal_handle->get_goal()->cmd_lin_x;
    turtle_cmd_vel.angular.z = goal_handle->get_goal()->cmd_ang_z;
    auto duration  = rclcpp::Duration::from_seconds(goal_handle->get_goal()->duration);

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
            turtle_cmd_pub_->publish(geometry_msgs::msg::Twist()); // Stop the turtle
            result->position = {turtle_pose_.x, turtle_pose_.y};
            result->message = "Turtle Spawn/Kill Server deactivated";
            goal_handle->abort(result);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                turtle_cmd_pub_->publish(geometry_msgs::msg::Twist()); // Stop the turtle
                result->position = {turtle_pose_.x, turtle_pose_.y};
                result->message = "Goal Preempted";
                goal_handle->abort(result);
                return;
            }
        }

        if (goal_handle->is_canceling()) {
            turtle_cmd_pub_->publish(geometry_msgs::msg::Twist()); // Stop the turtle
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

void TurtleController::callback_turtlepose(const turtlesim::msg::Pose pose) 
{
    turtle_pose_ = pose;
}

} // namespace final_proj3_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(final_proj3_cpp::TurtleController)