#include "final_proj3_cpp/turtle_spawn_kill.hpp"

namespace final_proj3_cpp {

TurtleSpawnKillClient::TurtleSpawnKillClient(const rclcpp::NodeOptions &options) : LifecycleNode("turtle_spawn_kill_client", options)
{
    lifecycle_state_pub_ = this->create_publisher<example_interfaces::msg::Bool>("lifecycle_state", 1);
    this->declare_parameter("turtle_name", "controlled_turtle");
}

void TurtleSpawnKillClient::callTurtleSpawn(const double x, const double y, 
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
        std::bind(&TurtleSpawnKillClient::callbackCallTurtleSpawn, this, _1));        
}

void TurtleSpawnKillClient::callTurtleKill(const std::string name)
{
    while(!kill_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for KILL server...");
    }

    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = name;

    kill_client_->async_send_request(request);
}

void TurtleSpawnKillClient::callClearBackground()
{
    while (!clear_bg_client_->wait_for_service(1s)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for CLEAR server...");
    }

    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    clear_bg_client_->async_send_request(request);
}

LifecycleCallbackReturn TurtleSpawnKillClient::on_configure(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    turtle_name_ = this->get_parameter("turtle_name").as_string();

    spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
    kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
    clear_bg_client_ = this->create_client<std_srvs::srv::Empty>("/clear");
    
    callTurtleKill("turtle1");
    callClearBackground();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    callTurtleSpawn(5.0, 5.0, 0.0, turtle_name_);

    example_interfaces::msg::Bool bool_;
    lifecycle_state_pub_->publish(bool_);

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleSpawnKillClient::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "IN on_activate");
    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    
    example_interfaces::msg::Bool bool_;
    bool_.data = true;
    lifecycle_state_pub_->publish(bool_);

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleSpawnKillClient::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "IN on_deactivate");

    example_interfaces::msg::Bool bool_;
    lifecycle_state_pub_->publish(bool_);

    // deactivate action server
    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleSpawnKillClient::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "IN on_cleanup");

    example_interfaces::msg::Bool bool_;
    lifecycle_state_pub_->publish(bool_);
    
    TurtleSpawnKillClient::callTurtleKill(turtle_name_);
    TurtleSpawnKillClient::callClearBackground();

    spawn_client_.reset();
    kill_client_.reset();
    clear_bg_client_.reset();

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn TurtleSpawnKillClient::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    (void)previous_state;

    example_interfaces::msg::Bool bool_;
    lifecycle_state_pub_->publish(bool_);

    spawn_client_.reset();
    kill_client_.reset();
    clear_bg_client_.reset();

    RCLCPP_INFO(this->get_logger(), "IN on_shutdown");
    return LifecycleCallbackReturn::SUCCESS;
}


void TurtleSpawnKillClient::callbackCallTurtleSpawn(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
{
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "'%s' spawned.", response->name.c_str());
}

}  // namespace final_proj3_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(final_proj3_cpp::TurtleSpawnKillClient)

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<TurtleSpawnKillClientNode>();
//     rclcpp::spin(node->get_node_base_interface());
//     rclcpp::shutdown();
//     return 0;
// }