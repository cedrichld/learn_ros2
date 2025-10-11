#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/target_array.hpp"
#include "my_robot_interfaces/msg/target.hpp"
#include "my_robot_interfaces/srv/kill_turtle.hpp"
#include "turtlesim/msg/pose.hpp" 
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

#include <random>
#include <algorithm> 

using namespace std::chrono_literals;
using namespace std::placeholders;
    
class TurtleManagerNode : public rclcpp::Node
{
public:
    TurtleManagerNode() : Node("turtle_manager"), spawn_period(0.3), rng_(std::random_device{}())
    {
        targets_ = this->create_publisher<my_robot_interfaces::msg::TargetArray>("/targets", 1);
        kill_turtle_ = this->create_service<my_robot_interfaces::srv::KillTurtle>(
            "/kill_turtle", std::bind(&TurtleManagerNode::callbackKillTurtle, this, _1, _2));
        kill_service_client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        spawn_service_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        spawn_timer_ = this->create_timer(std::chrono::duration<double>(spawn_period), 
            std::bind(&TurtleManagerNode::randomSpawn, this));
    }

    void callKillService(std::string turtle_name)
    {
        while (!kill_service_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle_name;

        /* creating a callback didn't work because async calls the callback with one argument 
        and i couldnt find a way to send over the turtle_name (request) */
        // kill_service_client_->async_send_request(request, 
        //     std::bind(&TurtleManagerNode::callbackCallKillService, this, _1, turtle_name));

        kill_service_client_->async_send_request(
            request,
            [this, name = turtle_name](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future) {
                (void)future.get();

                auto &v = current_targets.targets;
                v.erase(std::remove_if(v.begin(), v.end(),
                                    [&](const auto &t){ return t.name == name; }),
                        v.end());

                targets_->publish(current_targets);
                RCLCPP_INFO(this->get_logger(), "Turtle killed: %s", name.c_str());
            });
    }
    
private:
    void callbackKillTurtle(const my_robot_interfaces::srv::KillTurtle::Request::SharedPtr request, 
                            const my_robot_interfaces::srv::KillTurtle::Response::SharedPtr response)
    {
        TurtleManagerNode::callKillService(request->turtle_name);
        response->success = true;
    } 

    /* creating a callback didn't work because async calls the callback with one argument 
        and i couldnt find a way to send over the turtle_name (request) */
    // void callbackCallKillService(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future, const std::string turtle_name)
    // {
    //     auto response = future.get();
    //     auto &v = current_targets.targets;
    //     v.erase(std::remove_if(v.begin(), v.end(),
    //                             [&](const auto &t){ return t.name == turtle_name; }),
    //             v.end());
    //     targets_->publish(current_targets);
    //     RCLCPP_INFO(this->get_logger(), "Turtle killed: %s", turtle_name.c_str());
    // }

    void randomSpawn() 
    {
        my_robot_interfaces::msg::Target target;
        target.x = dist_xy_(rng_);
        target.y = dist_xy_(rng_);
        target.theta = dist_theta_(rng_);
        target.name = "turtle" + std::to_string(next_id_++);

        auto target_ptr = std::make_shared<my_robot_interfaces::msg::Target>(target);
        callSpawnService(target_ptr);

        // my_robot_interfaces::msg::TargetArray arr;
        current_targets.targets.reserve(1);
        current_targets.targets.push_back(target);
        targets_->publish(current_targets);
    }

    void callSpawnService(my_robot_interfaces::msg::Target::ConstSharedPtr turtle) // should be passing in a my_robot_interfaces/msg/target
    {
        // TODO: randomized positions
        while (!spawn_service_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = turtle->x;
        request->y = turtle->y;
        request->theta = turtle->theta;
        request->name = turtle->name;

        spawn_service_client_->async_send_request(request, 
            std::bind(&TurtleManagerNode::callbackCallSpawnService, this, _1));
    }

    void callbackCallSpawnService(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        auto response = future.get();
        // RCLCPP_INFO(this->get_logger(), "Turtle spawned at x: %s, y: %s, z: %s");
    }

    rclcpp::Publisher<my_robot_interfaces::msg::TargetArray>::SharedPtr targets_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_service_client_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_service_client_;
    rclcpp::Service<my_robot_interfaces::srv::KillTurtle>::SharedPtr kill_turtle_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    double spawn_period;
    my_robot_interfaces::msg::TargetArray current_targets;

    // From internet
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_xy_{1.0, 10.0}; // inside turtlesim window
    std::uniform_real_distribution<double> dist_theta_{-M_PI, M_PI};
    int next_id_{2};
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}