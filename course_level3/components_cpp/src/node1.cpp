#include "components_cpp/node1.hpp"

Node1::Node1() : Node("node1")
    {
        this->timer1_ = this->create_wall_timer(1000ms, std::bind(&Node1::callbackTimer1, this));
        this->timer2_ = this->create_wall_timer(1000ms, std::bind(&Node1::callbackTimer2, this));
        this->timer3_ = this->create_wall_timer(1000ms, std::bind(&Node1::callbackTimer3, this));
    }

void Node1::callbackTimer1()
{
    std::this_thread::sleep_for(2000ms);
    RCLCPP_INFO(this->get_logger(), "cb 1");
}

void Node1::callbackTimer2()
{
    std::this_thread::sleep_for(2000ms);
    RCLCPP_INFO(this->get_logger(), "cb 2");
}

void Node1::callbackTimer3()
{
    std::this_thread::sleep_for(2000ms);
    RCLCPP_INFO(this->get_logger(), "cb 3");
}