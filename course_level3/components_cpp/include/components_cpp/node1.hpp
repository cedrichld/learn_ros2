#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class Node1 : public rclcpp::Node
{
public:
    Node1();

private:
    void callbackTimer1();
    void callbackTimer2();
    void callbackTimer3();

    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::TimerBase::SharedPtr timer3_;
};