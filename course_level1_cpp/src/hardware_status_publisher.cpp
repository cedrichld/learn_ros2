#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"
    
using namespace std::chrono_literals;

class HardwareStatusNode : public rclcpp::Node
{
public:
    HardwareStatusNode() : Node("hardware_status_publisher")
    {
        publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>(
            "hardware_status", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&HardwareStatusNode::publishHardwareStatus, this));
    }
    
private:
    void publishHardwareStatus()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.are_motors_ready = true;
        msg.debug_message = "Motors good";
        msg.temperature = 34.2;
        publisher_->publish(msg);
    }
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
    // rclcpp::Subscription<my_robot_interfaces::msg::HardwareStatus>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}