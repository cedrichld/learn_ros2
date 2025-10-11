#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

namespace my_namespace {

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher(const rclcpp::NodeOptions &options);

private:
    void publishNumber();

    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    rclcpp::TimerBase::SharedPtr number_timer_;
};

} // namespace my_namespace