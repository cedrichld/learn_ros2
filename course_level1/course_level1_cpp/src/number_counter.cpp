#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::placeholders;
    
class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), number_counter_(0)
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 10, 
            std::bind(&NumberCounterNode::callbackNumberSub, this, _1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>(
            "number_count", 10);
        service_ = this->create_service<example_interfaces::srv::SetBool>(
                "reset_counter", std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));
    }
     
private:
    void callbackNumberSub(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        number_counter_ += msg->data;
        auto msg2 = example_interfaces::msg::Int64();
        msg2.data = number_counter_;
        publisher_->publish(msg2);
    }

    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request, 
                         const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data) {
            number_counter_ = 0;
            response->success = true;
            response->message = "Counter reset to 0";
        } else {
            response->success = false;
            response->message = "Counter not reset to 0";
        }
    }

    int number_counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}