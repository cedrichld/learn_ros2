#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class NumberNode : public rclcpp::Node
{
public:
    NumberNode() : Node("number_publisher")
    {
        this->declare_parameter("number", 2);
        this->declare_parameter("timer_period", 1.0);
        number_ = this->get_parameter("number").as_int();
        double timer_period_ = this->get_parameter("timer_period").as_double();

        param_callback_handle_ = this->add_post_set_parameters_callback(
            std::bind(&NumberNode::parametersCallback, this, _1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period_), 
            std::bind(&NumberNode::publish_number, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started.");

    }
private:
    void publish_number()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        publisher_->publish(msg);
    }

    // Used if we want to change parameter during runtime 
    // Use with caution (for example for timer_period would 
    // have to do .cancel and then relaunch with right value)
    void parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
    {
        for (const auto &param: parameters) {
            if (param.get_name() == "number") {
                number_ = param.as_int();
            }
        }
    }
    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    PostSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}