#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
     
class ResetCounterClientNode : public rclcpp::Node
{
public:
    ResetCounterClientNode() : Node("reset_counter_client")
    {
        client_ = this->create_client<example_interfaces::srv::SetBool>("reset_counter");
    }
    void callResetCounter(bool data)
    {
        while (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        request->data = data;

        client_->async_send_request(request, std::bind(&ResetCounterClientNode::callbackCallResetCounter, this, _1));
    }
private:
    void callbackCallResetCounter(rclcpp::Client<example_interfaces::srv::SetBool>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ResetCounterClientNode>();
    node->callResetCounter(true);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}