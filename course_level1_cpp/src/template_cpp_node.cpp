#include "rclcpp/rclcpp.hpp"
    
class MyCustomNode : public rclcpp::Node
{
public:
    MyCustomNode() : Node("node_name")
    {
    }
    
private:
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}