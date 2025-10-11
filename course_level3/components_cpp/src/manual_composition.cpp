#include "rclcpp/rclcpp.hpp"
#include "components_cpp/node1.hpp"
#include "components_cpp/node2.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<Node1>();
    auto node2 = std::make_shared<Node2>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
