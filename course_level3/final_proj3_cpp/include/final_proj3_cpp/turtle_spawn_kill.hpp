#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp" 
#include "example_interfaces/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using LifecycleCallbackReturn = 
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace final_proj3_cpp {

class TurtleSpawnKillClient : public rclcpp_lifecycle::LifecycleNode
{
public:
    TurtleSpawnKillClient(const rclcpp::NodeOptions &options);

    void callTurtleSpawn(const double x, const double y, const double theta, const std::string name);
    void callTurtleKill(const std::string name);
    void callClearBackground();

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

private:
    void callbackCallTurtleSpawn(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future);

    std::string turtle_name_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_bg_client_;
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr lifecycle_state_pub_;
};

}  // namespace final_proj3_cpp