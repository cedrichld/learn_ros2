#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilServerNode : public rclcpp::Node
{
public:
    CountUntilServerNode() : Node("node_name")
    {
        goal_queue_thread_ = std::thread(&CountUntilServerNode::run_goal_queue_thread, this);
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        count_until_server_ = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServerNode::goal_callback, this, _1, _2),
            std::bind(&CountUntilServerNode::cancel_callback, this, _1),
            std::bind(&CountUntilServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(), // we dont really care about this - just need to add cb_group_
            cb_group_
        );
        RCLCPP_INFO(this->get_logger(), "Action Server has been started.");
    }

    // When we destroy the node we ensure the goal_queue_thread_ is finished before we exit (to avoid any memory leak)
    ~CountUntilServerNode()
    {
        goal_queue_thread_.join();
    }

private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a goal.");

        if (goal->target_number <= 0.0) {
            RCLCPP_INFO(this->get_logger(), "Rejecting the goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received Cancel Request.");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(
        const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_queue_.push(goal_handle);
        RCLCPP_INFO(this->get_logger(), "Add goal to the queue.");
        RCLCPP_INFO(this->get_logger(), "Queue size: %d", (int)goal_queue_.size());
    }

    void run_goal_queue_thread()
    {
        rclcpp::Rate loop_rate(1000.0); // Need to establish rate, if not uses entire CPU to run...
        while (rclcpp::ok()) {
            std::shared_ptr<CountUntilGoalHandle> next_goal;
            {
                if(goal_queue_.size() > 0) {
                    next_goal = goal_queue_.front();
                    goal_queue_.pop();
                }
            }

            if (next_goal) {
                RCLCPP_INFO(this->get_logger(), "Execute next goal in the queue.");
                execute_goal(next_goal);
            }
            loop_rate.sleep();
        }
    }

    // Optional but best practice
    void execute_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        // Get request from goal
        int target_number = goal_handle->get_goal()->target_number;
        double period = goal_handle->get_goal()->period;

        // Execute the atcion
        int counter = 0;
        auto result = std::make_shared<CountUntil::Result>();
        auto feedback = std::make_shared<CountUntil::Feedback>();
        rclcpp::Rate loop_rate(1.0 / period);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        for (int i = 0; i < target_number; i++) {            
            if (goal_handle->is_canceling()) {
                result->result_number = counter;
                goal_handle->canceled(result);
                return;
            }
            counter++;
            RCLCPP_INFO(this->get_logger(), "%d", counter);
            feedback->current_number = counter;
            goal_handle->publish_feedback(feedback);

            // More precise than sleeping in the forloop
            loop_rate.sleep();
        }

        // We dont return anything in C++ (in handle and execute), instead
        // we pass the result when set set the final state (different from Python)
        result->result_number = counter;
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::mutex mutex_;
    std::queue<std::shared_ptr<CountUntilGoalHandle>> goal_queue_;
    std::thread goal_queue_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
