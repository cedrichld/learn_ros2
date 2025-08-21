#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilServerNode : public rclcpp::Node
{
public:
    CountUntilServerNode() : Node("count_until_server_node")
    {
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

private:

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a goal.");

        // Policy: refuse new goal if one goal is already running
        // {
        //     // Cannot run both mutex at the same time
        //     std::lock_guard<std::mutex> lock(mutex_);

        //     if (goal_handle_) {
        //         if (goal_handle_->is_active()) {
        //             RCLCPP_INFO(this->get_logger(), "A goal is still active, rejecting new goal.");
        //             return rclcpp_action::GoalResponse::REJECT;
        //         }
        //     }
        // }

        if (goal->target_number <= 0.0) {
            RCLCPP_INFO(this->get_logger(), "Rejecting the goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Policy: Preempt exisiting goal when receiving a new goal
        {
            // Cannot run both mutex at the same time
            std::lock_guard<std::mutex> lock(mutex_);

            if (goal_handle_) {
                if (goal_handle_->is_active()) {
                    RCLCPP_INFO(this->get_logger(), "Abort current goal and accept new goal.");
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                }
            }
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
        RCLCPP_INFO(this->get_logger(), "Executing the goal.");
        execute_goal(goal_handle);
    }

    // Optional but best practice
    void execute_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        {
            // Cannot run both mutex at the same time
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }
        
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
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->result_number = counter;
                    goal_handle->abort(result);
                    return;
                }
            }
            
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
    std::shared_ptr<CountUntilGoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;
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
