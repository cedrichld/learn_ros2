#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include "robot_arm_interfaces/msg/go_to_pose_target.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;

// this class is not a node but contains the node
class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
        open_gripper_sub_ = node_->create_subscription<example_interfaces::msg::Bool>( // and not this->create_subscription
            "open_gripper", 10, std::bind(&Commander::callbackOpenGripper, this, _1));
        joint_target_sub_ = node_->create_subscription<example_interfaces::msg::Float64MultiArray>(
            "joint_target", 10, std::bind(&Commander::callbackJointTarget, this, _1));
        pose_target_sub_ = node_->create_subscription<robot_arm_interfaces::msg::GoToPoseTarget>(
            "pose_target", 10, std::bind(&Commander::callbackPoseTarget, this, _1));
    }

    // * * * Named Goals (IK) * * * //
    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    // * * * Joint Goals (FK) * * * //
    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    // * * * Pose Goals (IK) * * * //
    void goToPoseTarget(double x, double y, double z, 
        double roll, double pitch, double yaw, bool cartestian_path=false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if (!cartestian_path) {
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        } else {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);

            if (fraction == 1) {
                arm_->execute(trajectory);
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path planning failed with only %.2f%% success.", fraction * 100.0);
            }
        }
    }

    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }

    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");
        planAndExecute(gripper_);
    }


private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        // Plan
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // Execute
        if (success) {
            interface->execute(plan);
        }
    }

    void callbackOpenGripper(const example_interfaces::msg::Bool &msg)
    {
        if (msg.data) {
            openGripper();
        } else {
            closeGripper();
        }
    }

    void callbackJointTarget(const example_interfaces::msg::Float64MultiArray &msg)
    {
        auto joints = msg.data;

        if (joints.size() == 6) {
            goToJointTarget(joints);
        }
    }

    void callbackPoseTarget(const robot_arm_interfaces::msg::GoToPoseTarget &msg)
    {
        goToPoseTarget(msg.x, msg.y, msg.z, 
            msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr joint_target_sub_;
    rclcpp::Subscription<robot_arm_interfaces::msg::GoToPoseTarget>::SharedPtr pose_target_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}