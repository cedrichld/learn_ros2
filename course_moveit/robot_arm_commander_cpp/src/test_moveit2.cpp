// usage: 
// ros2 run robot_arm_commander_cpp test_moveit2 --ros-args --params-file ~/ros2_ws/learn_ros2/src/course_moveit/robot_arm_commander_cpp/config/kinematics_params.yaml 

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // * * * Pose Goals (IK) * * * //
    // ******************
    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    // Set the goal
    moveit::planning_interface::MoveGroupInterface::Plan plan7;
    // Plan
    bool success7 = (arm.plan(plan7) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute
    if (success7) {
        arm.execute(plan7);
    }

    // Cartesian Path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z -= 0.2;
    waypoints.push_back(pose1);
    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.x += 0.2;
    waypoints.push_back(pose2);
    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.z += 0.2;
    waypoints.push_back(pose3);
    geometry_msgs::msg::Pose pose4 = pose3;
    pose4.position.x -= 0.2;
    waypoints.push_back(pose4);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction == 1) {
        arm.execute(trajectory);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path planning failed with only %.2f%% success.", fraction * 100);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}