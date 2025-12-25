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

    // * * * Named Goals (IK) * * * //
    // ******************
    // 1 ARM
    // Set starting state
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("pose_1");

    // Set the goal
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // Plan
    bool success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute
    if (success) {
        arm.execute(plan1);
    }

    // ******************
    // 2 GRIPPER
    // Set starting state
    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("gripper_closed");

    // Set the goal
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // Plan
    bool success2 = (gripper.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute
    if (success2) {
        gripper.execute(plan2);
    }

    // ******************
    // 3 ARM
    // Set starting state
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("pose_2");

    // Set the goal
    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    // Plan
    bool success3 = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute
    if (success3) {
        arm.execute(plan3);
    }

    // ******************
    // 4 GRIPPER
    // Set starting state
    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("gripper_open");

    // Set the goal
    moveit::planning_interface::MoveGroupInterface::Plan plan4;
    // Plan
    bool success4 = (gripper.plan(plan4) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute
    if (success4) {
        gripper.execute(plan4);
    }

    // ******************
    // 5 ARM
    // Set starting state
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");

    // Set the goal
    moveit::planning_interface::MoveGroupInterface::Plan plan5;
    // Plan
    bool success5 = (arm.plan(plan5) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute
    if (success5) {
        arm.execute(plan5);
    }

    // ------------------------------------------------------------------------------------

    // * * * Joint Goals (FK) * * * //
    // ******************
    std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };
    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan6;
    // Plan
    bool success6 = (arm.plan(plan6) == moveit::core::MoveItErrorCode::SUCCESS);

    // Execute
    if (success6) {
        arm.execute(plan6);
    }

    // ------------------------------------------------------------------------------------

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

    rclcpp::shutdown();
    spinner.join();
    return 0;
}