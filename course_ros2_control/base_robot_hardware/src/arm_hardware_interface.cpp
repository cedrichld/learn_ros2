#include "base_robot_hardware/arm_hardware_interface.hpp"

namespace two_link_arm_hardware {

hardware_interface::CallbackReturn TwoLinkArmHardwareInterface::on_init
    (const hardware_interface::HardwareComponentInterfaceParams & params)
{
    // Here function to check hardware_interfaces load properly + 
    // set actuator IDs + driver module at given port
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto & info = params.hardware_info;
    info_ = info; // Private attribute from System Interface

    first_motor_id_ = std::stoi(info_.hardware_parameters["first_motor_id"]);
    second_motor_id_ = std::stoi(info_.hardware_parameters["second_motor_id"]);
    port_ = info_.hardware_parameters["port"]; 

    driver_ = std::make_shared<XL330Driver>(port_);

    /* As a best practice also important to check that all hardware interfaces are
    working - what is defined in the urdf folder: *.ros2_control.xacro, command_interfaces */
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwoLinkArmHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    // Here, function to check if we can activate the hardware.
    // In this case, check the driver was set up properly: 
    // Managed to connect to motors + open port and set baudrate
    (void)previous_state;
    if (driver_->init() != 0) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // Check logs make sense for what we control / receive state feedback from
    for (const auto & [name, descr] : joint_command_interfaces_)
    {
        RCLCPP_INFO(get_logger(), "COMMAND INTERFACE NAME: ");
        RCLCPP_INFO(get_logger(), name.c_str());
    }
    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        RCLCPP_INFO(get_logger(), "STATE INTERFACE NAME: ");
        RCLCPP_INFO(get_logger(), name.c_str());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TwoLinkArmHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    // Driver is well set up and connected to actuators so we can activate them
    (void)previous_state;
    // might have to get_pos from actual motor based on robot, here:
    double first_pose_ = driver_->getPositionRadian(first_motor_id_);
    double second_pose_ = driver_->getPositionRadian(second_motor_id_);
    set_state("arm_base_forearm_joint/position", first_pose_);
    set_state("forearm_hand_joint/position", second_pose_);
    driver_->activateWithPositionMode(first_motor_id_);
    driver_->activateWithPositionMode(second_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn TwoLinkArmHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state) 
{
    (void)previous_state;
    driver_->deactivate(first_motor_id_);
    driver_->deactivate(second_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TwoLinkArmHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    double first_pose_ = driver_->getPositionRadian(first_motor_id_);
    double second_pose_ = driver_->getPositionRadian(second_motor_id_);
    set_state("arm_base_forearm_joint/position", first_pose_);
    set_state("forearm_hand_joint/position", second_pose_);
    
    // Constantly output first/second vel/pos
    // RCLCPP_INFO(get_logger(), "first vel: %lf, second vel: %lf, first pos: %lf, second pos: %lf",
    //          first_vel, second_vel, get_state("arm_base_forearm_joint/position"), get_state("forearm_hand_joint/position"));

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type TwoLinkArmHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    // Send to driver a Target Velocity   | motor id #   | from ros2 cmd sent to joint #     | vel/pos)
    driver_->setTargetPositionRadian(first_motor_id_, get_command("arm_base_forearm_joint/position"));
    driver_->setTargetPositionRadian(second_motor_id_, get_command("forearm_hand_joint/position"));
    
    // Constantly output what we are commanding from ros2
    // RCLCPP_INFO(get_logger(), "first vel: %lf, second vel: %lf", get_command("arm_base_forearm_joint/velocity"), 
    //                     get_command("forearm_hand_joint/velocity"));

    return hardware_interface::return_type::OK;
}

} // namespace two_link_arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(two_link_arm_hardware::TwoLinkArmHardwareInterface, hardware_interface::SystemInterface)