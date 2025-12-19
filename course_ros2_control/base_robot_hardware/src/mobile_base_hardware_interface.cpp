#include "base_robot_hardware/mobile_base_hardware_interface.hpp"

namespace mobile_base_hardware {

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init
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

    left_motor_id_ = 10;
    right_motor_id_ = 20;
    port_ = "dev/ttyACM0";

    driver_ = std::make_shared<XL330Driver>(port_);

    /* As a best practice also important to check that all hardware interfaces are
    working - what is defined in the urdf folder: *.ros2_control.xacro, command_interfaces */
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state)
{
    // Here, function to check if we can activate the hardware.
    // In this case, check the driver was set up properly: 
    // Managed to connect to motors + open port and set baudrate
    (void)previous_state;
    if (driver_->init() != 0) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state)
{
    // Driver is well set up and connected to actuators so we can activate them
    (void)previous_state;
    // can be important to set to the actual current state pos for example for robotic arm
    set_state("base_left_wheel_joint/velocity", 0.0); 
    set_state("base_right_wheel_joint/velocity", 0.0);
    // might have to get_pos from actual motor based on robot, here:
    // driver_->getPositionRadianPerSec(motor_id_);
    set_state("base_left_wheel_joint/position", 0.0);
    set_state("base_right_wheel_joint/position", 0.0);
    driver_->activateWithVelocityMode(left_motor_id_);
    driver_->activateWithVelocityMode(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state) 
{
    (void)previous_state;
    driver_->deactivate(left_motor_id_);
    driver_->deactivate(right_motor_id_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    double left_vel = driver_->getVelocityRadianPerSec(left_motor_id_);
    double right_vel = driver_->getVelocityRadianPerSec(right_motor_id_);
    set_state("base_left_wheel_joint/velocity", left_vel); // from the system interface class
    set_state("base_right_wheel_joint/velocity", right_vel);
    set_state("base_left_wheel_joint/position", 
        get_state("base_left_wheel_joint/position") + left_vel * period.seconds()); 
    //  ^^ could also get actual position from motor ofc
    set_state("base_right_wheel_joint/position", 
        get_state("base_right_wheel_joint/position") + right_vel * period.seconds());

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    // Send to driver a Target Velocity   | motor id #   | from ros2 cmd sent to joint #     | vel/pos)
    driver_->setTargetVelocityRadianPerSec(left_motor_id_, get_command("base_left_wheel_joint/velocity"));
    driver_->setTargetVelocityRadianPerSec(right_motor_id_, get_command("base_right_wheel_joint/velocity"));
    return hardware_interface::return_type::OK;
}

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)