#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "base_robot_hardware/xl330_driver.hpp"

namespace mobile_base_hardware {
    
class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Lifecycle node override
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    // SystemInterface override
    hardware_interface::CallbackReturn 
        on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    hardware_interface::return_type
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<XL330Driver> driver_;
    int left_motor_id_;
    int right_motor_id_;
    std::string port_;
    
}; // class MobileBaseHardwareInterface

} // namespace mobile_base_hardware


#endif