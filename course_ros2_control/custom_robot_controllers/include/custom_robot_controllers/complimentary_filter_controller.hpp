#ifndef COMPLIMENTARY_FILTER_HPP
#define COMPLIMENTARY_FILTER_HPP

#include "controller_interface/controller_interface.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

using FloatArray = example_interfaces::msg::Float64MultiArray;

namespace complimentary_filter_controller {
class ComplimentaryFilterController: public controller_interface::ControllerInterface
{
public:
    ComplimentaryFilterController();

    // These 2 methods are mandatory to be implemented
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure
        (const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate
        (const rclcpp_lifecycle::State & previous_state) override;
    
    // Loop is read (hardware interface), update (controller: here), write (again hardware interface)
    controller_interface::return_type update
        (const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
    // What here is more application dependent
    std::vector<std::string> joint_names_;
    std::string interface_name_; // position or velocity
    double coefficient_;

    std::vector<double> appCommand_;
    rclcpp::Subscription<FloatArray>::SharedPtr command_subscriber_;

}; // ComplimentaryFilterController

} // namespace complimentary_filter_controller

#endif