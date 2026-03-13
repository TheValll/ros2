#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "ros2_control_hardware_template/LX225Driver.hpp"
#include <string>
#include <memory>

namespace mobile_base_hardware {

class MobileBaseHardware : public hardware_interface::SystemInterface {
  public:
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::ReturnType read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::ReturnType write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
    std::shared_ptr<LX225Driver> driver_;
    int servo_id_;
    int baudrate_;
    std::string port_name_;
    double hw_positions_[2] = {0.0, 0.0};
    double hw_velocities_[2] = {0.0, 0.0};
    double hw_commands_[2] = {0.0, 0.0};
};

} // namespace mobile_base_hardware

#endif // MOBILE_BASE_HARDWARE_INTERFACE_HPP
