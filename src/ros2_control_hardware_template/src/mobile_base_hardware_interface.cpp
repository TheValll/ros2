#include "ros2_control_hardware_template/mobile_base_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace mobile_base_hardware {

hardware_interface::CallbackReturn MobileBaseHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        servo_id_  = std::stoi(info_.hardware_parameters.at("servo_id"));
        baudrate_  = std::stoi(info_.hardware_parameters.at("baudrate"));
        port_name_ = info_.hardware_parameters.at("port");
    } catch (...) {
        servo_id_  = 6;
        baudrate_  = 115200;
        port_name_ = "/dev/ttyUSB0";

        RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardware"),
            "Using default parameters (servo_id=6, baudrate=115200, port=/dev/ttyUSB0)");
    }

    for (size_t i = 0; i < 2; i++) {
        hw_positions_[i]  = 0.0;
        hw_velocities_[i] = 0.0;
        hw_commands_[i]   = 0.0;
    }

    driver_ = std::make_shared<LX225Driver>(port_name_, baudrate_, servo_id_);

    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardware"),
        "Initialized with port=%s baudrate=%d servo_id=%d",
        port_name_.c_str(), baudrate_, servo_id_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardware::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() != 0)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardware::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    hw_positions_[0] = hw_positions_[1] = 0.0;
    hw_velocities_[0] = hw_velocities_[1] = 0.0;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardware::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_->close_LX225();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileBaseHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;

    for (int i = 0; i < 2; i++) {
        hw_positions_[i] += hw_commands_[i] * period.seconds(); 
        hw_velocities_[i] = hw_commands_[i];
    }

    set_state("base_left_wheel_joint/position", hw_positions_[0]);
    set_state("base_left_wheel_joint/velocity", hw_velocities_[0]);
    set_state("base_right_wheel_joint/position", hw_positions_[1]);
    set_state("base_right_wheel_joint/velocity", hw_velocities_[1]);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileBaseHardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    double cmd_left  = get_command("base_left_wheel_joint/velocity");
    double cmd_right = get_command("base_right_wheel_joint/velocity");

    hw_commands_[0] = std::isnan(cmd_left)  ? 0.0 : cmd_left;
    hw_commands_[1] = std::isnan(cmd_right) ? 0.0 : cmd_right;

    return hardware_interface::return_type::OK;
}

} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardware, hardware_interface::SystemInterface)
