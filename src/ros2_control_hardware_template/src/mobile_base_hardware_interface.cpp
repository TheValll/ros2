#include "ros2_control_hardware_template/mobile_base_hardware_interface.hpp"

namespace mobile_base_hardware {

hardware_interface::CallbackReturn MobileBaseHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    servo_id_   = 6;
    baudrate_   = 115200;
    port_name_  = "/dev/ttyUSB0";

    driver_ = std::make_shared<LX225Driver>(port_name_, baudrate_, servo_id_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardware::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    if (driver_->init() != 0)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    set_state("base_left_wheel_joint/position", 0.0);
    set_state("base_left_wheel_joint/velocity", 0.0);
    set_state("base_right_wheel_joint/position", 0.0);
    set_state("base_right_wheel_joint/velocity", 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    driver_->close_LX225();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::ReturnType MobileBaseHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
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
    return hardware_interface::ReturnType::OK;
}

hardware_interface::ReturnType MobileBaseHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    hw_commands_[0] = get_command("base_left_wheel_joint/velocity");
    hw_commands_[1] = get_command("base_right_wheel_joint/velocity");
    return hardware_interface::ReturnType::OK;
}

} // namespace mobile_base_hardware
