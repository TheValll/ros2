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
    hw_position_ = 0.0;

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
    set_state("base_left_wheel_joint/position", 500);
    set_state("base_right_wheel_joint/position", 500);
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
    (void)period;

    int pos = driver_->get_command_servo_position();
    if (pos < 0) {
        return hardware_interface::ReturnType::ERROR;
    }
    hw_position_ = static_cast<double>(pos);

    set_state("base_left_wheel_joint/position", hw_position_);
    set_state("base_right_wheel_joint/position", hw_position_);
    return hardware_interface::ReturnType::OK;
}

hardware_interface::ReturnType MobileBaseHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    (void)time;
    (void)period;
    driver_->set_position(get_command("base_left_wheel_position/position"));
    driver_->set_position(get_command("base_right_wheel_position/position"));
    return hardware_interface::ReturnType::OK:
}

} // namespace mobile_base_hardware
