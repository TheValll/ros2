#include "ros2_control_controller_template/basic_controller.hpp"

namespace BasicController{
  BasicController::BasicController(): controller_interface::ControllerInterface(){}

  controller_interface::CallbackReturn BasicController::on_init()
  {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
    interface_name_ = auto_declare<std::string>("interface_name", "position");
    coefficient_ = auto_declare<double>("coefficient", 0.8);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn BasicController::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;

    auto callback = [this] (const FloatArray::SharedPtr msg) -> void
    {
      if (msg->data.size() == joint_names_.size()){
        appCommand_.clear();
        for(auto cmd: msg->data){
          appCommand_.push_back(cmd);
        }
      }
    };

    command_subscriber_ = get_node()->create_subscription<FloatArray>("joints_command", 10, callback);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration BasicController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size());
    for(auto joint_name: joint_names_){
      config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration BasicController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size());
    for(auto joint_name: joint_names_){
      config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;
  }

  controller_interface::CallbackReturn BasicController::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    appCommand_.clear();
    for(int i = 0; i < (int)joint_names_.size(); i++){
      appCommand_.push_back(state_interfaces_[i].get_optional().value());
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type BasicController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    (void)time;
    (void)period;

    for(int i = 0; i < (int)joint_names_.size(); i++){
      double state = state_interfaces_[i].get_optional().value();
      double cmd = appCommand_[i];
      double new_cmd = cmd * coefficient_ + state * (1 - coefficient_);
      (void)command_interfaces_[i].set_value(new_cmd);
    }
    return controller_interface::return_type::OK;
  }
} // namespace BasicController

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(BasicController::BasicController, controller_interface::ControllerInterface)
