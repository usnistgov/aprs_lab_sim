#include "passthrough_controller/passthrough_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace passthrough_controller
{

PassthroughController::PassthroughController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PassthroughController::on_init()
{
  try {
    // Declare parameters
    get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>());
    get_node()->declare_parameter<std::string>("real_robot_joint_states_topic", "/joint_states");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PassthroughController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  real_robot_joint_states_topic_ = get_node()->get_parameter("real_robot_joint_states_topic").as_string();

  if (joint_names_.empty()) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "No joints specified in 'joints' parameter. Got topic: " << real_robot_joint_states_topic_);
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(), "Configuring passthrough controller for %zu joints",
    joint_names_.size());

  // Create subscriber for real robot joint states
  joint_state_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    real_robot_joint_states_topic_, 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      joint_state_callback(msg);
    });

  RCLCPP_INFO(
    get_node()->get_logger(), "Subscribed to real robot joint states on topic: %s",
    real_robot_joint_states_topic_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PassthroughController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Check that we have the correct number of command interfaces
  if (command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Number of command interfaces (%zu) != number of joints (%zu)",
      command_interfaces_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Passthrough controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PassthroughController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Passthrough controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PassthroughController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request position command interfaces for all joints
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  return config;
}

controller_interface::InterfaceConfiguration
PassthroughController::state_interface_configuration() const
{
  // We don't need any state interfaces from the simulated robot
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::return_type PassthroughController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Get the latest joint state message
  auto joint_state_msg = rt_joint_state_buffer_.readFromRT();
  
  if (!joint_state_msg || !(*joint_state_msg)) {
    // No message received yet, keep current positions
    return controller_interface::return_type::OK;
  }

  const auto & msg = **joint_state_msg;

  // Apply joint positions from real robot to simulated robot
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    int joint_index = find_joint_index(joint_names_[i], msg);

    int mult = 1;

    if(joint_names_[i] == "joint_l" || joint_names_[i] == "joint_t" || joint_names_[i] == "joint_r"){
      mult = -1;
    }
    
    if (joint_index >= 0 && joint_index < static_cast<int>(msg.position.size())) {
      if(!command_interfaces_[i].set_value(msg.position[joint_index] * mult)){
        throw std::runtime_error("Unable to set joint");
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Joint '%s' not found in received joint state message", joint_names_[i].c_str());
    }
  }

  return controller_interface::return_type::OK;
}

void PassthroughController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  rt_joint_state_buffer_.writeFromNonRT(msg);
}

int PassthroughController::find_joint_index(const std::string& joint_name, const sensor_msgs::msg::JointState& msg)
{
  auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
  if (it != msg.name.end()) {
    return std::distance(msg.name.begin(), it);
  }
  return -1;
}

}  // namespace passthrough_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  passthrough_controller::PassthroughController, controller_interface::ControllerInterface)