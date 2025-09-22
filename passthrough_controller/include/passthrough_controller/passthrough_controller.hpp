#ifndef PASSTHROUGH_CONTROLLER__PASSTHROUGH_CONTROLLER_HPP_
#define PASSTHROUGH_CONTROLLER__PASSTHROUGH_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace passthrough_controller
{
class PassthroughController : public controller_interface::ControllerInterface
{
public:
  PassthroughController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // Parameters
  std::vector<std::string> joint_names_;
  std::string real_robot_joint_states_topic_;
  
  // Subscriber for real robot joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  
  // Realtime buffer to store the latest joint state message
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>> rt_joint_state_buffer_;
  
  // Joint state callback
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  // Helper function to find joint index in received message
  int find_joint_index(const std::string& joint_name, const sensor_msgs::msg::JointState& msg);
};

}  // namespace passthrough_controller

#endif  // PASSTHROUGH_CONTROLLER__PASSTHROUGH_CONTROLLER_HPP_