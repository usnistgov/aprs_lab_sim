#ifndef APRS_PLUGINS__CONVEYOR_PLUGIN_HPP_
#define APRS_PLUGINS__CONVEYOR_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>

#include <rclcpp/rclcpp.hpp>

// MSGS
#include <conveyor_interfaces/msg/conveyor_state.hpp>

// SRVS
#include <conveyor_interfaces/srv/enable_conveyor.hpp>
#include <conveyor_interfaces/srv/set_conveyor_state.hpp>

#include <thread>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace aprs_plugins
{
  class ConveyorPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: ConveyorPlugin();
 
    public: ~ConveyorPlugin() override;

    void Configure (const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_manager) override;

    void PreUpdate(const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;

    // Conveyor callbacks
    // 
    void enable_conveyor_callback(const std::shared_ptr<conveyor_interfaces::srv::EnableConveyor::Request> request, std::shared_ptr<conveyor_interfaces::srv::EnableConveyor::Response> response);
    void set_conveyor_state_callback(const std::shared_ptr<conveyor_interfaces::srv::SetConveyorState::Request> request, std::shared_ptr<conveyor_interfaces::srv::SetConveyorState::Response> response);

    private: 
      gz::sim::Model _model;
      
      gz::sim::Joint _belt_joint;
      
      bool _enabled = false;
      
      int _belt_direction = 1;

      double _belt_velocity = 0.2;
      double _max_velocity = 0.4;
      double _power = 0.1;
      double _conveyor_limit = 0.4;
      double _belt_position = 0.0;

      std::vector<double>_reset_positions{0};

      rclcpp::Node::SharedPtr _ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
      std::thread thread_executor_spin_;

      rclcpp::Context::SharedPtr conveyor_context;

      // gz::transport::Node gz_node;
      // gz::transport::Node::Publisher gz_node_publisher;

    //   // Publishers
      rclcpp::Publisher<conveyor_interfaces::msg::ConveyorState>::SharedPtr conveyor_state_publisher_;
      void robot_state_callback() const;
    //   // Timer
      rclcpp::TimerBase::SharedPtr conveyor_state_publisher_timer_;
      
    //   // Services
      rclcpp::Service<conveyor_interfaces::srv::EnableConveyor>::SharedPtr enable_conveyor_service_;
      rclcpp::Service<conveyor_interfaces::srv::SetConveyorState>::SharedPtr set_conveyor_state_service_;
  };
}

#endif

