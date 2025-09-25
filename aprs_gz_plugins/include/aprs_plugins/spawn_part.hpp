#ifndef ARIAC_PLUGINS__GEAR_PLUGIN_HPP_
#define ARIAC_PLUGINS__GEAR_PLUGIN_HPP_

#include <aprs_interfaces/srv/spawn_part.hpp>
#include <aprs_interfaces/srv/spawn_sensor.hpp>

#include <gflags/gflags.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <sstream>
#include <string>
#include <cmath>
#include <fstream>

#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>

namespace aprs_plugins
{
  class SpawnPartPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure
  {
    public:
      ~SpawnPartPlugin();

      void Configure (
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_mgr) override;
    
    private:
      int sensor_count = 0;
      int tray_count = 0;
      int gear_count = 0;
      int part_count = 0;

      rclcpp::Service<aprs_interfaces::srv::SpawnPart>::SharedPtr spawn_part_srv_;
      rclcpp::Service<aprs_interfaces::srv::SpawnSensor>::SharedPtr spawn_sensor_srv_;

      std::shared_ptr<gz::transport::Node> gz_node;

      rclcpp::Node::SharedPtr _ros_node;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
      std::thread thread_executor_spin_;

      void spawn_part_cb_(const std::shared_ptr<aprs_interfaces::srv::SpawnPart::Request> request, std::shared_ptr<aprs_interfaces::srv::SpawnPart::Response> response);
      void spawn_sensor_cb_(const std::shared_ptr<aprs_interfaces::srv::SpawnSensor::Request> request, std::shared_ptr<aprs_interfaces::srv::SpawnSensor::Response> response);
  
      std::vector<float> get_rpy_from_quaternion(float, float, float, float);
  };
}

#endif //ARIAC_PLUGINS__GEAR_PLUGIN_HPP_