#ifndef APRS_PLUGINS__STATE_SAVER_PLUGIN_HPP_
#define APRS_PLUGINS__STATE_SAVER_PLUGIN_HPP_

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/plugin/Register.hh>

#include <aprs_sim_interfaces/srv/save_state.hpp>
#include <aprs_sim_interfaces/srv/load_state.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace aprs_plugins
{
  class StateSaverPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    public:
      ~StateSaverPlugin();

      void Configure (
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_mgr) override;

      void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

    private:
      // Service callbacks
      void SaveState(
        const std::shared_ptr<aprs_sim_interfaces::srv::SaveState::Request> _req,
        std::shared_ptr<aprs_sim_interfaces::srv::SaveState::Response> _res);

      void LoadState(
        const std::shared_ptr<aprs_sim_interfaces::srv::LoadState::Request> _req,
        std::shared_ptr<aprs_sim_interfaces::srv::LoadState::Response> _res);

      // Helper functions
      void SaveRobotStates(gz::sim::EntityComponentManager &_ecm);
      void SaveTrayGearPoses(gz::sim::EntityComponentManager &_ecm);
      void RestoreRobotStates(gz::sim::EntityComponentManager &_ecm);
      void RestoreTrayGearPoses(gz::sim::EntityComponentManager &_ecm);

      // ROS 2 node for services
      std::shared_ptr<rclcpp::Node> ros_node_;
      rclcpp::Service<aprs_sim_interfaces::srv::SaveState>::SharedPtr save_state_service_;
      rclcpp::Service<aprs_sim_interfaces::srv::LoadState>::SharedPtr load_state_service_;

      // Latest ECM from PreUpdate
      gz::sim::EntityComponentManager* latest_ecm_ = nullptr;

      // Saved state
      struct JointState {
        std::string joint_name;
        double position;
      };

      struct ModelState {
        std::string model_name;
        gz::math::Pose3d pose;
      };

      // Map: robot_name -> vector of joint states
      std::map<std::string, std::vector<JointState>> saved_robot_joint_states;
      // Map: model_name -> model pose (for trays and gears)
      std::map<std::string, ModelState> saved_tray_gear_poses;

      bool state_saved = false;
  };
}

#endif // APRS_PLUGINS__STATE_SAVER_PLUGIN_HPP_