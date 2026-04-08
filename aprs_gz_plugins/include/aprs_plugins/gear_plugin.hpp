#ifndef ARIAC_PLUGINS__GEAR_PLUGIN_HPP_
#define ARIAC_PLUGINS__GEAR_PLUGIN_HPP_

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>

namespace aprs_plugins
{
  // enum class LockState {
  //   LOCKED,
  //   UNLOCKED,
  //   LOCK_REQUESTED,
  //   UNLOCK_REQUESTED
  // };

  class GearPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    public:
      ~GearPlugin();

      void Configure (
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_event_mgr) override;
      
      void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;
    
    private:
      void contact_cb(const gz::msgs::Contacts &);

      std::shared_ptr<gz::transport::Node> gz_node;

      std::string gz_contact_topic;
      
      gz::sim::Entity gear_entity;
      gz::sim::Entity gear_base_link;

      gz::math::Pose3d goal_gear_pose = gz::math::Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

      bool teleport_requested = false;
      bool teleported = false;
  };
}

#endif //ARIAC_PLUGINS__GEAR_PLUGIN_HPP_