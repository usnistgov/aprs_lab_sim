#ifndef ARIAC_PLUGINS__SLOT_PLUGIN_HPP_
#define ARIAC_PLUGINS__SLOT_PLUGIN_HPP_

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
  enum class TeleportLockState {
    WAITING,
    TELEPORT_REQUESTED,
    WAIT_AFTER_TELEPORT,
    LOCK_REQUESTED,
    LOCKED
  };

  class SlotPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
  {
    public:
      ~SlotPlugin();

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
      std::string gear_in_contact;
        
      gz::sim::Entity tray_entity;
      gz::sim::Entity slot_entity;
      gz::sim::Entity tray_base_link;
      gz::sim::Entity lock_joint;
      gz::sim::Entity current_gear_base_link_entity;

      int teleport_iter = -1;

      gz::math::Pose3d goal_gear_pose = gz::math::Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

      TeleportLockState current_state = TeleportLockState::WAITING;
  };
}

#endif //ARIAC_PLUGINS__SLOT_PLUGIN_HPP_