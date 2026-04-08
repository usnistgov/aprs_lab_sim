#include "aprs_plugins/slot_plugin.hpp"

GZ_ADD_PLUGIN(
  aprs_plugins::SlotPlugin,
  gz::sim::System,
  aprs_plugins::SlotPlugin::ISystemConfigure,
  aprs_plugins::SlotPlugin::ISystemPreUpdate
)

using namespace aprs_plugins;

SlotPlugin::~SlotPlugin(){}

void SlotPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  tray_entity = _ecm.ParentEntity(_entity);

  if (tray_entity != gz::sim::kNullEntity && 
        _ecm.Component<gz::sim::components::Model>(tray_entity) != nullptr) 
    {        
        auto name_comp = _ecm.Component<gz::sim::components::Name>(tray_entity);
        if (name_comp) {
            gzmsg << "Successfully found parent model: " << name_comp->Data() << std::endl;
        }   
    }
    else
    {
        gzerr << "Plugin is attached to an entity that does not have a parent Model!" << std::endl;
    }
  auto model = gz::sim::Model(_entity);
  slot_entity = _entity;
  tray_base_link = gz::sim::Model(tray_entity).LinkByName(_ecm, "base_link");

  gz_node = std::make_shared<gz::transport::Node>();

  gz_contact_topic =  "/world/lab/model/" + gz::sim::Model(tray_entity).Name(_ecm) + "/link/" + model.Name(_ecm) + "/sensor/contact_sensor/contact";

  gz_node->Subscribe(gz_contact_topic, &SlotPlugin::contact_cb, this);
}

void SlotPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if(current_state == TeleportLockState::LOCKED || current_state == TeleportLockState::WAITING){
    return;
  }

  switch(current_state){
    case TeleportLockState::WAITING:
    case TeleportLockState::LOCKED:
      return;
      break;
    case TeleportLockState::TELEPORT_REQUESTED:
    {
      std::optional<gz::sim::Entity> gear_entity_opt = _ecm.EntityByName(gear_in_contact);

      if (!gear_entity_opt.has_value()){
        return;
      }

      gz::sim::Entity gear_entity = gear_entity_opt.value();

      gz::sim::Model gear_model = gz::sim::Model(gear_entity);
      current_gear_base_link_entity = gear_model.LinkByName(_ecm, "base_link");

      std::optional<gz::math::Pose3d> gear_pose_opt = gz::sim::Link(current_gear_base_link_entity).WorldPose(_ecm);
        
      if(!gear_pose_opt.has_value()){
        return;
      }
      
      gz::math::Pose3d gear_pose = gear_pose_opt.value();

      gz::math::Pose3d target_pose = goal_gear_pose;

      target_pose.SetX(gear_pose.X());
      target_pose.SetY(gear_pose.Y());
      target_pose.SetZ(gear_pose.Z()+0.001);

      gz::sim::Model(gear_entity).SetWorldPoseCmd(_ecm, target_pose);
      
      current_state = TeleportLockState::WAIT_AFTER_TELEPORT;
      teleport_iter = _info.iterations;

      gz_node->Unsubscribe(gz_contact_topic);
      break;
    }
    case TeleportLockState::WAIT_AFTER_TELEPORT:
      if(_info.iterations - teleport_iter < 100){
        return;
      }
      current_state = TeleportLockState::LOCK_REQUESTED;
      break;
    case TeleportLockState::LOCK_REQUESTED:
    {
      lock_joint = _ecm.CreateEntity();
      _ecm.CreateComponent(lock_joint, gz::sim::components:: DetachableJoint({
        tray_base_link,
        current_gear_base_link_entity,
        "fixed"
      }));

      current_state = TeleportLockState::LOCKED;
      break;
    }
  }
}

void SlotPlugin::contact_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  if(current_state != TeleportLockState::WAITING){
    return;
  }

  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();
    if (collision.find("gear") != std::string::npos){
      gear_in_contact = collision.substr(0, collision.find("::"));
      current_state = TeleportLockState::TELEPORT_REQUESTED;
    }
  }
}