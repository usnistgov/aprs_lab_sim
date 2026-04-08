#include "aprs_plugins/gear_plugin.hpp"

GZ_ADD_PLUGIN(
  aprs_plugins::GearPlugin,
  gz::sim::System,
  aprs_plugins::GearPlugin::ISystemConfigure,
  aprs_plugins::GearPlugin::ISystemPreUpdate
)

using namespace aprs_plugins;

GearPlugin::~GearPlugin(){}

void GearPlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  auto model = gz::sim::Model(_entity);
  gear_entity = _entity;
  gear_base_link = model.LinkByName(_ecm, "base_link");

  gz_node = std::make_shared<gz::transport::Node>();

  gz_contact_topic =  "/world/lab/model/" + model.Name(_ecm) + "/link/base_link/sensor/contact_sensor/contact";

  gz_node->Subscribe(gz_contact_topic, &GearPlugin::contact_cb, this);
}

void GearPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if(teleported || !teleport_requested){
    return;
  }

  std::optional<gz::math::Pose3d> gear_pose_opt = gz::sim::Link(gear_base_link).WorldPose(_ecm);
        
  if(!gear_pose_opt.has_value()){
    return;
  }
  
  gz::math::Pose3d gear_pose = gear_pose_opt.value();

  gz::math::Pose3d target_pose = goal_gear_pose;

  target_pose.SetX(gear_pose.X());
  target_pose.SetY(gear_pose.Y());
  target_pose.SetZ(gear_pose.Z()+0.001);

  gz::sim::Model(gear_entity).SetWorldPoseCmd(_ecm, target_pose);

  teleported = true;

  gz_node->Unsubscribe(gz_contact_topic);
}

void GearPlugin::contact_cb(const gz::msgs::Contacts &_gz_contacts_msg){
  if(teleported){
    return;
  }

  for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
    std::string collision = _gz_contacts_msg.contact(i).collision2().name();
    if (collision.find("kit_tray") != std::string::npos){
      teleport_requested = true;
    }
  }
}