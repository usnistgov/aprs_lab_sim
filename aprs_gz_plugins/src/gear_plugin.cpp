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
  gear_base_link = model.LinkByName(_ecm, "base_link");

  gz_node = std::make_shared<gz::transport::Node>();

  std::string gz_contact_topic =  "/world/lab/model/large_gear_large_gear_tray_3_9/link/base_link/sensor/contact_sensor/contact";
}