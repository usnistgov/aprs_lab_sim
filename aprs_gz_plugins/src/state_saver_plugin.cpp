#include "aprs_plugins/state_saver_plugin.hpp"

#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>

namespace aprs_plugins
{
  StateSaverPlugin::~StateSaverPlugin()
  {
  }

  void StateSaverPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
  {
    // Initialize ROS 2 node if not already initialized
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    ros_node_ = rclcpp::Node::make_shared("state_saver_plugin");

    // Create services
    save_state_service_ = ros_node_->create_service<aprs_interfaces::srv::SaveState>(
      "/aprs/save_state",
      std::bind(&StateSaverPlugin::SaveState, this, std::placeholders::_1, std::placeholders::_2));

    load_state_service_ = ros_node_->create_service<aprs_interfaces::srv::LoadState>(
      "/aprs/load_state",
      std::bind(&StateSaverPlugin::LoadState, this, std::placeholders::_1, std::placeholders::_2));

    gzmsg << "StateSaverPlugin configured. Services available: /aprs/save_state, /aprs/load_state" << std::endl;
  }

  void StateSaverPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
  {
    // Store latest ECM for use in services
    latest_ecm_ = &_ecm;
  }

  void StateSaverPlugin::SaveState(
    const std::shared_ptr<aprs_interfaces::srv::SaveState::Request> _req,
    std::shared_ptr<aprs_interfaces::srv::SaveState::Response> _res)
  {
    std::string save_name = _req->save_state_name;
    gzmsg << "Saving state: " << save_name << std::endl;

    if (!latest_ecm_)
    {
      gzmsg << "ECM not available yet" << std::endl;
      _res->success = false;
      _res->message = "ECM not available yet";
      return;
    }

    // Clear previous saved states
    saved_robot_joint_states.clear();
    saved_tray_gear_poses.clear();

    // Save robot joint states
    SaveRobotStates(*latest_ecm_);

    // Save tray and gear poses
    SaveTrayGearPoses(*latest_ecm_);

    state_saved = true;
    _res->success = true;
    _res->message = "State '" + save_name + "' saved successfully";
    gzmsg << _res->message << std::endl;
  }

  void StateSaverPlugin::LoadState(
    const std::shared_ptr<aprs_interfaces::srv::LoadState::Request> _req,
    std::shared_ptr<aprs_interfaces::srv::LoadState::Response> _res)
  {
    std::string load_name = _req->save_state_name;

    if (!state_saved)
    {
      _res->success = false;
      _res->message = "No state has been saved yet. Please save a state first.";
      gzwarn << _res->message << std::endl;
      return;
    }

    if (!latest_ecm_)
    {
      gzmsg << "ECM not available yet" << std::endl;
      _res->success = false;
      _res->message = "ECM not available yet";
      return;
    }

    gzmsg << "Loading state: " << load_name << std::endl;

    // Restore robot joint states
    RestoreRobotStates(*latest_ecm_);

    // Restore tray and gear poses
    RestoreTrayGearPoses(*latest_ecm_);

    _res->success = true;
    _res->message = "State '" + load_name + "' loaded successfully";
    gzmsg << _res->message << std::endl;
  }

  void StateSaverPlugin::SaveRobotStates(gz::sim::EntityComponentManager &_ecm)
  {
    // Find all robot models in the simulation
    // Based on the simulation setup, robots are likely named with aprs_ prefix or similar
    auto model_entities = _ecm.EntitiesByComponents(
      gz::sim::components::Model(),
      gz::sim::components::Name());

    for (auto entity : model_entities)
    {
      auto name_comp = _ecm.Component<gz::sim::components::Name>(entity);
      if (!name_comp) continue;

      std::string model_name = name_comp->Data();

      // Filter for robot models (assuming they contain robot identifiers)
      if (model_name.find("aprs_") != std::string::npos ||
          model_name.find("fanuc") != std::string::npos ||
          model_name.find("franka") != std::string::npos ||
          model_name.find("motoman") != std::string::npos ||
          model_name.find("ur") != std::string::npos)
      {
        gzmsg << "Found robot model: " << model_name << std::endl;

        gz::sim::Model model(entity);
        auto joint_entities = model.Joints(_ecm);

        std::vector<JointState> joint_states;
        for (auto joint_entity : joint_entities)
        {
          auto joint_name_comp = _ecm.Component<gz::sim::components::Name>(joint_entity);
          if (!joint_name_comp) continue;

          std::string joint_name = joint_name_comp->Data();

          // Get joint position - returns vector of positions for multi-DOF joints
          gz::sim::Joint joint(joint_entity);
          std::optional<std::vector<double>> position_opt = joint.Position(_ecm);

          if (position_opt.has_value() && !position_opt.value().empty())
          {
            // For simplicity, we'll take the first position (assuming 1DOF or saving all)
            JointState joint_state;
            joint_state.joint_name = joint_name;
            joint_state.position = position_opt.value()[0];
            joint_states.push_back(joint_state);

            gzmsg << "  Saved joint '" << joint_name << "': " << position_opt.value()[0] << std::endl;
          }
        }

        if (!joint_states.empty())
        {
          saved_robot_joint_states[model_name] = joint_states;
        }
      }
    }
  }

  void StateSaverPlugin::SaveTrayGearPoses(gz::sim::EntityComponentManager &_ecm)
  {
    // Find tray and gear models
    auto model_entities = _ecm.EntitiesByComponents(
      gz::sim::components::Model(),
      gz::sim::components::Name());

    for (auto entity : model_entities)
    {
      auto name_comp = _ecm.Component<gz::sim::components::Name>(entity);
      if (!name_comp) continue;

      std::string model_name = name_comp->Data();

      // Look for trays and gears
      if (model_name.find("tray") != std::string::npos ||
          model_name.find("gear") != std::string::npos ||
          model_name.find("bin") != std::string::npos ||
          model_name.find("conveyor") != std::string::npos)
      {
        gzmsg << "Found tray/gear model: " << model_name << std::endl;

        gz::sim::Model model(entity);
        // Get the base link and check its WorldPose
        auto base_link = model.LinkByName(_ecm, "base_link");
        if (base_link != gz::sim::kNullEntity)
        {
          gz::sim::Link link(base_link);
          std::optional<gz::math::Pose3d> pose_opt = link.WorldPose(_ecm);

          if (pose_opt.has_value())
          {
            ModelState model_state;
            model_state.model_name = model_name;
            model_state.pose = pose_opt.value();
            saved_tray_gear_poses[model_name] = model_state;

            gzmsg << "  Saved pose for '" << model_name << "': "
                  << pose_opt.value().Pos() << std::endl;
          }
        }
      }
    }
  }

  void StateSaverPlugin::RestoreRobotStates(gz::sim::EntityComponentManager &_ecm)
  {
    gzmsg << "Restoring robot joint states..." << std::endl;

    for (const auto& robot_pair : saved_robot_joint_states)
    {
      const std::string& model_name = robot_pair.first;
      const std::vector<JointState>& joint_states = robot_pair.second;

      // Find the model entity
      auto entities = _ecm.EntitiesByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name());

      gz::sim::Entity model_entity = gz::sim::kNullEntity;
      for (auto entity : entities)
      {
        auto name_comp = _ecm.Component<gz::sim::components::Name>(entity);
        if (name_comp && name_comp->Data() == model_name)
        {
          model_entity = entity;
          break;
        }
      }

      if (model_entity == gz::sim::kNullEntity)
      {
        gzwarn << "Could not find model entity for: " << model_name << std::endl;
        continue;
      }

      gz::sim::Model model(model_entity);

      // Set each joint position using ResetPosition (like conveyor plugin)
      for (const auto& joint_state : joint_states)
      {
        auto joint_entity = model.JointByName(_ecm, joint_state.joint_name);
        if (joint_entity != gz::sim::kNullEntity)
        {
          gz::sim::Joint joint(joint_entity);
          // Create a vector with the position for all DOFs (assuming 1DOF for simplicity)
          std::vector<double> positions = {joint_state.position};
          joint.ResetPosition(_ecm, positions);
          gzmsg << "  Restored joint '" << joint_state.joint_name
                << "' to " << joint_state.position << std::endl;
        }
        else
        {
          gzwarn << "Could not find joint: " << joint_state.joint_name
                 << " in model " << model_name << std::endl;
        }
      }
    }
  }

  void StateSaverPlugin::RestoreTrayGearPoses(gz::sim::EntityComponentManager &_ecm)
  {
    gzmsg << "Restoring tray and gear poses..." << std::endl;

    for (const auto& model_pair : saved_tray_gear_poses)
    {
      const std::string& model_name = model_pair.first;
      const ModelState& model_state = model_pair.second;

      // Find the model entity
      auto entities = _ecm.EntitiesByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name());

      gz::sim::Entity model_entity = gz::sim::kNullEntity;
      for (auto entity : entities)
      {
        auto name_comp = _ecm.Component<gz::sim::components::Name>(entity);
        if (name_comp && name_comp->Data() == model_name)
        {
          model_entity = entity;
          break;
        }
      }

      if (model_entity == gz::sim::kNullEntity)
      {
        gzwarn << "Could not find model entity for: " << model_name << std::endl;
        continue;
      }

      gz::sim::Model model(model_entity);
      model.SetWorldPoseCmd(_ecm, model_state.pose);

      gzmsg << "  Restored pose for '" << model_name << "': "
            << model_state.pose.Pos() << std::endl;
    }
  }
}

// Register this plugin
GZ_ADD_PLUGIN(
  aprs_plugins::StateSaverPlugin,
  gz::sim::System,
  aprs_plugins::StateSaverPlugin::ISystemConfigure,
  aprs_plugins::StateSaverPlugin::ISystemPreUpdate
)

