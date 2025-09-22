#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <aprs_plugins/conveyor_plugin.hpp>

#include <time.h>

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    aprs_plugins::ConveyorPlugin,
    gz::sim::System,
    aprs_plugins::ConveyorPlugin::ISystemPreUpdate,
    aprs_plugins::ConveyorPlugin::ISystemConfigure)

using namespace aprs_plugins;
 
ConveyorPlugin::ConveyorPlugin()
{
}
 
ConveyorPlugin::~ConveyorPlugin()
{
  executor_->cancel();
  thread_executor_spin_.join();
}

void ConveyorPlugin::Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &)
{
  _model = gz::sim::Model(_entity);
  _belt_joint = gz::sim::Joint(_model.JointByName(_ecm, "belt_joint"));
  _belt_joint.EnablePositionCheck(_ecm, true);
  _max_velocity = _sdf->GetElementImpl("max_velocity")->Get<double>();
  
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::string node_name = "conveyor_ros_node";

  _ros_node = rclcpp::Node::make_shared(node_name);

  rclcpp::Parameter sim_time("use_sim_time", true);
  _ros_node->set_parameter(sim_time);

  std::cout << "NODE SET" << std::endl;
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(_ros_node);

  auto spin = [this](){
    while(rclcpp::ok(conveyor_context)){
      executor_->spin_once();
    }
  };

  thread_executor_spin_ = std::thread(spin);

  // Publisher
  this->conveyor_state_publisher_ = this->_ros_node->create_publisher<conveyor_interfaces::msg::ConveyorState>("aprs_conveyor/conveyor_state", 10);
  conveyor_state_publisher_timer_ = _ros_node->create_wall_timer(std::chrono::duration<double>(0.1), std::bind(&ConveyorPlugin::robot_state_callback, this));
  std::cout << "Plugin loaded" << std::endl;

  // Services
  enable_conveyor_service_ = _ros_node->create_service<conveyor_interfaces::srv::EnableConveyor>(
    "/aprs_conveyor/enable_conveyor", 
    std::bind(
      &ConveyorPlugin::enable_conveyor_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  set_conveyor_state_service_ = _ros_node->create_service<conveyor_interfaces::srv::SetConveyorState>(
    "/aprs_conveyor/set_conveyor_state", 
    std::bind(
      &ConveyorPlugin::set_conveyor_state_callback, this,
      std::placeholders::_1, std::placeholders::_2));

}

void ConveyorPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm)
{ 
  // gzmsg << "preupdate" << std::endl;

  if(!_enabled){
    _belt_velocity = 0;
    _belt_direction = 0;
    _belt_joint.SetVelocity(_ecm, {_belt_velocity});
    return;
  }
  if(_belt_direction == 0){
    _belt_joint.SetVelocity(_ecm, {_belt_velocity});
  }else{
    _belt_joint.SetVelocity(_ecm, {-1 * _belt_velocity});
  }
  double position = 0.0;
  std::optional<std::vector<double>> position_vector = _belt_joint.Position(_ecm);
  if(position_vector.has_value()){
    if(position_vector.value().size() > 0){
      _belt_position = position_vector.value()[0];
    }else{
      _belt_position = 0.0;
    }
  }
  if(abs(_belt_position) >= _conveyor_limit){
    _belt_joint.ResetPosition(_ecm, _reset_positions);
  }
}

void ConveyorPlugin::robot_state_callback() const{
  auto state_msg = conveyor_interfaces::msg::ConveyorState();
  state_msg.enabled = _enabled;
  state_msg.speed = _belt_velocity;
  state_msg.direction = _belt_direction;
  conveyor_state_publisher_->publish(state_msg);
}

void ConveyorPlugin::enable_conveyor_callback(
  const std::shared_ptr<conveyor_interfaces::srv::EnableConveyor::Request> request, 
  std::shared_ptr<conveyor_interfaces::srv::EnableConveyor::Response> response){
  if(request->enable && _enabled){
    gzmsg << "Conveyor is already enabled" << std::endl;
    response->message = "Conveyor is already enabled";
    response->success = false;
    return;
  }

  if(!request->enable && !_enabled){
    gzmsg << "Conveyor is already disabled" << std::endl;
    response->message = "Conveyor is already disabled";
    response->success = false;
    return;
  }
  
  if(request->enable){
    _enabled = true;
    response->message = "Conveyor is now enabled";
    response->success = true;
    return;
  }
  _enabled = false;
  response->message = "Conveyor is now disabled";
  response->success = true;
}


void ConveyorPlugin::set_conveyor_state_callback(
  const std::shared_ptr<conveyor_interfaces::srv::SetConveyorState::Request> request, 
  std::shared_ptr<conveyor_interfaces::srv::SetConveyorState::Response> response){
    if(!_enabled){
      gzmsg << "Conveyor is disabled and must be enabled to set state" << std::endl;
      response->message = "Conveyor is disabled and must be enabled to set state";
      response->success = false;
      return;
    }

    if(request->speed < 0 || request->speed > _max_velocity){
      gzmsg << "Speed must be between 0 and " << std::to_string (_max_velocity) << std::endl;
      response->message = "Speed must be between 0 and " + std::to_string (_max_velocity);
      response->success = false;
      return;
    }

    if (request->direction != 0 && request->direction != 1){
      gzmsg << "Direction must be either 0 (forward) or 1 (backwards)" << std::endl;
      response->message = "Direction must be either 0 (forward) or 1 (backwards)";
      response->success = false;
      return;
    }

    gzmsg << "Setting speed to " + std::to_string(request->speed) + " and direction to " + ((request->direction==0)?"forward":"backward");

    _belt_velocity = request->speed;
    _belt_direction = request->direction;
    response->message = "Set speed to " + std::to_string(request->speed) + " and direction to " + ((request->direction==0)?"forward":"backward");
    response->success = true;
}


