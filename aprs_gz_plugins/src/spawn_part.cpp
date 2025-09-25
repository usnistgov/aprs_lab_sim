#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <aprs_plugins/spawn_part.hpp>

#include <time.h>

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    aprs_plugins::SpawnPartPlugin,
    gz::sim::System,
    aprs_plugins::SpawnPartPlugin::ISystemConfigure)

using namespace aprs_plugins;
 
SpawnPartPlugin::~SpawnPartPlugin()
{
  executor_->cancel();
  thread_executor_spin_.join();
}

void SpawnPartPlugin::Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &)
{  
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::string node_name = "spawn_part_ros_node";

  _ros_node = rclcpp::Node::make_shared(node_name);

  rclcpp::Parameter sim_time("use_sim_time", true);
  _ros_node->set_parameter(sim_time);

  std::cout << "NODE SET" << std::endl;
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(_ros_node);

  auto spin = [this](){
    while(rclcpp::ok()){
      executor_->spin_once();
    }
  };

  thread_executor_spin_ = std::thread(spin);

  gz_node = std::make_shared<gz::transport::Node>();

  spawn_part_srv_ = _ros_node->create_service<aprs_interfaces::srv::SpawnPart>(
      "/spawn_part",
      std::bind(&SpawnPartPlugin::spawn_part_cb_, this, std::placeholders::_1, std::placeholders::_2)
  );

  spawn_sensor_srv_ = _ros_node->create_service<aprs_interfaces::srv::SpawnSensor>(
      "/spawn_sensor",
      std::bind(&SpawnPartPlugin::spawn_sensor_cb_, this, std::placeholders::_1, std::placeholders::_2)
  );

}

void SpawnPartPlugin::spawn_part_cb_(
    const std::shared_ptr<aprs_interfaces::srv::SpawnPart::Request> request,
    std::shared_ptr<aprs_interfaces::srv::SpawnPart::Response> response
){
    std::string world_name = "lab";
    std::string service{"/world/" + world_name + "/create"};

    // Request message
    gz::msgs::EntityFactory req;

    if (request->type.find("tray") != std::string::npos){
        req.set_name(request->type + "_" + std::to_string(tray_count));
        tray_count++;
    }
    else if (request->type.find("gear") != std::string::npos){
        req.set_name(request->type + "_" + std::to_string(gear_count));
        gear_count++;
    }
    else {
        req.set_name(request->type + "_" + std::to_string(part_count));
        part_count++;
    }

    // File
    req.set_sdf(request->xml);

    // Pose
    std::vector rpy = get_rpy_from_quaternion(request->pose.orientation.x, request->pose.orientation.y,
                                              request->pose.orientation.z, request->pose.orientation.w);
    gz::math::Pose3d pose{request->pose.position.x, request->pose.position.y, request->pose.position.z,
                          rpy[0], rpy[1], rpy[2]};
    gz::msgs::Set(req.mutable_pose(), pose);

    // Request
    gz::transport::Node node;
    gz::msgs::Boolean rep;
    bool result;
    unsigned int timeout = 5000;
    bool executed = node.Request(service, req, timeout, rep, result);

    if (executed) {
        if (result && rep.data()) {
          gzmsg << "Requested creation of entity.";
        } else {
          gzerr << "Failed request to create entity.\n %s", req.DebugString();
        }
    } else {
        gzerr << "Request to create entity from service [%s] timed out..\n %s", service, req.DebugString();
        response->set__success(false);
        return;
    }
    gzmsg << "OK creation of entity.";
    response->set__success(true);
}

void SpawnPartPlugin::spawn_sensor_cb_(
    const std::shared_ptr<aprs_interfaces::srv::SpawnSensor::Request> request,
    std::shared_ptr<aprs_interfaces::srv::SpawnSensor::Response> response
){
    std::string world_name = "lab";
    std::string service{"/world/" + world_name + "/create"};

    // Request message
    gz::msgs::EntityFactory req;

    req.set_name(request->name + "_" + std::to_string(sensor_count));
    // req.set_name(request->name);
    sensor_count++;

    // File
    req.set_sdf(request->xml);

    // Pose
    std::vector rpy = get_rpy_from_quaternion(request->pose.orientation.x, request->pose.orientation.y,
                                              request->pose.orientation.z, request->pose.orientation.w);
    gz::math::Pose3d pose{request->pose.position.x, request->pose.position.y, request->pose.position.z,
                          rpy[0], rpy[1], rpy[2]};
    gz::msgs::Set(req.mutable_pose(), pose);

    // Request
    gz::transport::Node node;
    gz::msgs::Boolean rep;
    bool result;
    unsigned int timeout = 5000;
    bool executed = node.Request(service, req, timeout, rep, result);

    if (executed) {
        if (result && rep.data()) {
        gzmsg << "Requested creation of entity.";
        } else {
        gzerr << "Failed request to create entity.\n %s", req.DebugString();
        }
    } else {
        gzerr << "Request to create entity from service [%s] timed out..\n %s", service, req.DebugString();
        response->set__success(false);
        return;
    }
    gzmsg << "OK creation of entity.";
    response->set__success(true);
}

std::vector<float> SpawnPartPlugin::get_rpy_from_quaternion(float x, float y, float z, float w){

    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    float roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (w * y - z * x);
    float pitch = asin(sinp);

    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    float yaw = atan2(siny_cosp, cosy_cosp);

    std::vector<float> rpy = {roll, pitch, yaw};

    return rpy;
}
