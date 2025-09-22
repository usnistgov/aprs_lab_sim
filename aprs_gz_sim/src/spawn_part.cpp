#include <aprs_gz_sim/spawn_part.hpp>

SpawnPart::SpawnPart()
    : Node("part_spawner") 
{
    spawn_part_srv_ = this->create_service<aprs_interfaces::srv::SpawnPart>(
        "/spawn_part",
        std::bind(&SpawnPart::spawn_part_cb_, this, std::placeholders::_1, std::placeholders::_2)
    );

    spawn_sensor_srv_ = this->create_service<aprs_interfaces::srv::SpawnSensor>(
        "/spawn_sensor",
        std::bind(&SpawnPart::spawn_sensor_cb_, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void SpawnPart::spawn_part_cb_(
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
        RCLCPP_INFO(this->get_logger(), "Requested creation of entity.");
        } else {
        RCLCPP_ERROR(
            this->get_logger(), "Failed request to create entity.\n %s",
            req.DebugString().c_str());
        }
    } else {
        RCLCPP_ERROR(
        this->get_logger(), "Request to create entity from service [%s] timed out..\n %s",
        service.c_str(), req.DebugString().c_str());
        response->set__success(false);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "OK creation of entity.");
    response->set__success(true);
}

void SpawnPart::spawn_sensor_cb_(
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
        RCLCPP_INFO(this->get_logger(), "Requested creation of entity.");
        } else {
        RCLCPP_ERROR(
            this->get_logger(), "Failed request to create entity.\n %s",
            req.DebugString().c_str());
        }
    } else {
        RCLCPP_ERROR(
        this->get_logger(), "Request to create entity from service [%s] timed out..\n %s",
        service.c_str(), req.DebugString().c_str());
        response->set__success(false);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "OK creation of entity.");
    response->set__success(true);
}

std::vector<float> SpawnPart::get_rpy_from_quaternion(float x, float y, float z, float w){

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

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);

  auto spawn_part_node = std::make_shared<SpawnPart>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(spawn_part_node);

  executor.spin();

  rclcpp::shutdown();
}