import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from typing import Optional, cast
import yaml

from std_msgs.msg import Bool as BoolMsg

from ros_gz_interfaces.srv import SpawnEntity

from aprs_gz_sim.utils import quaternion_from_euler

import math
import os
from time import sleep

import xml.etree.ElementTree as ET

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import (
    Pose
)

from aprs_interfaces.srv import SpawnPart, SpawnSensor

from ament_index_python.packages import get_package_share_directory
        
class Error(Exception):
  def __init__(self, value: str):
      self.value = value

  def __str__(self):
      return repr(self.value)

class EnvironmentStartup(Node):
    gear_offsets_ = {
        "small_gear_tray": {
            'slot_1': (-0.045, 0.045),
            'slot_2': (0.045, 0.045),
            'slot_3': (-0.045, -0.045),
            'slot_4': (0.045, -0.045),
        },
        "medium_gear_tray": {
            'slot_1': (-0.050, 0.050),
            'slot_2': (0.050, 0.050),
            'slot_3': (-0.050, -0.050),
            'slot_4': (0.050, -0.050),
        },
        "large_gear_tray": {
            'slot_1': (-0.052, 0.06),
            'slot_2': (0.052, 0.06),
        },
        "m2l1_kit_tray": {
            'lg_1': (0.0, -0.075),
            'mg_1': (-0.065, 0.0),
            'mg_2': (0.065, 0.0),
        },
        "s2l2_kit_tray": {
            'lg_1': (-0.052, 0.060),
            'lg_2': (0.052, 0.060),
            'sg_1': (-0.045, -0.045),
            'sg_2': (0.045, -0.045),
    }}
    
    colors = {
        'blue': (0, 0, 168),
        'green': (0, 100, 0),
        'red': (139, 0, 0),
        'purple': (138, 0, 226),
        'orange': (255, 140, 0),
        'black': (0, 0, 0)
    }
    
    
    def __init__(self):
        super().__init__("environment_startup_node")
        """Helper node to spawn models, sensors and parts into the simulation.

        This class wraps ROS/Gazebo spawn services and provides convenience
        methods to read SDF/XML templates, adjust colors, and call the
        appropriate spawn services for sensors, parts, gears and trays.
        """


        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.env_ready = False
        self.environment_ready_publisher = self.create_publisher(BoolMsg, '/aprs_environment_ready', latching_qos)
                
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        self.part_count = 0
        self.tray_count = 0
        
        self.used_ros_topic_names = []
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.spawn_part_client = self.create_client(SpawnPart, "/spawn_part")
        self.spawn_sensor_client = self.create_client(SpawnSensor, "/spawn_sensor")

    def read_yaml(self, path):
        """Read a YAML file from disk and return the parsed object.

        On parse failure an empty dict is returned and an error is logged.
        """
        with open(path, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError:
                self.get_logger().error("Unable to read configuration file")
                return {}   

    def get_advanced_logical_camera_xml(self, file_path, sensor_type, name = "camera_1"):
        """Return an adjusted SDF/XML string for an advanced logical camera.

        The function reads the camera SDF, rewrites topic and plugin fields to
        use the provided name and returns the XML as a string.
        """
        
        
        xml = ET.fromstring(self.get_sdf(file_path))
                        
        xml.find("model").find("link").find("sensor").find("topic").text = f"{name}_gz_topic" #type: ignore
                
        xml.find("model").find("link").find("sensor").find("plugin").find("rostopic").text = f"{name}_ros_topic" #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("gztopic").text = f"{name}_gz_topic" #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("camera_name").text = name #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("frame_name").text = f"{name}_gz_topic"   #type: ignore
        
        ray_sensors = ["break_beam", "proximity", "laser_profiler", "lidar"]
        if sensor_type in ray_sensors:
            plugin = xml.find('model').find('link').find('sensor').find('plugin') #type: ignore

            plugin.find('sensor_name').text = name #type: ignore
            plugin.find('frame_name').text = name + "_frame" #type: ignore
        
        cameras = ['rgb_camera', 'rgbd_camera', 'basic_logical_camera', 'advanced_logical_camera']
        if sensor_type in cameras:
            plugin = xml.find('model').find('link').find('sensor').find('plugin') #type: ignore

            plugin.find('camera_name').text = name #type: ignore
            plugin.find('frame_name').text = name + "_frame" #type: ignore

        return ET.tostring(xml, encoding="unicode")

    def get_rgb_camera_xml(self, file_path, sensor_type, name = "camera_1"):
        """Return an adjusted SDF/XML string for an RGB camera sensor.

        Adjusts ROS topic and plugin fields to use the provided name.
        """
        
        
        xml = ET.fromstring(self.get_sdf(file_path))
        
                
        xml.find("model").find("link").find("sensor").find("topic").text = f"{name}_gz_topic" #type: ignore
                
        xml.find("model").find("link").find("sensor").find("plugin").find("rgb_img_ros_topic").text = f"/ariac/sensors/{name}/rgb_image" #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("cam_info_ros_topic").text = f"/ariac/sensors/{name}/camera_info" #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("gz_topic").text = f"{name}_gz_topic" #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("cam_info_gz_topic").text = f"{name}_gz_topic_info" #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("camera_name").text = name #type: ignore
        
        ray_sensors = ["break_beam", "proximity", "laser_profiler", "lidar"]
        if sensor_type in ray_sensors:
            plugin = xml.find('model').find('link').find('sensor').find('plugin') #type: ignore

            plugin.find('sensor_name').text = name #type: ignore
            plugin.find('frame_name').text = name + "_frame" #type: ignore
        
        cameras = ['rgb_camera', 'rgbd_camera', 'basic_logical_camera', 'advanced_logical_camera']
        if sensor_type in cameras:
            plugin = xml.find('model').find('link').find('sensor').find('plugin') #type: ignore

            plugin.find('camera_name').text = name #type: ignore

        return ET.tostring(xml, encoding="unicode")

    def get_rgbd_camera_xml(self, file_path, sensor_type, name = "camera_1"):
        """Return an adjusted SDF/XML string for an RGB-D camera sensor.

        Adjusts ROS topic and plugin fields to use the provided name.
        """
        
        
        xml = ET.fromstring(self.get_sdf(file_path))
                
        xml.find("model").find("link").find("sensor").find("topic").text = f"{name}_gz_topic" #type: ignore
                
        xml.find("model").find("link").find("sensor").find("plugin").find("rgb_img_ros_topic").text = f"/ariac/sensors/{name}/rgb_image" #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("cam_info_ros_topic").text = f"/ariac/sensors/{name}/camera_info" #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("gz_topic").text = f"{name}_gz_topic" #type: ignore
        xml.find("model").find("link").find("sensor").find("plugin").find("depth_img_ros_topic").text = f"/ariac/sensors/{name}/depth_image" #type: ignore
        
        ray_sensors = ["break_beam", "proximity", "laser_profiler", "lidar"]
        if sensor_type in ray_sensors:
            plugin = xml.find('model').find('link').find('sensor').find('plugin') #type: ignore

            plugin.find('sensor_name').text = name #type: ignore
            plugin.find('frame_name').text = name + "_frame" #type: ignore
        
        cameras = ['rgb_camera', 'rgbd_camera', 'basic_logical_camera', 'advanced_logical_camera']
        if sensor_type in cameras:
            plugin = xml.find('model').find('link').find('sensor').find('plugin') #type: ignore

            plugin.find('camera_name').text = name #type: ignore

        return ET.tostring(xml, encoding="unicode")
    
    def spawn_sensor(self, name: str, sensor_type: str, xyz: list[float], rpy: list[float]):
        """Spawn a sensor model into the simulation via the SpawnSensor service.

        Args:
            name: Unique name for the spawned sensor.
            sensor_type: Sensor model name (e.g. 'rgb_camera').
            xyz: Position of the sensor as [x,y,z].
            rpy: Orientation of the sensor as [roll,pitch,yaw].
        """
        
        new_sensor_pose = Pose()
        new_sensor_pose.position.x = float(xyz[0])
        new_sensor_pose.position.y = float(xyz[1])
        new_sensor_pose.position.z = float(xyz[2])
        orientation = quaternion_from_euler(*rpy)
        new_sensor_pose.orientation.x = float(orientation[0])
        new_sensor_pose.orientation.y = float(orientation[1])
        new_sensor_pose.orientation.z = float(orientation[2])
        new_sensor_pose.orientation.w = float(orientation[3])
        
        request = SpawnSensor.Request()
        request.name = name
        request.pose = new_sensor_pose
        
        file_path = os.path.join(get_package_share_directory("ariac_gz_plugins"), "models", sensor_type, "model.sdf")
        
        if sensor_type == "advanced_logical_camera":
            request.xml = self.get_advanced_logical_camera_xml(file_path, sensor_type, name)
        elif sensor_type == "rgb_camera":
            request.xml = self.get_rgb_camera_xml(file_path, sensor_type, name)
        elif sensor_type == "rgbd_camera":
            request.xml = self.get_rgbd_camera_xml(file_path, sensor_type, name)
        
        future = self.spawn_sensor_client.call_async(request)
            
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if not future.done():
            raise Error("Timeout reached when calling spawn_sensor service")

        result: SpawnSensor.Response
        response = cast(SpawnSensor.Response, future.result())

        if not response.success:
            self.get_logger().error("Error calling spawn_sensor service")
    
    def spawn_sensors(self):
        """Read sensor configuration and spawn static sensors listed in the YAML."""
        sensor_file = os.path.join(get_package_share_directory("aprs_gz_sim"), "config", "sensors.yaml")

        sensor_config = self.read_yaml(sensor_file)

        if "static_sensors" in sensor_config.keys():
            static_sensors: dict[str, dict] = sensor_config["static_sensors"]

            for sensor in static_sensors.keys():
                name = sensor
                sensor_type = static_sensors[sensor]["type"]
                sensor_xyz = static_sensors[sensor]["pose"]["xyz"]
                sensor_rpy = static_sensors[sensor]["pose"]["rpy"]

                self.spawn_sensor(name, sensor_type, sensor_xyz, sensor_rpy)
    
    def get_sdf(self, file_path: str) -> str:
        """Read the contents of an SDF/XML file and return as string.

        Returns an empty string on error.
        """
        try:
            f = open(file_path, 'r')
            entity_xml = f.read()
        except IOError:
            return ''
        
        return entity_xml
    
    def get_part_xml(self, p_type, p_color):
        """Return an SDF/XML string for a part with the requested color applied.

        Args:
            p_type: Part type (folder name under gz_models/demo_parts).
            p_color: Color key used to look up RGB values.

        Returns:
            SDF/XML string with color modifications applied.
        """
        file_path = os.path.join(get_package_share_directory("aprs_gz_sim"), "gz_models", "demo_parts", p_type, "model.sdf")
        xml = ET.fromstring(self.get_sdf(file_path))
        
        r, g, b = self.colors[p_color]
        color_string = str(r/255) + " " + str(g/255) + " " + str(b/255) + " 1" 

        for elem in xml.find('model').find('link').findall('visual'): # type: ignore
            if elem.attrib['name'] == "visual":
                elem.find("material").find("ambient").text = color_string # type: ignore
                elem.find("material").find("diffuse").text = color_string # type: ignore

        return ET.tostring(xml, encoding="unicode")
    
    def spawn_part(self, part_type: str, color: str, xyz: list[float]):
        """Spawn a demo part into the simulation using the SpawnPart service.

        Args:
            part_type: The demo part type (e.g. 'battery', 'sensor').
            color: Color key name to apply to the model.
            xyz: Position vector [x,y,z].
        """
        self.get_logger().info("INSIDE SPAWN PART")
        # while True:
        request = SpawnPart.Request()
        
        request.type = part_type
        request.color = color
        
        new_part_pose = Pose()
        new_part_pose.position.x = float(xyz[0])
        new_part_pose.position.z = float(xyz[2])
        new_part_pose.position.y = float(xyz[1])
        new_part_pose.orientation.x = 0.0
        new_part_pose.orientation.y = 0.0
        new_part_pose.orientation.z = 0.0
        new_part_pose.orientation.w = 0.0
        
        request.pose = new_part_pose
        
        request.xml = self.get_part_xml(request.type, request.color)
        
        future = self.spawn_part_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if not future.done():
            raise Error("Timeout reached when calling spawn_part service")

        response = cast(SpawnPart.Response, future.result())

        if not response.success:
            self.get_logger().error("Error calling spawn_part service")
            
    def get_gear_xml(self, gear_size, color):
        """Return an SDF/XML string for a gear with a color applied.

        Args:
            gear_size: Size key for demo gear (e.g. 'small_gear').
            color: Color name to apply.

        Returns:
            Modified SDF/XML for the gear.
        """
        file_path = os.path.join(get_package_share_directory("aprs_gz_sim"), "gz_models", "demo_parts", gear_size, "model.sdf")
        self.get_logger().info(file_path)
        xml = ET.fromstring(self.get_sdf(file_path))
        
        r, g, b = self.colors[color]
        color_string = str(r/255) + " " + str(g/255) + " " + str(b/255) + " 1" 

        for elem in xml.find('model').find('link').findall('visual'): # type: ignore
            if elem.attrib['name'] == "visual":
                elem.find("material").find("ambient").text = color_string # type: ignore
                elem.find("material").find("diffuse").text = color_string # type: ignore

        return ET.tostring(xml, encoding="unicode")
    
    def spawn_gear(self, name: str, gear_size: str, color: str, xyz: list[float], tray_name: Optional[str] = None):
        """Spawn a gear model into the simulation via SpawnPart service.

        Args:
            name: Base name for the spawned gear.
            gear_size: The gear size/type.
            color: Color key to apply.
            xyz: Position vector [x,y,z].
            tray_name: Optional tray suffix to append to the spawned part name.
        """
        self.get_logger().info("INSIDE SPAWN GEAR")

        request = SpawnPart.Request()
        
        suffix = ""
        
        if tray_name is not None:
            suffix = "_" + tray_name
        
        gear_size = gear_size.replace("_gear", "")
        
        gear_name = gear_size + "_gear"
        
        request.name = name
        request.type = gear_name + suffix
        request.color = color
        
        new_part_pose = Pose()
        new_part_pose.position.x = float(xyz[0])
        new_part_pose.position.z = float(xyz[2])
        new_part_pose.position.y = float(xyz[1])
        new_part_pose.orientation.x = 0.0
        new_part_pose.orientation.y = 0.0
        new_part_pose.orientation.z = 0.0
        new_part_pose.orientation.w = 0.0
        
        request.pose = new_part_pose
        
        request.xml = self.get_gear_xml(gear_name, request.color)
                
        future = self.spawn_part_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if not future.done():
            raise Error("Timeout reached when calling spawn_part service")

        response = cast(SpawnPart.Response, future.result())

        if not response.success:
            self.get_logger().error("Error calling spawn_part service")

        else:
            self.get_logger().info("\n"*5 + "Successfully spwned gear" + "\n"*5)
    
    def get_tray_xml(self, tray_name, color):
        """Return an SDF/XML string for a tray model with the requested color.

        Args:
            tray_name: Name of the tray model folder under gz_models/demo_parts.
            color: Color key used to look up RGB values from self.colors.

        Returns:
            A string containing the tray SDF/XML with modified material colors.
        """
        self.get_logger().info("Tray Name: "+ tray_name)
        file_path = os.path.join(get_package_share_directory("aprs_gz_sim"), "gz_models", "demo_parts", tray_name, "model.sdf")
        self.get_logger().info(file_path)
        xml = ET.fromstring(self.get_sdf(file_path))
        
        r, g, b = self.colors[color]
        color_string = str(r/255) + " " + str(g/255) + " " + str(b/255) + " 1" 

        for elem in xml.find('model').find('link').findall('visual'): # type: ignore
            if elem.attrib['name'] == "visual":
                elem.find("material").find("ambient").text = color_string # type: ignore
                elem.find("material").find("diffuse").text = color_string # type: ignore

        return ET.tostring(xml, encoding="unicode")
    
    def spawn_tray(self, name: str, tray_type: str, color: str, xyz: list[float], rotation: float = 0.0, occupied_slots:list[tuple[str, str]]=[]):
        """Spawn a tray model into the simulation and optionally spawn parts in slots.

        This method constructs a SpawnPart request for the tray model, sets
        its pose based on the provided xyz and rotation (degrees), calls the
        spawn service, and if occupied_slots are provided will spawn gears
        at the corresponding offsets.

        Args:
            name: Logical name for the spawned tray.
            tray_type: Tray type string (e.g. 'm2l1_kit_tray').
            color: Color key string to apply to the tray visuals.
            xyz: Position vector [x,y,z].
            rotation: Rotation about the tray's vertical axis in degrees.
            occupied_slots: Optional list of tuples (gear_name, slot_key) that
                indicate which gears should be spawned in which tray slots.
        """
        self.get_logger().info("INSIDE SPAWN TRAY")

        request = SpawnPart.Request()
        
        tray_type = tray_type.replace("_tray", "")
        
        request.name = name
        request.type = tray_type + "_tray"
        request.color = color
        
        new_part_pose = Pose()
        new_part_pose.position.x = float(xyz[0])
        new_part_pose.position.z = float(xyz[2])
        new_part_pose.position.y = float(xyz[1])
        rad_rot = rotation*math.pi/180
        orientation = quaternion_from_euler(rad_rot, 0.0, 3.14159)
        new_part_pose.orientation.x = float(orientation[0])
        new_part_pose.orientation.y = float(orientation[1])
        new_part_pose.orientation.z = float(orientation[2])
        new_part_pose.orientation.w = float(orientation[3])
        
        request.pose = new_part_pose
        
        request.xml = self.get_tray_xml(request.type, request.color)
                
        future = self.spawn_part_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        self.get_logger().info("After future")

        if not future.done():
            raise Error("Timeout reached when calling spawn_part service")

        response = cast(SpawnPart.Response, future.result())

        if not response.success:
            self.get_logger().error("Error calling spawn_part service")

        else:
            self.get_logger().info("\n"*5 + "Successfully spwned tray" + "\n"*5)
            occupied_slots = list(set(occupied_slots))
            if len(occupied_slots) > 0:
                sleep(0.5)
            for gear_name, slot in occupied_slots:
                if slot in self.gear_offsets_[tray_type+"_tray"].keys():
                    slot_x, slot_y = self.gear_offsets_[tray_type+"_tray"][slot]
                    self.get_logger().info(f"Slot x: {slot_x} slot_y: {slot_y}")
                    new_x = slot_x * math.cos(rad_rot) - slot_y * math.sin(rad_rot)
                    new_y = slot_x * math.sin(rad_rot) + slot_y * math.cos(rad_rot)
                    self.get_logger().info(f"New x: {new_x} new y: {new_y}")
                    if "small" in tray_type or "sg" in slot:
                        slot_size = "small"
                    elif "medium" in tray_type or "mg" in slot:
                        slot_size = "medium"
                    else:
                        slot_size = "large"
                    
                    self.spawn_gear(
                        gear_name + ("teach" if name.endswith("teach") else ""), 
                        slot_size, 
                        "green", 
                        [xyz[0]+new_x, xyz[1]+new_y, xyz[2]+0.03], 
                        tray_type+f"_tray_{self.tray_count}"
                    )
                    
                else:
                    self.get_logger().error(f"Slot {slot} does not exist in tray {tray_type}_tray")
            self.tray_count+=1
    
    def publish_environment_status(self):
        """Publish the current environment ready status on the latching topic.

        Publishes a std_msgs/Bool on '/aprs_environment_ready' using a
        transient-local (latching) QoS so late-joining nodes receive the
        current state.
        """
        msg = BoolMsg()
        msg.data = self.env_ready
        self.environment_ready_publisher.publish(msg)
    
    def environment_ready(self):
        """Mark the environment ready and publish the status.

        Sets an internal flag and publishes the updated status message.
        """
        self.env_ready = True
        self.publish_environment_status()
        
