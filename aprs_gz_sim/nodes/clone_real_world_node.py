#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math

import tf2_ros

from rclpy.time import Time, Duration
from rclpy.parameter import Parameter

from geometry_msgs.msg import Quaternion, Pose

from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool

from time import sleep

from math import pi
from aprs_interfaces.msg import Trays, Tray, SlotInfo

from aprs_gz_sim.environment_startup import EnvironmentStartup

from aprs_gz_sim.utils import quaternion_from_euler, build_pose, multiply_pose, rad_to_deg, rpy_from_quaternion, quaternion_to_msg

class CloneNode(Node):
    """Node which listens to real-world tray topics and spawns equivalent
    entities into the simulation environment.

    The node subscribes to tray topics for fanuc, motoman and teach stations and
    uses the EnvironmentStartup helper to spawn tray entities in the simulation
    once tray information is received.
    """
    tray_types_ = ["small_gear", "medium_gear", "large_gear", "m2l1_kit", "s2l2_kit"]
    def __init__(self):
        super().__init__('clone_node')

        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])
        
        self.spawner_node = EnvironmentStartup()
        self.motoman_trays_spawned = False
        self.fanuc_trays_spawned = False
        self.teach_trays_spawned = False
        
        self.trays_spawned = [self.fanuc_trays_spawned, self.motoman_trays_spawned, self.teach_trays_spawned]
        
        fanuc_orientation = quaternion_to_msg(quaternion_from_euler(0, math.pi, 0))
        motoman_orientation = quaternion_to_msg(quaternion_from_euler(0, math.pi, 0))
        teach_orientation = quaternion_to_msg(quaternion_from_euler(0, math.pi, math.pi))

        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        self.fanuc_vision_pose_ = build_pose(-0.25, 0.575 - 0.5, 0.9, fanuc_orientation)
        self.motoman_vision_pose_ = build_pose(0.35, 0.325 - 0.5, 0.9, motoman_orientation)
        self.teach_vision_pose_ = build_pose(-2.35, -1.4 - 0.5, 0.76, teach_orientation)
        
        self.fanuc_trays_info_sub = self.create_subscription(Trays, '/fanuc/table_trays_info', self.update_fanuc_trays, 10)
        self.motoman_trays_info_sub = self.create_subscription(Trays, '/motoman/table_trays_info', self.update_motoman_trays, 10)
        self.teach_trays_info_sub = self.create_subscription(Trays, '/teach/table_trays_info', self.update_teach_trays, 10)

    def update_motoman_trays(self, msg: Trays):
        image_frame = 'motoman_table_image'
        table_frame = 'optical_table_corner_frame'


        try:
            self.table_frame_transform = self.tf_buffer.lookup_transform('world', table_frame, self.get_clock().now(), timeout=Duration(seconds=1)).transform

            self.image_frame_transform = self.tf_buffer.lookup_transform('world', image_frame, self.get_clock().now(), timeout=Duration(seconds=1)).transform

            self.table_frame_pose = Pose()
            self.table_frame_pose.position = self.table_frame_transform.translation
            self.table_frame_pose.orientation = self.table_frame_transform.rotation

            self.image_frame_pose = Pose()
            self.image_frame_pose.position = self.image_frame_transform.translation
            self.image_frame_pose.orientation = self.image_frame_transform.rotation

        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return
        
        if self.motoman_trays_spawned:
            return
        
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays # type: ignore
        
        for tray in all_trays:
            tray: Tray
            image_pose = multiply_pose(self.table_frame_pose, self.image_frame_pose)
            world_pose = multiply_pose(image_pose, tray.tray_pose.pose)

            
            tray_type = self.tray_types_[tray.identifier - 13]
            tray_color = "black"
            
            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            
            rotation = rad_to_deg(rpy_from_quaternion(world_pose.orientation)[-1])
            
            occupied_slots = []
            for slot in tray.slots:
                slot: SlotInfo
                if slot.occupied:
                    occupied_slots.append((slot.name, "_".join(slot.name.split("_")[-2:])))
            
            self.spawner_node.spawn_tray(tray.name, tray_type, tray_color, xyz, rotation, occupied_slots)
        self.motoman_trays_spawned = True

        self.destroy_subscription(self.motoman_trays_info_sub)
        if all(self.trays_spawned):
            self.spawner_node.environment_ready()
    
    def update_fanuc_trays(self, msg: Trays):
        image_frame = 'fanuc_table_image'
        table_frame = 'optical_table_corner_frame'

        self.get_logger().info("Inside fanuc spawn trays")

        try:
            self.table_frame_transform = self.tf_buffer.lookup_transform('world', table_frame, self.get_clock().now(), timeout=Duration(seconds=1)).transform
            self.image_frame_transform = self.tf_buffer.lookup_transform('world', image_frame, self.get_clock().now(), timeout=Duration(seconds=1)).transform

            self.table_frame_pose = Pose()
            self.table_frame_pose.position = self.table_frame_transform.translation
            self.table_frame_pose.orientation = self.table_frame_transform.rotation

            self.image_frame_pose = Pose()
            self.image_frame_pose.position = self.image_frame_transform.translation
            self.image_frame_pose.orientation = self.image_frame_transform.rotation

        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return

        if self.fanuc_trays_spawned:
            return
        
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays # type: ignore
        
        for tray in all_trays:
            image_pose = multiply_pose(self.table_frame_pose, self.image_frame_pose)
            world_pose = multiply_pose(image_pose, tray.tray_pose.pose)
            
            tray_type = self.tray_types_[tray.identifier - 13]
            tray_color = "black"
            
            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            
            rotation = rad_to_deg(rpy_from_quaternion(world_pose.orientation)[-1])
            
            occupied_slots = []
            for slot in tray.slots:
                slot: SlotInfo
                if slot.occupied:
                    occupied_slots.append((slot.name, "_".join(slot.name.split("_")[-2:])))
            
            self.spawner_node.spawn_tray(tray.name, tray_type, tray_color, xyz, rotation, occupied_slots)
        self.fanuc_trays_spawned = True

        self.destroy_subscription(self.fanuc_trays_info_sub)
        if all(self.trays_spawned):
            self.spawner_node.environment_ready()
            
    def update_teach_trays(self, msg: Trays):
        if self.teach_trays_spawned:
            return
        
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays # type: ignore
        
        for tray in all_trays:
            world_pose = multiply_pose(self.teach_vision_pose_, tray.tray_pose.pose)
            
            tray_type = self.tray_types_[tray.identifier - 13]
            tray_color = "black"
            
            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            
            rotation = rad_to_deg(rpy_from_quaternion(world_pose.orientation)[-1])
            
            occupied_slots = []
            for slot in tray.slots:
                slot: SlotInfo
                if slot.occupied:
                    occupied_slots.append((slot.name, "_".join(slot.name.split("_")[-2:])))
            
            self.spawner_node.spawn_tray(tray.name + "_teach", tray_type, tray_color, xyz, rotation, occupied_slots)
        self.teach_trays_spawned = True

        self.destroy_subscription(self.teach_trays_info_sub)
        if all(self.trays_spawned):
            self.spawner_node.environment_ready()

if __name__ == "__main__":
    rclpy.init()

    clone_node = CloneNode()
    
    try:
        rclpy.spin(clone_node)
    except KeyboardInterrupt:
        clone_node.get_logger().info('KeyboardInterrupt caught, shutting down')
    finally:
        clone_node.destroy_node()
        rclpy.shutdown()