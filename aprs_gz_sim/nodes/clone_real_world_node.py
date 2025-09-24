#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool

from time import sleep

from math import pi
from aprs_interfaces.msg import Trays, Tray, SlotInfo

from aprs_gz_sim.evironment_startup import EnvironmentStartup

from aprs_gz_sim.utils import *

class CloneNode(Node):
    tray_types_ = ["small_gear", "medium_gear", "large_gear", "m2l1_kit", "s2l2_kit"]
    def __init__(self):
        super().__init__('clone_node')
        
        self.spawner_node = EnvironmentStartup()
        self.motoman_trays_spawned = False
        self.fanuc_trays_spawned = False
        self.teach_trays_spawned = False
        
        self.trays_spawned = [self.fanuc_trays_spawned, self.motoman_trays_spawned, self.teach_trays_spawned]
        
        fanuc_vision_quaternion = quaternion_from_euler(0, math.pi, math.pi)
        fanuc_orientation = Quaternion()
        fanuc_orientation.w = fanuc_vision_quaternion[0]
        fanuc_orientation.x = fanuc_vision_quaternion[1]
        fanuc_orientation.y = fanuc_vision_quaternion[2]
        fanuc_orientation.z = fanuc_vision_quaternion[3]

        motoman_vision_quaternion = quaternion_from_euler(0, 0, math.pi)
        motoman_orientation = Quaternion()
        motoman_orientation.w = motoman_vision_quaternion[0]
        motoman_orientation.x = motoman_vision_quaternion[1]
        motoman_orientation.y = motoman_vision_quaternion[2]
        motoman_orientation.z = motoman_vision_quaternion[3]
        
        teach_vision_quaternion = quaternion_from_euler(0, 0, math.pi)
        teach_orientation = Quaternion()
        teach_orientation.w = teach_vision_quaternion[0]
        teach_orientation.x = teach_vision_quaternion[1]
        teach_orientation.y = teach_vision_quaternion[2]
        teach_orientation.z = teach_vision_quaternion[3]

        self.fanuc_vision_pose_ = build_pose(-0.7, 1.2 - 0.5, 0.9, fanuc_orientation)
        self.motoman_vision_pose_ = build_pose(0.0, 0.75 - 0.5, 0.9, motoman_orientation)
        self.teach_vision_pose_ = build_pose(-1.5, -1.4 - 0.5, 0.79, teach_orientation)
        
        fanuc_trays_info_sub = self.create_subscription(Trays, '/fanuc/table_trays_info', self.update_fanuc_trays, 10)
        motoman_trays_info_sub = self.create_subscription(Trays, '/motoman/table_trays_info', self.update_motoman_trays, 10)
        teach_trays_info_sub = self.create_subscription(Trays, '/teach/table_trays_info', self.update_teach_trays, 10)
        
    def update_motoman_trays(self, msg: Trays):
        if self.motoman_trays_spawned:
            return
        
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays # type: ignore
        
        for tray in all_trays:
            world_pose = multiply_pose(self.motoman_vision_pose_, tray.tray_pose.pose)
            
            tray_type = self.tray_types_[tray.identifier - 13]
            tray_color = "black"
            
            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            
            rotation = rad_to_deg(rpy_from_quaternion(world_pose.orientation)[-1])
            
            occupied_slots = []
            for slot in tray.slots:
                slot: SlotInfo
                if slot.occupied:
                    occupied_slots.append("_".join(slot.name.split("_")[-2:]))
            
            self.spawner_node.spawn_tray(tray_type, tray_color, xyz, rotation, occupied_slots)
        self.motoman_trays_spawned = True

        if all(self.trays_spawned):
            self.spawner_node.environment_ready()
    
    def update_fanuc_trays(self, msg: Trays):
        if self.fanuc_trays_spawned:
            return
        
        all_trays: list[Tray] = msg.kit_trays + msg.part_trays # type: ignore
        
        for tray in all_trays:
            world_pose = multiply_pose(self.fanuc_vision_pose_, tray.tray_pose.pose)
            
            tray_type = self.tray_types_[tray.identifier - 13]
            tray_color = "black"
            
            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            
            rotation = rad_to_deg(rpy_from_quaternion(world_pose.orientation)[-1])
            
            occupied_slots = []
            for slot in tray.slots:
                slot: SlotInfo
                if slot.occupied:
                    occupied_slots.append("_".join(slot.name.split("_")[-2:]))
            
            self.spawner_node.spawn_tray(tray_type, tray_color, xyz, rotation, occupied_slots)
        self.fanuc_trays_spawned = True

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
                    occupied_slots.append("_".join(slot.name.split("_")[-2:]))
            
            self.spawner_node.spawn_tray(tray_type, tray_color, xyz, rotation, occupied_slots)
        self.teach_trays_spawned = True

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