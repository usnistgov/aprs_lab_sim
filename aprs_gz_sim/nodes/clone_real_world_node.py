#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool

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
        
        vision_quaternion = quaternion_from_euler(0, 0, pi/4)
        vision_orientation = Quaternion()
        vision_orientation.w = vision_quaternion[0]
        vision_orientation.x = vision_quaternion[1]
        vision_orientation.y = vision_quaternion[2]
        vision_orientation.z = vision_quaternion[3]

        self.fanuc_vision_pose_ = build_pose(-1.0, 0.75 - 0.5, 0.9, vision_orientation)
        self.motoman_vision_pose_ = build_pose(0.0, 0.75 - 0.5, 0.9, vision_orientation)
        self.teach_vision_pose_ = build_pose(-1.95505, -2.181225 - 0.5, 0.77, vision_orientation)
        
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