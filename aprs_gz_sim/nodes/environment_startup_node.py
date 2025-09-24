#!/usr/bin/env python3

from aprs_gz_sim.evironment_startup import EnvironmentStartup
from time import sleep
from random import randint, random

import rclpy

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    sleep(15)
    colors = ["blue", "green", "red", "purple", "orange", "black"]
    part_type = "m2l1_kit_tray"
    part_color = "black"
    # for i in range(8):
    #     for j in range(25):
    #         startup_node.spawn_gear("medium", colors[randint(0,len(colors)-1)], [1.0 - (0.1 * j), 0.15+(0.1*i), 0.9])
    
    # # sleep(3)

    # startup_node.spawn_sensors("advanced_logical_camera_1", "advanced_logical_camera", [0.0, 0.0, 5.0])
    # startup_node.spawn_sensors("advanced_logical_camera_2", "advanced_logical_camera", [1.0, 0.0, 5.0])

    startup_node.spawn_tray(part_type, part_color, [0.0, 0.25, 0.9], 45, ["lg_1"])

    # part_type = "m2l1_kit"
    # part_color = "black"
    # startup_node.spawn_tray(part_type, part_color, [1.1, -1.375, 1.0], 137)
    
    startup_node.environment_ready()

    try:
        rclpy.spin(startup_node)
    except KeyboardInterrupt:
        startup_node.destroy_node()

if __name__ == "__main__":
    main()