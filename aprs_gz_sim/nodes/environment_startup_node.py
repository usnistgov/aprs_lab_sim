#!/usr/bin/env python3

from aprs_gz_sim.environment_startup import EnvironmentStartup
from time import sleep

import rclpy

def main():
    """Small script to exercise the EnvironmentStartup helper.

    This module is a convenience entry point used in tests or manual runs to
    spawn a few demo parts and mark the simulated environment as ready.
    """
    rclpy.init()

    startup_node = EnvironmentStartup()

    sleep(15)
    colors = ["blue", "green", "red", "purple", "orange", "black"]
    part_type = "m2l1_kit_tray"
    part_color = "black"

    startup_node.spawn_tray(part_type+"_01", part_type, part_color, [0.0, 0.25, 0.9], 45)

    startup_node.environment_ready()

    try:
        rclpy.spin(startup_node)
    except KeyboardInterrupt:
        startup_node.destroy_node()

if __name__ == "__main__":
    main()