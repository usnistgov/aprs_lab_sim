#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from aprs_interfaces.msg import Trays, Tray, SlotInfo
import json

class TrayPublisherNode(Node):
    """
    A ROS 2 node that publishes aprs_interfaces/Trays messages
    on three different topics for FANUC, MOTOMAN, and TEACH stations.
    """

    def __init__(self):
        super().__init__('tray_publisher_node')
        self.get_logger().info("Tray Publisher Node has been started.")

        # --- Publishers ---
        self.fanuc_publisher = self.create_publisher(Trays, '/fanuc/table_trays_info', 10)
        self.motoman_publisher = self.create_publisher(Trays, '/motoman/table_trays_info', 10)
        self.teach_publisher = self.create_publisher(Trays, '/teach/table_trays_info', 10)

        # --- Data to Publish ---
        # The provided data is represented as a Python dictionary.
        # This makes the data structure clear and easy to manage.
        self.fanuc_data = {
            "kit_trays": [
                {
                    "identifier": 16,
                    "name": "m2l1_kit_tray_02",
                    "tray_pose": {
                        "header": {"frame_id": "fanuc_table_image"},
                        "pose": {
                            "position": {"x": 0.3932457, "y": 0.3449895, "z": -0.015},
                            "orientation": {"x": 0.9990319939885262, "y": 0.04398948723626484, "z": -6.117306668419231e-17, "w": 2.693579237001254e-18}
                        }
                    },
                    "slots": [
                        {
                            "occupied": True, "size": 3, "name": "m2l1_kit_tray_02_lg_1",
                            "slot_pose": {
                                "header": {"frame_id": "m2l1_kit_tray_02"},
                                "pose": {"position": {"x": 0.0, "y": -0.075, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 2, "name": "m2l1_kit_tray_02_mg_1",
                            "slot_pose": {
                                "header": {"frame_id": "m2l1_kit_tray_02"},
                                "pose": {"position": {"x": -0.065, "y": 0.0, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 2, "name": "m2l1_kit_tray_02_mg_2",
                            "slot_pose": {
                                "header": {"frame_id": "m2l1_kit_tray_02"},
                                "pose": {"position": {"x": 0.065, "y": 0.0, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        }
                    ]
                },
                {
                    "identifier": 16,
                    "name": "m2l1_kit_tray_05",
                    "tray_pose": {
                        "header": {"frame_id": "fanuc_table_image"},
                        "pose": {
                            "position": {"x": 0.14561520000000003, "y": 0.33356040000000003, "z": -0.015},
                            "orientation": {"x": -1.0, "y": 6.123233995736766e-17, "z": 6.123233995736766e-17, "w": 3.749399456654644e-33}
                        }
                    },
                    "slots": [
                        {
                            "occupied": False, "size": 3, "name": "m2l1_kit_tray_05_lg_1",
                            "slot_pose": {
                                "header": {"frame_id": "m2l1_kit_tray_05"},
                                "pose": {"position": {"x": 0.0, "y": -0.075, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": False, "size": 2, "name": "m2l1_kit_tray_05_mg_1",
                            "slot_pose": {
                                "header": {"frame_id": "m2l1_kit_tray_05"},
                                "pose": {"position": {"x": -0.065, "y": 0.0, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": False, "size": 2, "name": "m2l1_kit_tray_05_mg_2",
                            "slot_pose": {
                                "header": {"frame_id": "m2l1_kit_tray_05"},
                                "pose": {"position": {"x": 0.065, "y": 0.0, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        }
                    ]
                }
            ],
            "part_trays": [
                {
                    "identifier": 14,
                    "name": "medium_gear_tray_03",
                    "tray_pose": {
                        "header": {"frame_id": "fanuc_table_image"},
                        "pose": {
                            "position": {"x": 0.406368, "y": 0.1591608, "z": -0.015},
                            "orientation": {"x": 0.9998919327768259, "y": 0.014701114509568961, "z": -6.122572274842001e-17, "w": 9.00183641402117e-19}
                        }
                    },
                    "slots": [
                        {
                            "occupied": False, "size": 2, "name": "medium_gear_tray_03_slot_1",
                            "slot_pose": {
                                "header": {"frame_id": "medium_gear_tray_03"},
                                "pose": {"position": {"x": -0.05, "y": 0.05, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": False, "size": 2, "name": "medium_gear_tray_03_slot_2",
                            "slot_pose": {
                                "header": {"frame_id": "medium_gear_tray_03"},
                                "pose": {"position": {"x": 0.05, "y": 0.05, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 2, "name": "medium_gear_tray_03_slot_3",
                            "slot_pose": {
                                "header": {"frame_id": "medium_gear_tray_03"},
                                "pose": {"position": {"x": -0.05, "y": -0.05, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 2, "name": "medium_gear_tray_03_slot_4",
                            "slot_pose": {
                                "header": {"frame_id": "medium_gear_tray_03"},
                                "pose": {"position": {"x": 0.05, "y": -0.05, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        }
                    ]
                },
                {
                    "identifier": 15,
                    "name": "large_gear_tray_04",
                    "tray_pose": {
                        "header": {"frame_id": "fanuc_table_image"},
                        "pose": {
                            "position": {"x": 0.18879179999999998, "y": 0.1815957, "z": -0.015},
                            "orientation": {"x": 0.9998919327768259, "y": 0.014701114509568961, "z": -6.122572274842001e-17, "w": 9.00183641402117e-19}
                        }
                    },
                    "slots": [
                        {
                            "occupied": False, "size": 3, "name": "large_gear_tray_04_slot_1",
                            "slot_pose": {
                                "header": {"frame_id": "large_gear_tray_04"},
                                "pose": {"position": {"x": -0.052, "y": 0.06, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 3, "name": "large_gear_tray_04_slot_2",
                            "slot_pose": {
                                "header": {"frame_id": "large_gear_tray_04"},
                                "pose": {"position": {"x": 0.052, "y": 0.06, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        }
                    ]
                }
            ]
        }
        
        self.motoman_data = {
            "kit_trays": [
                {
                    "identifier": 17,
                    "name": "s2l2_kit_tray_05",
                    "tray_pose": {
                        "header": {"frame_id": "motoman_table_image"},
                        "pose": {
                            "position": {"x": 0.3631914, "y": 0.17820930000000001, "z": -0.015},
                            "orientation": {"x": -1.0, "y": 6.123233995736766e-17, "z": 6.123233995736766e-17, "w": 3.749399456654644e-33}
                        }
                    },
                    "slots": [
                        {
                            "occupied": True, "size": 3, "name": "s2l2_kit_tray_05_lg_1",
                            "slot_pose": {
                                "header": {"frame_id": "s2l2_kit_tray_05"},
                                "pose": {"position": {"x": -0.052, "y": 0.06, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 3, "name": "s2l2_kit_tray_05_lg_2",
                            "slot_pose": {
                                "header": {"frame_id": "s2l2_kit_tray_05"},
                                "pose": {"position": {"x": 0.052, "y": 0.06, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 1, "name": "s2l2_kit_tray_05_sg_1",
                            "slot_pose": {
                                "header": {"frame_id": "s2l2_kit_tray_05"},
                                "pose": {"position": {"x": -0.045, "y": -0.045, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 1, "name": "s2l2_kit_tray_05_sg_2",
                            "slot_pose": {
                                "header": {"frame_id": "s2l2_kit_tray_05"},
                                "pose": {"position": {"x": 0.045, "y": -0.045, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        }
                    ]
                }
            ],
            "part_trays": [
                {
                    "identifier": 13,
                    "name": "small_gear_tray_01",
                    "tray_pose": {
                        "header": {"frame_id": "motoman_table_image"},
                        "pose": {
                            "position": {"x": 0.37462049999999997, "y": 0.3784302, "z": -0.015},
                            "orientation": {"x": 0.12218326369570447, "y": 0.992507556682903, "z": -7.481567139716074e-18, "w": 6.077356012106387e-17}
                        }
                    },
                    "slots": [
                        {
                            "occupied": False, "size": 1, "name": "small_gear_tray_01_slot_1",
                            "slot_pose": {
                                "header": {"frame_id": "small_gear_tray_01"},
                                "pose": {"position": {"x": -0.045, "y": 0.045, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": False, "size": 1, "name": "small_gear_tray_01_slot_2",
                            "slot_pose": {
                                "header": {"frame_id": "small_gear_tray_01"},
                                "pose": {"position": {"x": 0.045, "y": 0.045, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 1, "name": "small_gear_tray_01_slot_3",
                            "slot_pose": {
                                "header": {"frame_id": "small_gear_tray_01"},
                                "pose": {"position": {"x": -0.045, "y": -0.045, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 1, "name": "small_gear_tray_01_slot_4",
                            "slot_pose": {
                                "header": {"frame_id": "small_gear_tray_01"},
                                "pose": {"position": {"x": 0.045, "y": -0.045, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        }
                    ]
                },
                {
                    "identifier": 15,
                    "name": "large_gear_tray_03",
                    "tray_pose": {
                        "header": {"frame_id": "motoman_table_image"},
                        "pose": {
                            "position": {"x": 0.20106749999999998, "y": 0.37462049999999997, "z": -0.015},
                            "orientation": {"x": -0.7171331026730243, "y": 0.6969362331308091, "z": 4.391173793755647e-17, "w": 4.2675036355672945e-17}
                        }
                    },
                    "slots": [
                        {
                            "occupied": False, "size": 3, "name": "large_gear_tray_03_slot_1",
                            "slot_pose": {
                                "header": {"frame_id": "large_gear_tray_03"},
                                "pose": {"position": {"x": -0.052, "y": 0.06, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": False, "size": 3, "name": "large_gear_tray_03_slot_2",
                            "slot_pose": {
                                "header": {"frame_id": "large_gear_tray_03"},
                                "pose": {"position": {"x": 0.052, "y": 0.06, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        }
                    ]
                }
            ]
        }
        
        self.teach_data = {
            "kit_trays": [
                {
                    "identifier": 17,
                    "name": "s2l2_kit_tray_05",
                    "tray_pose": {
                        "header": {"frame_id": ""},
                        "pose": {
                            "position": {"x": 0.359805, "y": 0.4017117, "z": -0.015},
                            "orientation": {"x": -0.9998919327768259, "y": 0.014701114509568961, "z": 6.122572274842001e-17, "w": 9.00183641402117e-19}
                        }
                    },
                    "slots": [
                        {
                            "occupied": True, "size": 3, "name": "s2l2_kit_tray_05_lg_1",
                            "slot_pose": {
                                "header": {"frame_id": "s2l2_kit_tray_05"},
                                "pose": {"position": {"x": -0.052, "y": 0.06, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 3, "name": "s2l2_kit_tray_05_lg_2",
                            "slot_pose": {
                                "header": {"frame_id": "s2l2_kit_tray_05"},
                                "pose": {"position": {"x": 0.052, "y": 0.06, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": False, "size": 1, "name": "s2l2_kit_tray_05_sg_1",
                            "slot_pose": {
                                "header": {"frame_id": "s2l2_kit_tray_05"},
                                "pose": {"position": {"x": -0.045, "y": -0.045, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": False, "size": 1, "name": "s2l2_kit_tray_05_sg_2",
                            "slot_pose": {
                                "header": {"frame_id": "s2l2_kit_tray_05"},
                                "pose": {"position": {"x": 0.045, "y": -0.045, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        }
                    ]
                },
                {
                    "identifier": 16,
                    "name": "m2l1_kit_tray_02",
                    "tray_pose": {
                        "header": {"frame_id": ""},
                        "pose": {
                            "position": {"x": 0.3788535, "y": 0.1265667, "z": -0.015},
                            "orientation": {"x": 0.9996150141943179, "y": 0.027745691508661203, "z": -6.120876637563537e-17, "w": 1.698933614810592e-18}
                        }
                    },
                    "slots": [
                        {
                            "occupied": True, "size": 3, "name": "m2l1_kit_tray_02_lg_1",
                            "slot_pose": {
                                "header": {"frame_id": "m2l1_kit_tray_02"},
                                "pose": {"position": {"x": 0.0, "y": -0.075, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 2, "name": "m2l1_kit_tray_02_mg_1",
                            "slot_pose": {
                                "header": {"frame_id": "m2l1_kit_tray_02"},
                                "pose": {"position": {"x": -0.065, "y": 0.0, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        },
                        {
                            "occupied": True, "size": 2, "name": "m2l1_kit_tray_02_mg_2",
                            "slot_pose": {
                                "header": {"frame_id": "m2l1_kit_tray_02"},
                                "pose": {"position": {"x": 0.065, "y": 0.0, "z": 0.016}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                            }
                        }
                    ]
                }
            ],
            "part_trays": []
        }
        
        # Create a timer to publish the messages at a regular interval (e.g., 1 Hz)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def create_slot_info(self, slot_data):
        """Helper function to create a SlotInfo message from a dictionary."""
        slot = SlotInfo()
        slot.occupied = slot_data["occupied"]
        slot.size = slot_data["size"]
        slot.name = slot_data["name"]

        slot.slot_pose = PoseStamped()
        slot.slot_pose.header.stamp = self.get_clock().now().to_msg()
        slot.slot_pose.header.frame_id = slot_data["slot_pose"]["header"]["frame_id"]
        slot.slot_pose.pose.position.x = float(slot_data["slot_pose"]["pose"]["position"]["x"])
        slot.slot_pose.pose.position.y = float(slot_data["slot_pose"]["pose"]["position"]["y"])
        slot.slot_pose.pose.position.z = float(slot_data["slot_pose"]["pose"]["position"]["z"])
        slot.slot_pose.pose.orientation.x = float(slot_data["slot_pose"]["pose"]["orientation"]["x"])
        slot.slot_pose.pose.orientation.y = float(slot_data["slot_pose"]["pose"]["orientation"]["y"])
        slot.slot_pose.pose.orientation.z = float(slot_data["slot_pose"]["pose"]["orientation"]["z"])
        slot.slot_pose.pose.orientation.w = float(slot_data["slot_pose"]["pose"]["orientation"]["w"])

        return slot

    def create_tray_msg(self, tray_data):
        """Helper function to create a Tray message from a dictionary."""
        tray = Tray()
        tray.identifier = tray_data["identifier"]
        tray.name = tray_data["name"]

        tray.tray_pose = PoseStamped()
        tray.tray_pose.header.stamp = self.get_clock().now().to_msg()
        tray.tray_pose.header.frame_id = tray_data["tray_pose"]["header"]["frame_id"]
        tray.tray_pose.pose.position.x = float(tray_data["tray_pose"]["pose"]["position"]["x"])
        tray.tray_pose.pose.position.y = float(tray_data["tray_pose"]["pose"]["position"]["y"])
        tray.tray_pose.pose.position.z = float(tray_data["tray_pose"]["pose"]["position"]["z"])
        tray.tray_pose.pose.orientation.x = float(tray_data["tray_pose"]["pose"]["orientation"]["x"])
        tray.tray_pose.pose.orientation.y = float(tray_data["tray_pose"]["pose"]["orientation"]["y"])
        tray.tray_pose.pose.orientation.z = float(tray_data["tray_pose"]["pose"]["orientation"]["z"])
        tray.tray_pose.pose.orientation.w = float(tray_data["tray_pose"]["pose"]["orientation"]["w"])
        
        tray.slots = [self.create_slot_info(slot) for slot in tray_data["slots"]]

        return tray

    def create_trays_msg(self, data):
        """Helper function to create the main Trays message."""
        trays_msg = Trays()
        trays_msg.kit_trays = [self.create_tray_msg(tray) for tray in data["kit_trays"]]
        trays_msg.part_trays = [self.create_tray_msg(tray) for tray in data["part_trays"]]
        return trays_msg

    def timer_callback(self):
        """
        Callback function that is executed by the timer.
        It creates and publishes the three Trays messages.
        """
        # Create the messages
        fanuc_trays_msg = self.create_trays_msg(self.fanuc_data)
        motoman_trays_msg = self.create_trays_msg(self.motoman_data)
        teach_trays_msg = self.create_trays_msg(self.teach_data)

        # Publish the messages
        self.fanuc_publisher.publish(fanuc_trays_msg)
        self.motoman_publisher.publish(motoman_trays_msg)
        self.teach_publisher.publish(teach_trays_msg)
        
def main(args=None):
    """
    Main function for the ROS node.
    """
    rclpy.init(args=args)
    tray_publisher_node = TrayPublisherNode()
    rclpy.spin(tray_publisher_node)
    tray_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
