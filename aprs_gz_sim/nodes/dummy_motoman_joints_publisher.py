#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    """
    A simple ROS2 node that publishes a JointState message.
    """
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Declare and initialize the publisher for JointState messages
        self.publisher_ = self.create_publisher(JointState, '/motoman/joint_states', 10)
        
        # Define the joint names for your robot
        self.joint_names = ['joint_b', "joint_e", "joint_l", "joint_r", "joint_s", "joint_t", "joint_u"]
        
        # Initialize joint positions. These will be updated in the timer callback.
        self.joint_positions = [1.57, -1.57, 0.0, 0.0, -1.57, 0.0, 1.57]
        
        # Set the publishing rate to 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('JointState Publisher node has been started.')

    def timer_callback(self):
        """
        Callback function for the timer. This is where the JointState message is
        created and published.
        """
        # Create a new JointState message
        msg = JointState()
        
        # Set the message header timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Assign the joint names to the message
        msg.name = self.joint_names
        
        # Update the joint positions. Here, we'll make them move in a simple sine wave
        # to demonstrate a changing value. In a real application, you would
        # replace this with your actual sensor data or control loop output.
        current_time = self.get_clock().now().nanoseconds / 1e9

        self.joint_positions[-3] += 0.005
        # self.joint_positions[0] = math.sin(current_time)
        # self.joint_positions[1] = math.cos(current_time)
        # self.joint_positions[2] = math.sin(current_time * 0.5)
        
        # Assign the updated positions to the message
        msg.position = self.joint_positions
                
        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    """
    The main entry point for the ROS2 node.
    """
    rclpy.init(args=args)
    
    joint_state_publisher = JointStatePublisher()
    
    # Spin the node to process callbacks
    rclpy.spin(joint_state_publisher)
    
    # Destroy the node explicitly
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()