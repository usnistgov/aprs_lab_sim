import os
import yaml
import xacro
import rclpy.logging
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
)

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def read_yaml(path):
    with open(path, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError:
            print("Unable to read configuration file")
            return {} 

def launch_setup(context, *args, **kwargs):
    # Get robot description

    urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', 'aprs_lab_robots.urdf.xacro')
    
    doc = xacro.process_file(urdf)

    robot_description_content = doc.toprettyxml(indent='  ')
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        namespace="simulation",
        parameters=[
            {'use_sim_time': True}, 
            {'robot_description': robot_description_content}
        ],
    )
    
    # GZ spawn robot
    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'simulation/robot_description',
                '-name', 'aprs_robots',
                '-allow_renaming', 'true'],
    )
    
    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster'],
        namespace="simulation",
        parameters=[
            {'use_sim_time': True},
        ],
    )
    
    # robot switcher
    controller_switcher = Node(
        package='aprs_gz_sim',
        executable='combined_controller_switcher_node.py',
        output='screen'
    )
    
    #Joint trajectory controllers
    joint_trajectory_controllers = []
    for robot in ['fanuc', 'franka', 'motoman', 'ur']:
        joint_trajectory_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            namespace="simulation",
            name=f'{robot}_joint_trajectory_controller_spawner',
            arguments=[
                f'{robot}_joint_trajectory_controller', '--inactive'
            ],
            parameters=[
                {'use_sim_time': True},
            ],
        ))

    nodes_to_start = [
        robot_state_publisher_node,
        gz_spawn_robot,
        controller_switcher,
        joint_state_broadcaster,
        *joint_trajectory_controllers
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
