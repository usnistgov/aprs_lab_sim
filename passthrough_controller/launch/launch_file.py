#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description(context, *args, **kwargs):
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('passthrough_controller'),
            'config',
            'passthrough_controller.yaml'
        ]),
        description='Path to controller configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    robot_description_arg = DeclareLaunchArgument(
        'robot_description_file',
        description='Path to robot URDF file'
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_file = LaunchConfiguration('robot_description_file')

    # Load robot description
    with open(robot_description_file.perform(context)) as infp:
        robot_desc = infp.read()

    # Robot state publisher (for simulation)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Gazebo spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simulated_robot'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='both'
    )

    # Load and activate the passthrough controller
    load_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['passthrough_controller'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        robot_description_arg,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        load_controller
    ])