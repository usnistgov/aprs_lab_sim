import os
import yaml
import xacro
import rclpy.logging

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
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
    mirror_env = LaunchConfiguration("mirror_env").perform(context).lower() == "true"

    robot_state_publishers = []
    robot_spawners = []
    joint_state_broadcasters = []
    joint_trajectory_controllers = []
    passthrough_controllers = []

    robots = ['fanuc', 'franka', 'motoman', 'ur']

    for robot in robots:
        urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', f'aprs_{robot}.urdf.xacro')

        doc = xacro.process_file(urdf)

        robot_description_content = doc.toprettyxml(indent='  ') # type: ignore

        robot_state_publisher_params = {'use_sim_time': True,
                                        'robot_description': robot_description_content,
                                        'frame_prefix': 'sim/'}
        robot_state_publishers.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            namespace=f"simulation/{robot}",
            parameters=[
                robot_state_publisher_params
            ],
        ))

        robot_spawners.append(Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                "-string",
                robot_description_content,
                '-name', f'aprs_{robot}',
            ]
        ))

    controller_loader_node = Node(
        package="aprs_gz_sim",
        executable="seperate_load_controllers.py",
        output="screen",
        parameters=[{'use_sim_time': True, 'mirror_env': bool(mirror_env)}]
    )

    nodes_to_start = [
        *robot_state_publishers,
        *robot_spawners,
        *joint_state_broadcasters,
        *passthrough_controllers,
        *joint_trajectory_controllers,
        controller_loader_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("mirror_env", default_value="false", description="Whether or not to mirror real robots")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])