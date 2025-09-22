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

    # robots=['fanuc', 'franka', 'motoman', 'ur']
    robots=["motoman"]

    for robot in robots:
    # for robot in ["motoman", "fanuc"]:
        urdf = os.path.join(get_package_share_directory('aprs_description'), 'urdf', f'aprs_{robot}.urdf.xacro')
        
        doc = xacro.process_file(urdf)

        robot_description_content = doc.toprettyxml(indent='  ')
        
        # Robot state publisher
        robot_state_publisher_params = {'use_sim_time': True,
                                        'robot_description': robot_description_content}
        robot_state_publishers.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            namespace=f"simulation/{robot}",
            # remappings=[
            #     ("joint_states", "/joint_states")
            # ],
            parameters=[
                robot_state_publisher_params
            ],
        ))
        
        # GZ spawn robot
        robot_spawners.append(Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            # name=f'{robot}_ros_gz_sim',
            arguments=[
                "-string",
                robot_description_content,
                # '-topic', f'simulation/{robot}/robot_description',        
                '-name', f'aprs_{robot}',
                # '-allow_renaming', 'true'
            ]
        ))
        
        # Joint state broadcaster
        joint_state_broadcasters.append(Node(
            package='controller_manager',
            executable='spawner',
            name=f'{robot}_joint_state_broadcaster_spawner',
            arguments = ["joint_state_broadcaster", "-c", f"/simulation/{robot}/controller_manager"],
            parameters=[{"use_sim_time": True}]
        ))
        
        # Joint trajectory controllers
        jt_controller_name = f'joint_trajectory_controller'
        jt_arguments = [jt_controller_name, "-c", f"/simulation/{robot}/controller_manager"]
        if mirror_env and robot in ["fanuc", "motoman"]:
            jt_arguments.append('--inactive')

        joint_trajectory_controllers.append(Node(
            package='controller_manager',
            executable='spawner',
            name=f'{robot}_controller_spawner',
            arguments=jt_arguments,
            parameters=[
                {'use_sim_time': True},
            ],
        ))

        # joint_trajectory_controllers.append(Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     name='controller_spawner',
        #     namespace=f"simulation/{robot}",
        #     arguments=[
        #         'joint_trajectory_controller'
        #     ],
        #     parameters=[
        #         {'use_sim_time': True},
        #     ],
        # ))
        
        if mirror_env and robot in ["fanuc", "motoman"]:
            passthrough_controllers.append(Node(
                package='controller_manager',
                executable='spawner',
                namespace=f"simulation/{robot}",
                arguments=[
                    'passthrough_controller',
                ],
                parameters=[
                    {'use_sim_time': True},
                ],
            ))

    nodes_to_start = [
        *robot_state_publishers,
        *robot_spawners,
        *joint_state_broadcasters,
        *passthrough_controllers,
        *joint_trajectory_controllers,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("mirror_env", default_value="false", description="Whether or not to mirror real robots")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])