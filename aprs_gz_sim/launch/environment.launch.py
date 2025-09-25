import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    world_path = os.path.join(get_package_share_directory('aprs_gz_sim'), 'worlds', 'lab.sdf')
    
    use_seperate_descriptions = LaunchConfiguration("use_seperate_descriptions")
    mirror_env = LaunchConfiguration("mirror_env")
    
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v4 '+ world_path])
        ]
    )
    
    spawn_part_node = Node(
        package='aprs_gz_sim',
        executable='spawn_part'
    )
    
    
    environment_startup_node = Node(
        package='aprs_gz_sim',
        executable='environment_startup_node.py'
    )
    
    combined_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('aprs_gz_sim'),'launch', 'combined_description.launch.py')]
        ),
        condition=UnlessCondition(use_seperate_descriptions)
    )
    
    seperate_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('aprs_gz_sim'),'launch', 'seperate_description.launch.py')]
        ),
        condition=IfCondition(use_seperate_descriptions),
        launch_arguments=[
            ("mirror_env", mirror_env)
        ]
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    clone_real_world_node = Node(
        package='aprs_gz_sim',
        executable='clone_real_world_node.py',
        output="screen"
    )
    
    return [
        gz,
        combined_robots,
        seperate_robots,
        clone_real_world_node,
        # spawn_part_node,
        # environment_startup_node,
        gz_sim_bridge
    ]
    
def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("use_seperate_descriptions", default_value="false", description="use seperate robot descriptions")
    )

    declared_arguments.append(
        DeclareLaunchArgument("mirror_env", default_value="false", description="Whether or not to mirror real robots")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
