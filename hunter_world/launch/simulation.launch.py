#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.actions.append_environment_variable import AppendEnvironmentVariable

# Enum for world types
class WorldType:
    RMUC = 'RMUC'
    RMUL = 'RMUL'
    WAREHOUSE = 'WAREHOUSE'

def get_world_config(world_type):
    world_configs = {
        WorldType.RMUC: {
            'x': '6.35',
            'y': '7.6',
            'z': '0.2',
            'yaw': '0.0',
            'world_path': 'RMUC2024_world/RMUC2024_world.world'
        },
        WorldType.RMUL: {
            'x': '4.3',
            'y': '3.35',
            'z': '1.16',
            'yaw': '0.0',
            'world_path': 'RMUL2024_world/RMUL2024_world.world'
            # 'world_path': 'RMUL2024_world/RMUL2024_world_dynamic_obstacles.world'
        },
        WorldType.WAREHOUSE:{
            'world_path': 'small_warehouse.world'
        }
    }
    return world_configs.get(world_type, None)

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('hunter_world')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')


    new_model_path = bringup_dir + '/meshes'
    print(f"new model path  = {new_model_path}")

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = new_model_path + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        os.environ['GAZEBO_MODEL_PATH'] = new_model_path
    
    # Set Gazebo plugin path
    append_enviroment = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(os.path.join(bringup_dir, 'meshes', 'obstacles', 'obstacle_plugin', 'lib'))
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.WAREHOUSE,
        description='Choose <RMUC>, <WAREHOUSE> or <RMUL>'
    )

    # Specify the actions
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    def create_gazebo_launch_group(world_type):
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={'world': os.path.join(bringup_dir, 'world', world_config['world_path']),
                    'verbose': 'true'}.items(),
                )
            ]
        )

    bringup_RMUC_cmd_group = create_gazebo_launch_group(WorldType.RMUC)
    bringup_RMUL_cmd_group = create_gazebo_launch_group(WorldType.RMUL)
    bringup_WAREHOUSE_cmd_group = create_gazebo_launch_group(WorldType.WAREHOUSE)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(append_enviroment)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(gazebo_client_launch)
    ld.add_action(bringup_RMUC_cmd_group) # type: ignore
    ld.add_action(bringup_RMUL_cmd_group) # type: ignore
    ld.add_action(bringup_WAREHOUSE_cmd_group) # type: ignore

    return ld
