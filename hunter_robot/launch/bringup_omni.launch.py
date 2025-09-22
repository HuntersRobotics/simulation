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
            'z': '2',
            'yaw': '0.0',
            'world_path': 'RMUC2024_world/RMUC2024_world.world'
        },
        WorldType.RMUL: {
            'x': '4.3',
            'y': '3.35',
            'z': '2',
            'yaw': '0.0',
            'world_path': 'RMUL2024_world/RMUL2024_world.world'
            # 'world_path': 'RMUL2024_world/RMUL2024_world_dynamic_obstacles.world'
        },
        WorldType.WAREHOUSE:{
            'x': '0',
            'y': '0',
            'z': '0.25',
            'yaw': '0.0',
            'world_path': 'small_warehouse.world'
        }
    }
    return world_configs.get(world_type, None)


def generate_launch_description():
    package_name = "hunter_robot"
    urdf_name = "omni.xacro"

    ld = LaunchDescription()
    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)
    # 正确生成robot_description的方式
    default_robot_description = Command(
        ['xacro ', urdf_model_path]
    )

    new_model_path = pkg_share + '/models'
    print(f"new model path  = {new_model_path}")

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = new_model_path + ':' + os.environ['GAZEBO_MODEL_PATH']
    else:
        os.environ['GAZEBO_MODEL_PATH'] = new_model_path

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description = LaunchConfiguration('robot_description')

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

    declare_robot_description_cmd = DeclareLaunchArgument(
        'robot_description',
        default_value=default_robot_description,
        description='Robot description'
    )




    # Launch the robot in Gazebo
    def create_robot(world_type):
        world_config = get_world_config(world_type)
        print(f'world type is {world_type}')
        if world_config is Node:
            return
        print(f'world type is {world_type}')
        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-timeout', "600",
                        '-entity', 'robot',
                        '-topic', 'robot_description',
                        '-x', world_config['x'],
                        '-y', world_config['y'],
                        '-z', world_config['z'],
                        '-Y', world_config['yaw']
                    ],
                    output='screen',
                ),
            ]    
        )
    # spawn_entity_cmd = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=[
    #         "-timeout", "600",
    #         "-entity", "robot",
    #         # 这里应该传递解析后的URDF内容，而不是直接传递xacro文件
    #         "-topic", "/robot_description",
    #         "-x", "-2.0",
    #         "-y", "0.0",
    #         "-Y", "0"
    #     ],
    #     output="screen",
    # )

    # Start Robot State publisher
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='omni_robot_state_publisher',
        # arguments=[urdf_model_path]
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='omni_joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
        output='screen'
    )
    bringup_RMUC = create_robot(WorldType.RMUC)
    bringup_RMUL = create_robot(WorldType.RMUL)
    bringup_WAREHOUSE = create_robot(WorldType.WAREHOUSE)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(bringup_RMUC)
    ld.add_action(bringup_RMUL)
    ld.add_action(bringup_WAREHOUSE)

    return ld