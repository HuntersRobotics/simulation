import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = "hunter_mecanum"
    urdf_name = "mecanum.xacro"

    ld = LaunchDescription()
    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)

    # 正确生成robot_description的方式
    robot_description_content = Command(
        ['xacro ', urdf_model_path]
    )
    # Launch the robot in Gazebo
    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-timeout", "600",
            "-entity", "robot",
            # 这里应该传递解析后的URDF内容，而不是直接传递xacro文件
            "-topic", "/robot_description",
            "-x", "-2.0",
            "-y", "0.0",
            "-Y", "0"
        ],
        output="screen",
    )

    # Start Robot State publisher
    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            'robot_description': robot_description_content
        }],
    )

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld