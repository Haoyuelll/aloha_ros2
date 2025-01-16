from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declare_arguments = [
        DeclareLaunchArgument('robot_model_master', default_value='wx250s'),
        DeclareLaunchArgument('robot_model_puppet', default_value='vx300s'),
        DeclareLaunchArgument('base_link_master_left', default_value='base_link'),
        DeclareLaunchArgument('base_link_master_right', default_value='base_link'),
        DeclareLaunchArgument('base_link_puppet_left', default_value='base_link'),
        DeclareLaunchArgument('base_link_puppet_right', default_value='base_link'),
        DeclareLaunchArgument('master_modes_left', default_value=PathJoinSubstitution([FindPackageShare('aloha'), 'config', 'master_modes_left.yaml'])),
        DeclareLaunchArgument('puppet_modes_left', default_value=PathJoinSubstitution([FindPackageShare('aloha'), 'config', 'puppet_modes_left.yaml'])),
        DeclareLaunchArgument('master_modes_right', default_value=PathJoinSubstitution([FindPackageShare('aloha'), 'config', 'master_modes_right.yaml'])),
        DeclareLaunchArgument('puppet_modes_right', default_value=PathJoinSubstitution([FindPackageShare('aloha'), 'config', 'puppet_modes_right.yaml'])),
        DeclareLaunchArgument('launch_driver', default_value='true'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('robot_name_master_left', default_value='master_left'),
        DeclareLaunchArgument('robot_name_puppet_left', default_value='puppet_left'),
        DeclareLaunchArgument('robot_name_master_right', default_value='master_right'),
        DeclareLaunchArgument('robot_name_puppet_right', default_value='puppet_right')
    ]
    
    # Stolen from: https://github.com/google-deepmind/mujoco_menagerie/blob/main/aloha/aloha.xml
    # Modified .469->.449
    arm_poses = {
        "robot_name_master_left": ["-0.449", "-0.519", "0.02", '0', '0', '0'],
        "robot_name_master_right": ["0.449", "-0.519", "0.02", '3.1415', '0', '0',],
        "robot_name_puppet_left": ["-0.449", "-0.019", "0.02", '0', '0', '0'],
        "robot_name_puppet_right": ["0.449", "-0.019", "0.02", '3.1415', '0', '0',],
    }
    
    # xsarm_control launch inclusions with conditions
    includes = []
    for side, mode_config, robot_model, robot_name, base_link in [
        ('master_left', 'master_modes_left', 'robot_model_master', 'robot_name_master_left', 'base_link_master_left'),
        ('master_right', 'master_modes_right', 'robot_model_master', 'robot_name_master_right', 'base_link_master_right'),
        ('puppet_left', 'puppet_modes_left', 'robot_model_puppet', 'robot_name_puppet_left', 'base_link_puppet_left'),
        ('puppet_right', 'puppet_modes_right', 'robot_model_puppet', 'robot_name_puppet_right', 'base_link_puppet_right')
    ]:
        includes.append(
            GroupAction([
                # SetLaunchConfiguration("ros_namespace", LaunchConfiguration(robot_name)),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                        FindPackageShare('interbotix_xsarm_control'),
                        'launch',
                        'xsarm_control.launch.py'
                        ])
                    ]),
                    condition=IfCondition(LaunchConfiguration('launch_driver')),
                    launch_arguments={
                        'robot_model': LaunchConfiguration(robot_model),
                        'robot_name': LaunchConfiguration(robot_name),
                        'base_link_frame': LaunchConfiguration(base_link),
                        'use_world_frame': 'false',
                        'use_rviz': 'false',
                        'mode_configs': LaunchConfiguration(mode_config),
                        'use_sim': LaunchConfiguration('use_sim')
                    }.items()
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'{side}_base_transform_broadcaster',
                    arguments=arm_poses[robot_name] + ['/world', [LaunchConfiguration(robot_name), '/base_link']]
                ),
                ]
            )
        )
    return LaunchDescription(declare_arguments + includes)
