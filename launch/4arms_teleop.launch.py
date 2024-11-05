from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
        DeclareLaunchArgument('base_link_master', default_value='base_link'),
        DeclareLaunchArgument('base_link_puppet', default_value='base_link'),
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

    # xsarm_control launch inclusions with conditions
    includes = []
    for side, mode_config, robot_model, robot_name, base_link in [
        ('master_left', 'master_modes_left', 'robot_model_master', 'robot_name_master_left', 'base_link_master'),
        ('master_right', 'master_modes_right', 'robot_model_master', 'robot_name_master_right', 'base_link_master'),
        ('puppet_left', 'puppet_modes_left', 'robot_model_puppet', 'robot_name_puppet_left', 'base_link_puppet'),
        ('puppet_right', 'puppet_modes_right', 'robot_model_puppet', 'robot_name_puppet_right', 'base_link_puppet')
    ]:
        includes.append(
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
            )
        )

    # Transform broadcasters
    transform_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='master_left_transform_broadcaster',
            arguments=['0', '-0.25', '0', '0', '0', '0', '/world', f'/{LaunchConfiguration("robot_name_master_left")}/base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='master_right_transform_broadcaster',
            arguments=['0', '-0.25', '0', '0', '0', '0', '/world', f'/{LaunchConfiguration("robot_name_master_right")}/base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='puppet_left_transform_broadcaster',
            arguments=['0', '0.25', '0', '0', '0', '0', '/world', f'/{LaunchConfiguration("robot_name_puppet_left")}/base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='puppet_right_transform_broadcaster',
            arguments=['0', '0.25', '0', '0', '0', '0', '/world', f'/{LaunchConfiguration("robot_name_puppet_right")}/base_link']
        )
    ]

    # USB camera nodes
    # usb_cam_nodes = [
    #     Node(
    #         package='usb_cam',
    #         executable='usb_cam_node',
    #         name='usb_cam_high',
    #         output='screen',
    #         parameters=[{
    #             'video_device': '/dev/CAM_HIGH',
    #             'framerate': 60,
    #             'image_width': 640,
    #             'image_height': 480,
    #             'pixel_format': 'yuyv',
    #             'camera_frame_id': 'usb_cam',
    #             'io_method': 'mmap',
    #             'autofocus': False,
    #             'focus': 5,
    #             'autoexposure': True
    #         }]
    #     ),
    #     Node(
    #         package='usb_cam',
    #         executable='usb_cam_node',
    #         name='usb_cam_low',
    #         output='screen',
    #         parameters=[{
    #             'video_device': '/dev/CAM_LOW',
    #             'framerate': 60,
    #             'image_width': 640,
    #             'image_height': 480,
    #             'pixel_format': 'yuyv',
    #             'camera_frame_id': 'usb_cam',
    #             'io_method': 'mmap',
    #             'autofocus': False,
    #             'focus': 35,
    #             'autoexposure': True
    #         }]
    #     ),
    #     Node(
    #         package='usb_cam',
    #         executable='usb_cam_node',
    #         name='usb_cam_left_wrist',
    #         output='screen',
    #         parameters=[{
    #             'video_device': '/dev/CAM_LEFT_WRIST',
    #             'framerate': 60,
    #             'image_width': 640,
    #             'image_height': 480,
    #             'pixel_format': 'yuyv',
    #             'camera_frame_id': 'usb_cam',
    #             'io_method': 'mmap',
    #             'autofocus': False,
    #             'focus': 40,
    #             'autoexposure': True
    #         }]
    #     ),
    #     Node(
    #         package='usb_cam',
    #         executable='usb_cam_node',
    #         name='usb_cam_right_wrist',
    #         output='screen',
    #         parameters=[{
    #             'video_device': '/dev/CAM_RIGHT_WRIST',
    #             'framerate': 60,
    #             'image_width': 640,
    #             'image_height': 480,
    #             'pixel_format': 'yuyv',
    #             'camera_frame_id': 'usb_cam',
    #             'io_method': 'mmap',
    #             'autofocus': False,
    #             'focus': 40,
    #             'autoexposure': True
    #         }]
    #     )
    # ]

    # return LaunchDescription(declare_arguments + includes + transform_nodes + usb_cam_nodes)
    return LaunchDescription(declare_arguments + includes + transform_nodes)


"""
ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
generate-parameter-library-py 0.3.8 requires pyyaml, which is not installed.
generate-parameter-library-py 0.3.8 requires typeguard, which is not installed.
black 24.10.0 requires click>=8.0.0, which is not installed.
black 24.10.0 requires platformdirs>=2, which is not installed.

ERROR: pip's dependency resolver does not currently take into account all the packages that are installed. This behaviour is the source of the following dependency conflicts.
generate-parameter-library-py 0.3.8 requires typeguard, which is not installed.

"""