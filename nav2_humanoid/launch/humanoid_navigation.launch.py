from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    # Path to your custom Nav2 parameters for the humanoid
    humanoid_params_file = os.path.join(
        FindPackageShare('nav2_humanoid').find('nav2_humanoid'),
        'params',
        'humanoid_nav2_params.yaml'
    )

    # Declare arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml',
        default_value='nav2_humanoid/maps/your_map.yaml', # Placeholder map
        description='Full path to map file to load'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=humanoid_params_file,
        description='Full path to the Nav2 parameters file'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the Nav2 stack'
    )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map_yaml'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
    ])

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        bringup_cmd_group
    ])