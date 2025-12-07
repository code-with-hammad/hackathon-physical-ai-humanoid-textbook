from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo/Isaac Sim) clock if true'
    )

    # Paths to your package directories
    openai_whisper_pkg_dir = FindPackageShare('openai_whisper').find('openai_whisper')
    llm_planning_pkg_dir = FindPackageShare('llm_planning').find('llm_planning')
    humanoid_vla_pkg_dir = FindPackageShare('humanoid_vla').find('humanoid_vla')

    # Launch the Whisper Node (speech-to-text)
    whisper_node = Node(
        package='openai_whisper',
        executable='whisper_node.py',
        name='whisper_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Launch the Action Mapper Node (direct action mapping for simple commands)
    action_mapper_node = Node(
        package='openai_whisper',
        executable='action_mapper_node.py',
        name='action_mapper_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Launch the LLM Planner Node
    llm_planner_node = Node(
        package='llm_planning',
        executable='llm_planner_node.py',
        name='llm_planner_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            PathJoinSubstitution([llm_planning_pkg_dir, 'config', 'llm_config.yaml'])
        ],
    )

    # Launch the Perception Simulator (to provide dummy perception feedback to LLM)
    perception_simulator_node = Node(
        package='humanoid_vla',
        executable='perception_simulator.py',
        name='perception_simulator_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Launch the Action Executor Node (translates high-level actions to robot commands)
    action_executor_node = Node(
        package='humanoid_vla',
        executable='action_executor_node.py',
        name='action_executor_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    # Placeholder for launching robot simulation (e.g., Isaac Sim or Gazebo)
    # This would typically be another IncludeLaunchDescription, assuming a package like
    # 'humanoid_sim' exists and has a launch file for the humanoid robot.
    # robot_sim_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('humanoid_sim').find('humanoid_sim'),
    #             'launch',
    #             'humanoid_robot_sim.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    # )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        
        # Group VLA nodes
        GroupAction([
            whisper_node,
            action_mapper_node,
            llm_planner_node,
            perception_simulator_node,
            action_executor_node,
            # robot_sim_launch # Uncomment and define if a separate robot simulation launch is available
        ])
    ])