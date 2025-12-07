from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the share directory for the current package
    # In a real scenario, this would be your Gazebo package
    # For now, let's assume models and worlds are directly in the current workspace's gazebo folder
    # Or define a custom path
    
    # Path to the world file
    world_file_name = 'empty_world.world'
    world_path = os.path.join(
        get_package_share_directory('gazebo_ros'), # Assuming gazebo_ros is installed
        'worlds',
        world_file_name
    )
    
    # Path to the model SDF file
    humanoid_model_path = os.path.join(
        os.getcwd(), # Current working directory
        'gazebo', 'models', 'humanoid_robot', 'model.sdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=TextSubstitution(text=world_path),
            description='Full path to world model file to load'
        ),
        
        # Launch Gazebo
        Node(
            package='gazebo_ros',
            executable='gazebo_node.py',
            arguments=['-s', 'libgazebo_ros_factory.so',
                       LaunchConfiguration('world')],
            output='screen',
        ),
        
        # Spawn the humanoid robot
        # Note: This is a simplified spawn. For URDF, you'd typically use robot_state_publisher
        # and gazebo_ros_pkgs robot_state_publisher + spawn_entity.
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid_robot',
                       '-file', humanoid_model_path,
                       '-x', '0.0', '-y', '0.0', '-z', '1.0'],
            output='screen',
        )
    ])