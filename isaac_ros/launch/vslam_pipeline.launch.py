from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    # This is a placeholder. In a real scenario, you would dynamically
    # find the package share directory for isaac_ros_visual_slam or similar.
    # For now, we assume standard ROS 2 package finding or direct path.

    # Example of how to define VSLAM node with arguments
    vslam_node = Node(
        package='isaac_ros_visual_slam', # Assuming this package exists
        executable='isaac_ros_visual_slam_node',
        name='visual_slam_node',
        output='screen',
        parameters=[{
            'debug_mode': False,
            'enable_slam': True,
            'enable_localization': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'camera_frame': 'camera_link',
            'gyro_frame': 'imu_link',
            'denoise_input_images': True,
            'rectify_input_images': True,
            'publish_tf': True,
            'publish_pose': True,
            'publish_landmarks': False,
            'publish_graph': False,
            'publish_map_pose_over_ros': True,
            'use_imu_fusion': True,
            'use_image_transport': True,
            'input_image_left_topic': '/camera/image_raw', # Matches publisher
            'input_imu_topic': '/imu/data', # Matches publisher
            'set_from_odometry': False,
            'path_odometry_topic': 'odometry/slam',
            'path_pose_topic': 'slam/camera_pose',
            'path_odom_reset_distance': 1.0,
            'enable_landmarks_view': False,
        }],
        remappings=[
            ('stereo_camera/left/image', '/camera/image_raw'),
            ('imu', '/imu/data')
        ]
    )

    return [vslam_node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])