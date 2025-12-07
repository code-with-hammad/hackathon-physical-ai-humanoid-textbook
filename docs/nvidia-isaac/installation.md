---
title: Installation Instructions
---

# Installation Instructions

This document provides detailed instructions for setting up the necessary environments for Module 3: The AI-Robot Brain (NVIDIA Isaac). This includes installing NVIDIA Isaac Sim, Isaac ROS, and configuring Nav2 for humanoid robotics.

## 1. NVIDIA Isaac Sim Installation

Follow these steps to install NVIDIA Isaac Sim. It is part of the NVIDIA Omniverse platform.

1.  **NVIDIA GPU Drivers**: Ensure you have the latest NVIDIA GPU drivers installed.
2.  **NVIDIA Omniverse Launcher**: Download and install the Omniverse Launcher from the NVIDIA website.
3.  **Install Isaac Sim**: Within the Omniverse Launcher, navigate to the "Exchange" tab, search for "Isaac Sim," and install it.
4.  **Verify Installation**: Launch Isaac Sim and load a sample scene to confirm it's working correctly.

## 2. Isaac ROS Installation

Isaac ROS components are typically deployed as Docker containers or built from source within a ROS 2 workspace.

1.  **Install Docker**: Install Docker Engine and NVIDIA Container Toolkit (nvidia-docker2) for GPU acceleration with Docker.
2.  **ROS 2 Installation**: Ensure you have a compatible ROS 2 distribution (e.g., Humble, Iron, Jazzy) installed and sourced.
3.  **Create ROS 2 Workspace**:
    ```bash
    mkdir -p ~/ros2_isaac_ws/src
    cd ~/ros2_isaac_ws/src
    ```
4.  **Clone Isaac ROS Repositories**: Clone the necessary Isaac ROS packages into your workspace. Specific packages (e.g., `isaac_ros_visual_slam`) will be detailed in the respective chapters.
    ```bash
    git clone https://github.com/NVIDIA-AI-IOT/isaac_ros_common.git
    git clone https://github.com/NVIDIA-AI-IOT/isaac_ros_visual_slam.git
    # ... clone other required Isaac ROS packages
    ```
5.  **Build Workspace**:
    ```bash
    cd ~/ros2_isaac_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    source install/setup.bash
    ```
6.  **Verify Installation**: Run an Isaac ROS sample application (e.g., a VSLAM example).

## 3. Nav2 Installation and Configuration

Nav2 is part of the ROS 2 ecosystem. Configuration for humanoid robots involves specific parameter tuning.

1.  **ROS 2 Nav2 Installation**: Install Nav2 for your ROS 2 distribution.
    ```bash
    sudo apt install ros-<ROS_DISTRO>-navigation2 ros-<ROS_DISTRO>-nav2-msgs
    ```
2.  **Humanoid-Specific Configuration**:
    *   Create a dedicated ROS 2 package for your humanoid's Nav2 configuration (e.g., `nav2_humanoid`).
    *   Place your `humanoid_nav2_params.yaml` file (from this module's example) into the `params` directory of this package.
    *   Create launch files (e.g., `humanoid_navigation.launch.py`) to integrate these parameters.
3.  **Verify Installation**: Launch Nav2 with your humanoid configuration and test basic path planning.

## Troubleshooting

- Common installation issues and solutions for NVIDIA Isaac components.
- Debugging ROS 2, Isaac ROS, and Nav2 communication.
