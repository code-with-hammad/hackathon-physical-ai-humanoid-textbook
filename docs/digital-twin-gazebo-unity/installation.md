---
title: Installation Instructions
---

# Installation Instructions

This document provides detailed instructions for setting up the necessary environments for the Digital Twin (Gazebo & Unity) module. This includes installing Gazebo, Unity, and configuring their integration with ROS 2.

## 1. Gazebo Installation

Follow these steps to install Gazebo on your system. It is recommended to use the ROS 2-compatible version of Gazebo (e.g., Gazebo Garden or Gazebo Harmonic with their respective ROS 2 distributions).

1.  **Install ROS 2**: If you haven't already, install a compatible ROS 2 distribution (e.g., Humble, Iron, Jazzy). Refer to the official ROS 2 documentation for your operating system.
2.  **Install Gazebo**: Install Gazebo via your ROS 2 distribution or directly from the Gazebo website.
3.  **Verify Installation**:
    ```bash
    gazebo --version
    gz sim --version # For Gazebo Garden/Harmonic
    ```

## 2. Unity Installation

Follow these steps to install Unity Hub and a compatible Unity Editor version.

1.  **Download Unity Hub**: Download and install Unity Hub from the official Unity website.
2.  **Install Unity Editor**: Use Unity Hub to install a Unity Editor version compatible with the ROS-Unity Integration package (e.g., Unity 2022.3 LTS).
3.  **Verify Installation**: Launch Unity Hub and ensure your installed Editor is listed.

## 3. ROS-Unity Integration

This section covers how to set up communication between ROS 2 and Unity.

1.  **Install ROS-Unity Integration Package**:
    *   Open your Unity project (initialized in `unity/`).
    *   Import the ROS-Unity Integration package (e.g., `ROS-TCP-Connector`, `ROS-Unity-Message-Generation`) via the Unity Package Manager or by importing a `.unitypackage` file.
2.  **Configure ROS 2 Workspace**: Ensure your ROS 2 environment is sourced and any necessary ROS 2 packages for Unity communication are built.
    ```bash
    source /opt/ros/YOUR_ROS2_DISTRO/setup.bash
    cd YOUR_ROS2_WORKSPACE
    colcon build
    source install/setup.bash
    ```
3.  **Test Connection**: Use provided examples or simple publishers/subscribers to test the communication between ROS 2 and Unity.

## Troubleshooting

- Common installation issues and solutions.
- Debugging ROS 2 and Unity communication.
