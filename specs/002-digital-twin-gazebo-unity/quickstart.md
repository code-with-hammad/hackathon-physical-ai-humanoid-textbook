# Quickstart: Setting up the Digital Twin Environment

This quickstart guides you through setting up the necessary environments for the Digital Twin module, including Gazebo for robot simulation and Unity for visualization.

## 1. Gazebo Simulation Setup

### Prerequisites

*   **ROS 2 Installation**: Ensure you have ROS 2 installed and sourced as per the instructions in Module 1.
*   **Gazebo Installation**: Gazebo usually comes with the ROS 2 desktop installation. If not, follow the official Gazebo installation guide for your ROS 2 distribution.

### Launching the Humanoid Simulation

1.  **Open a terminal and source your ROS 2 environment.**
2.  **Navigate to your workspace where the Gazebo models are located (e.g., `~/robot_ws/src/my_robot/`).**
3.  **Launch Gazebo with the humanoid robot model and world file:**
    ```bash
    ros2 launch <your_robot_package> gazebo_launch.py # This will be a custom launch file
    ```
    This command will open Gazebo, load the specified world, and spawn the humanoid robot.

## 2. Unity Visualization Setup

### Prerequisites

*   **Unity Hub and Unity Editor**: Install Unity Hub and a compatible Unity Editor version (e.g., Unity 2022.3 LTS) from the [Unity website](https://unity3d.com/get-unity/download).
*   **ROS-Unity Integration Package**: You will need to install a ROS-Unity integration package (e.g., [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) or a similar custom solution).

### Setting up the Unity Project

1.  **Open Unity Hub and create a new 3D project or open the provided Unity project for this module.**
2.  **Import the ROS-Unity integration package into your Unity project.**
3.  **Configure ROS 2 communication**: Follow the package's instructions to set up message serialization/deserialization and establish connection to your ROS 2 environment.

### Running the Unity Visualization

1.  **Ensure your Gazebo simulation is running and publishing ROS 2 data.**
2.  **In Unity Editor, open the scene containing the robot visualization.**
3.  **Enter Play Mode** (click the Play button in the Unity Editor). The Unity application should now connect to ROS 2 and display the robot's state from Gazebo.

## 3. Verifying Sensor Data

While Gazebo simulation is running and publishing sensor data to ROS 2 topics:

1.  **Open a new terminal and source your ROS 2 environment.**
2.  **Use `ros2 topic list` to see all available topics.** You should see topics like `/lidar/scan`, `/camera/depth/image_raw`, `/imu/data`.
3.  **Use `ros2 topic echo <topic_name>` to inspect the data being published.** For example:
    ```bash
    ros2 topic echo /imu/data
    ```
    This will show the raw sensor data being streamed from Gazebo.
