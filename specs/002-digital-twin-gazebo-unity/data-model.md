# Data Model: Digital Twin (Gazebo & Unity) Simulation Entities and Data Flow

## Gazebo Entities

### Humanoid Robot Model

- **`links`**: Rigid bodies of the robot (e.g., torso, head, limbs) with visual, inertial, and collision properties.
- **`joints`**: Connections between links, defining degrees of freedom and limits.
- **`sensors`**: Simulated LiDAR, Depth, and IMU sensors attached to specific links.
- **`plugins`**: Gazebo plugins for physics, sensor data publication, and ROS 2 interface.

### World

- **`ground_plane`**: A static ground plane for the robot to stand on.
- **`static_objects`**: Simple geometric shapes for collision demonstration.
- **`light_sources`**: Environmental lighting.

## Unity Entities

### Visualized Robot Model

- **`parts`**: 3D models representing the robot's links, mirroring the Gazebo model.
- **`joints`**: Visual representation of joints, allowing for real-time articulation based on Gazebo state.

### Human Representation

- **`avatar`**: A simple 3D model representing a human.
- **`interaction_elements`**: UI elements or visual cues for human interaction.

## Data Flow between Gazebo and Unity (via ROS 2)

### Gazebo -> ROS 2 (Publishers)

- **`robot_state_publisher`**: Publishes the current joint states and TF transformations of the humanoid robot (`sensor_msgs/msg/JointState`, `tf2_msgs/msg/TFMessage`).
- **`LiDAR_publisher`**: Publishes simulated LiDAR scan data (`sensor_msgs/msg/LaserScan`).
- **`Depth_publisher`**: Publishes simulated depth images (`sensor_msgs/msg/Image`).
- **`IMU_publisher`**: Publishes simulated IMU data (`sensor_msgs/msg/Imu`).

### ROS 2 <- Unity (Subscribers)

- **`robot_state_subscriber`**: Subscribes to `robot_state_publisher` topics to update the Unity robot model.
- **`human_interaction_publisher`**: Publishes human interaction events/commands from Unity to ROS 2 topics (`std_msgs/msg/String` or custom message types).

## Communication Protocol

- **Middleware**: ROS 2 (rclpy and potentially ROS-Unity integration packages).
- **Message Types**: Standard ROS 2 message types will be used where applicable. Custom message types will be defined as needed for specific human interaction events.
- **Frequency**: Sensor data and robot state updates will be published at a configurable frequency to maintain visualization fidelity while managing network bandwidth.
