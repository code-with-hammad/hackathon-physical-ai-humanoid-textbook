# Contracts for Module 3: The AI-Robot Brain (NVIDIA Isaac)

This directory would typically contain formal API contract definitions (e.g., OpenAPI specifications for REST APIs, GraphQL schemas).

For Module 3, the functional requirements primarily involve providing instructions, demonstrations, and configurations within a technical book context. Therefore, traditional API contracts for a software service are not generated here.

The "contracts" for this module are implicitly defined by:
- **ROS 2 Message Types**: The standard ROS 2 interfaces (e.g., `sensor_msgs/Image`, `sensor_msgs/PointCloud2`, `sensor_msgs/Imu`, `geometry_msgs/Twist`) used for inter-component communication between Isaac Sim, Isaac ROS, and Nav2. These define the data structures exchanged.
- **ROS 2 Services and Actions**: The functional interfaces for requesting and providing capabilities within the ROS 2 ecosystem (e.g., Nav2 actions for goal setting).
- **Configuration Formats**: The structure and parameters of configuration files for Isaac Sim, Isaac ROS, and Nav2.

These interfaces are described in more detail within the `data-model.md` and `research.md` documents, and will be elaborated upon in the implementation chapters.
