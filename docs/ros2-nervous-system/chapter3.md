# Chapter 3: Humanoid URDF Modeling

This chapter delves into the Unified Robot Description Format (URDF), a standard XML format for describing robots in ROS. You will learn how to create a basic URDF model for a humanoid robot, defining its links, joints, and visual properties. Understanding URDF is crucial for simulating and controlling robots in environments like Gazebo and RViz.

## Learning Objectives

By the end of this chapter, you will be able to:

*   Understand the structure and components of a URDF file.
*   Define links (rigid bodies) and joints (connections between links).
*   Add visual and collision properties to links.
*   Create a simple URDF model for a humanoid robot.
*   Visualize a URDF model in RViz or Gazebo.

## What is URDF?

URDF (Unified Robot Description Format) is an XML file format that describes all aspects of a robot. It's used by ROS packages for various purposes, including:

*   **Visualization**: Displaying the robot in tools like RViz.
*   **Simulation**: Using the robot in physics simulators like Gazebo.
*   **Motion Planning**: Generating collision-free paths for the robot.
*   **Kinematics**: Calculating the robot's forward and inverse kinematics.

## Core URDF Elements

A URDF file primarily consists of two main elements:

*   **`<link>`**: Represents a rigid body of the robot. Links have physical and visual properties (e.g., mass, inertia, shape, color).
*   **`<joint>`**: Represents a connection between two links. Joints define the type of motion allowed between links (e.g., revolute, prismatic, fixed).

## Example: Basic Humanoid URDF

Let's create a very simple URDF for a humanoid robot. This example will focus on the basic structure with a torso, head, and two arms.

### The Code

Create a directory named `urdf` in your workspace, and inside it, create a file named `humanoid.urdf`.

```xml
<?xml version="1.0"?>
<robot name="humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Neck Joint (Torso to Head) -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
  </joint>

  <!-- Left Arm Link -->
  <link name="left_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Left Shoulder Joint (Torso to Left Arm) -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_arm"/>
    <origin xyz="0 0.05 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
  </joint>

  <!-- Right Arm Link -->
  <link name="right_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="green"/>
    </visual>
  </link>

  <!-- Right Shoulder Joint (Torso to Right Arm) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_arm"/>
    <origin xyz="0 -0.05 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="0.5"/>
  </joint>

</robot>
```

### How to Visualize the URDF

To visualize this URDF, you typically use `RViz` or `Gazebo`.

1.  **Ensure you have RViz installed** (usually comes with ROS 2 desktop installation).
2.  **Open a terminal and source your ROS 2 environment.**
3.  **Launch RViz with the URDF file:**
    ```bash
    ros2 launch urdf_tutorial display.launch model:=/path/to/your/humanoid.urdf
    ```
    (Note: You might need to install `urdf_tutorial` package: `sudo apt install ros-<ros2-distro>-urdf-tutorial`)

This will open RViz and display your simple humanoid robot model. You can interact with it to change joint angles and see the robot's configuration.
