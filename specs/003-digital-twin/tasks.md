# Tasks: Module 2: The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `specs/003-digital-twin/`

## Phase 1: Setup (Shared Infrastructure)

- [X] T001 Create directory `docs/digital-twin-gazebo-unity`
- [X] T002 Create `docs/digital-twin-gazebo-unity/_category_.json`
- [X] T003 Create directory `gazebo/models`
- [X] T004 Create directory `gazebo/worlds`
- [X] T005 Create directory `gazebo/plugins`
- [X] T006 Create directory `unity/Assets`
- [X] T007 Create directory `unity/ProjectSettings`

---

## Phase 3: User Story 1 - Run a Gazebo Humanoid Simulation (Priority: P1) 🎯 MVP

**Goal**: A student can successfully run a Gazebo simulation of a humanoid robot, observing physics such as gravity and collisions.

**Independent Test**: The student can launch Gazebo with the humanoid model and verify basic physics interactions (e.g., robot falls due to gravity, collides with objects).

### Implementation for User Story 1

- [X] T008 [US1] Create `docs/digital-twin-gazebo-unity/chapter1.md`
- [X] T009 [US1] Write content for "Physics in Gazebo (gravity, collisions)" in `docs/digital-twin-gazebo-unity/chapter1.md`
- [X] T010 [US1] Create a basic humanoid model in `gazebo/models/humanoid_robot/model.sdf`
- [X] T011 [US1] Create a Gazebo world file with static objects in `gazebo/worlds/empty_world.world`
- [X] T012 [US1] Create a ROS 2 launch file for Gazebo simulation in `gazebo/launch/humanoid_sim.launch.py`

---

## Phase 4: User Story 2 - Visualize Human-Robot Interaction in Unity (Priority: P2)

**Goal**: A developer can visualize human-robot interaction within a Unity environment, connected to the Gazebo simulation.

**Independent Test**: The developer can connect Unity to the Gazebo simulation and see the humanoid robot's movements mirrored in Unity, along with any simulated human interaction elements.

### Implementation for User Story 2

- [X] T013 [US2] Create `docs/digital-twin-gazebo-unity/chapter2.md`
- [X] T014 [US2] Write content for "Human–Robot Interaction in Unity" in `docs/digital-twin-gazebo-unity/chapter2.md`
- [X] T015 [US2] Initialize Unity project in `unity/`
- [X] T016 [US2] Import ROS-Unity integration package into Unity project
- [X] T017 [US2] Create Unity scene for robot visualization and human interaction elements in `unity/Assets/Scenes/RobotVisualization.unity`
- [X] T018 [US2] Implement C# scripts for ROS 2 subscription to robot state in `unity/Assets/Scripts/Ros2RobotSubscriber.cs`

---

## Phase 5: User Story 3 - Publish Simulated Sensor Data (Priority: P3)

**Goal**: A student can configure and publish simulated sensor data (e.g., LiDAR, Depth, IMU) from the Gazebo environment.

**Independent Test**: The student can run the Gazebo simulation and use ROS 2 tools (e.g., `ros2 topic echo`) to verify that sensor data is being published correctly on designated topics.

### Implementation for User Story 3

- [X] T019 [US3] Create `docs/digital-twin-gazebo-unity/chapter3.md`
- [X] T020 [US3] Write content for "Sensor Simulation: LiDAR, Depth, IMU" in `docs/digital-twin-gazebo-unity/chapter3.md`
- [X] T021 [US3] Add LiDAR sensor to `gazebo/models/humanoid_robot/model.sdf`
- [X] T022 [US3] Add Depth camera sensor to `gazebo/models/humanoid_robot/model.sdf`
- [X] T023 [US3] Add IMU sensor to `gazebo/models/humanoid_robot/model.sdf`
- [X] T024 [US3] Configure Gazebo plugins to publish sensor data to ROS 2 topics

---

## Phase N: Polish & Cross-Cutting Concerns

- [X] T025 Configure `docusaurus.config.ts` to include the new module
- [X] T026 Configure `sidebars.ts` to include the new module sidebar
- [X] T027 Add installation instructions for Gazebo and Unity in `docs/digital-twin-gazebo-unity/installation.md`

---

## Dependencies & Execution Order

- **Setup (Phase 1)**: No dependencies.
- **User Stories (Phase 3+)**: Depend on Setup completion. User stories can be implemented in parallel.
- **Polish (Final Phase)**: Depends on all user stories being complete.