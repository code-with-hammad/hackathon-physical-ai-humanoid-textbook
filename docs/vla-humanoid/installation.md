---
title: Installation Instructions
---

# Installation Instructions

This document provides detailed instructions for setting up the necessary environments for Module 4: Vision-Language-Action (VLA). This includes installing OpenAI Whisper, configuring Large Language Models (LLMs), and setting up ROS 2 for the VLA pipeline.

## 1. ROS 2 Installation

If you haven't already, install a compatible ROS 2 distribution (e.g., Humble, Iron, Jazzy). Refer to the official ROS 2 documentation for your operating system.

## 2. OpenAI Whisper Setup

1.  **Install Whisper**: Install the OpenAI Whisper Python package.
    ```bash
    pip install openai-whisper
    ```
    (Alternatively, if using the OpenAI API, ensure the `openai` Python package is installed: `pip install openai`).
2.  **API Key Configuration**: If using the OpenAI API, set your API key as an environment variable:
    ```bash
    export OPENAI_API_KEY="YOUR_OPENAI_API_KEY"
    ```
    (Replace with your actual key).

## 3. Large Language Model (LLM) Access

Depending on the LLM you choose, the setup process will vary:

### Option A: Cloud-based LLM (e.g., OpenAI GPT-4, Gemini API)

- Obtain an API key for your chosen LLM provider.
- Install the corresponding Python SDK (e.g., `pip install openai` for OpenAI, `pip install google-generativeai` for Gemini).
- Configure your API key as an environment variable or securely load it in your application.

### Option B: Local LLM (e.g., Llama 3 via Ollama/LocalAI)

- Install a local LLM server (e.g., Ollama, LocalAI).
- Download the desired LLM model.
- Ensure your ROS 2 nodes can communicate with the local LLM server.

## 4. ROS 2 Workspace Setup for VLA

1.  Create a new ROS 2 workspace:
    ```bash
    mkdir -p ~/ros2_vla_ws/src
    cd ~/ros2_vla_ws/src
    ```
2.  Clone the necessary VLA-related ROS 2 packages into your workspace. These will include packages for Whisper integration, LLM planning, and robot action execution.
    ```bash
    git clone <openai_whisper_ros_pkg_url>
    git clone <llm_planning_ros_pkg_url>
    git clone <humanoid_vla_ros_pkg_url>
    ```
3.  Build the workspace:
    ```bash
    cd ~/ros2_vla_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    ```
4.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

## 5. Simulated Humanoid Robot Environment

Ensure you have a simulated robot environment (e.g., Gazebo or Isaac Sim) with a humanoid robot model configured to publish sensor data and receive ROS 2 command messages (e.g., `geometry_msgs/Twist`). Refer to previous modules for setting up such simulation environments.

## Troubleshooting

- Common installation issues and solutions for ROS 2, Python packages, and API keys.
- Debugging ROS 2 node communication and LLM API calls.
