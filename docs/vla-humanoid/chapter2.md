---
title: LLM-Based Cognitive Planning for ROS 2
---

# LLM-Based Cognitive Planning for ROS 2

This chapter explores how Large Language Models (LLMs) can be integrated into ROS 2 to enable cognitive planning for humanoid robots. We will cover the principles of prompt engineering, function calling, and translating high-level natural language goals into sequences of executable ROS 2 actions.

## 1. Introduction to Cognitive Planning with LLMs

- The role of LLMs in robotics: reasoning, task decomposition, human-robot interaction.
- From high-level goals to low-level actions: the planning problem.
- Advantages of LLM-based planning (flexibility, natural language interface) vs. traditional planners.

## 2. Prompt Engineering for Robotics

- Designing effective system prompts to guide the LLM's planning process.
- Defining robot capabilities as "tools" or "functions" for the LLM.
- Strategies for grounding LLM outputs in the physical world.
- Handling context and memory in LLM-based planning.

## 3. Integrating LLMs with ROS 2

- Communication interfaces: using LLM APIs (e.g., OpenAI, Gemini) within ROS 2 nodes.
- Translating LLM-generated plans: parsing LLM responses into structured `Robot Action`s.
- Executing `Robot Action`s as ROS 2 commands (topics, services, actions).

## 4. Perception Feedback in LLM Planning

- Incorporating real-time perception data (e.g., object detection, pose estimation) into the LLM's context.
- Enabling the LLM to dynamically adapt plans based on environmental changes.
- Challenges of integrating noisy or incomplete perception data.

## 5. Building a ROS 2 LLM Planner Node

- Developing a ROS 2 Python node that:
  - Subscribes to natural language goals (e.g., from `Transcribed Text`).
  - Queries the LLM with the goal and available tools.
  - Parses the LLM's plan (sequence of actions).
  - Publishes individual `Robot Action`s to a ROS 2 topic.
- Developing a ROS 2 Python node for action execution that:
  - Subscribes to `Robot Action`s.
  - Translates them into specific ROS 2 commands (e.g., publishing `geometry_msgs/Twist`).
  - Monitors execution status and reports back.

## Practical Exercises

- Experimenting with different prompts to guide an LLM to generate robot action sequences.
- Building a basic ROS 2 node that interacts with an LLM API.
- Implementing a simple action execution node that moves a simulated robot based on LLM output.

## Further Reading

- Research papers on LLMs in robotics and cognitive architectures.
- OpenAI Function Calling documentation.
- ROS 2 control and action frameworks.
