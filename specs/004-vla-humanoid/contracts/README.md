# Contracts for Module 4: Vision-Language-Action (VLA)

This directory would typically contain formal API contract definitions (e.g., OpenAPI specifications for REST APIs, GraphQL schemas).

For Module 4, the functional requirements primarily involve providing instructions, demonstrations, and configurations within a technical book context. Therefore, traditional API contracts for a software service are not generated here.

The "contracts" for this module are implicitly defined by:
- **ROS 2 Message Types**: The standard ROS 2 interfaces (e.g., `sensor_msgs/Image`, `geometry_msgs/Twist`) used for inter-component communication within the VLA pipeline. These define the data structures exchanged.
- **ROS 2 Services and Actions**: The functional interfaces for requesting and providing capabilities within the ROS 2 ecosystem.
- **OpenAI Whisper API/SDK**: The interface for interacting with the OpenAI Whisper service for speech-to-text conversion.
- **Large Language Model (LLM) API/SDK**: The interface for interacting with the chosen LLM for cognitive planning, including prompt structure and function calling conventions.
- **Configuration Formats**: The structure and parameters of configuration files for various components (e.g., action mapping rules, LLM prompts).

These interfaces are described in more detail within the `data-model.md` and `research.md` documents, and will be elaborated upon in the implementation chapters.
