# Chapter 1: ROS 2 Nodes, Topics, and Services

This chapter introduces the fundamental concepts of ROS 2, focusing on nodes, topics, and services. You will learn how to create and run a basic ROS 2 node, publish data to a topic, and subscribe to a topic to receive data. Additionally, you will explore how to implement and call ROS 2 services for request-response communication.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the core components of ROS 2: nodes, topics, and services.
- Create and run a simple ROS 2 node using `rclpy`.
- Publish messages to ROS 2 topics.
- Subscribe to ROS 2 topics to receive messages.
- Implement and call ROS 2 services.

## What is a ROS 2 Node?

In ROS 2, a **node** is an executable process that performs computation. Nodes are designed to be modular, meaning each node is responsible for a single, well-defined task. For example, one node might be responsible for reading data from a sensor, another for processing that data, and yet another for controlling a motor.

## ROS 2 Topics

**Topics** are the main way for nodes to exchange data in ROS 2. Topics are essentially named buses over which nodes send messages. Nodes that want to send data to a topic are called **publishers**, and nodes that want to receive data from a topic are called **subscribers**.

## ROS 2 Services

**Services** provide a request/reply mechanism between nodes. Unlike topics, which are designed for continuous data streams, services are used for calls that expect a response. For example, a node might offer a service to calculate a specific value, and another node can call this service to get the result.

## Example: Running a Simple ROS 2 Node

Let's start by creating a simple ROS 2 node that prints "Hello, ROS 2!" to the console.

First, ensure you have a ROS 2 environment sourced.

### The Code

Create a Python file named `my_first_node.py` in your workspace.

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node("my_first_node") # Create a node with the name "my_first_node"
    node.get_logger().info("Hello, ROS 2!") # Log a message
    rclpy.spin_once(node, timeout_sec=1) # Keep node alive for 1 second
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run the Node

1.  **Open a terminal and source your ROS 2 environment.**
2.  **Navigate to the directory where you saved `my_first_node.py`.**
3.  **Run the node using `ros2 run`:**
    ```bash
    python3 my_first_node.py
    ```

You should see "Hello, ROS 2!" printed in your terminal. This demonstrates the basic structure of a ROS 2 node and how to execute it. In the following sections, we will expand on this by exploring topics and services.
