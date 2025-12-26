# Chapter 2: Python Agents with rclpy

This chapter focuses on integrating Python agents with the ROS 2 ecosystem using `rclpy`, the Python client library for ROS 2. You will learn how to create more complex Python nodes that can communicate with other ROS 2 components, enabling intelligent robotic behaviors.

## Learning Objectives

By the end of this chapter, you will be able to:

- Create Python-based ROS 2 nodes using `rclpy`.
- Implement publishers and subscribers in Python to exchange data.
- Develop ROS 2 services and clients in Python.
- Understand best practices for writing robust ROS 2 Python agents.

## Creating a Publisher-Subscriber Agent

Let's create a simple agent that publishes a "ping" message to a topic and subscribes to a "pong" topic.

### The Publisher Node (`ping_publisher.py`)

This node will continuously publish a string message to the `/ping` topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PingPublisher(Node):
    def __init__(self):
        super().__init__('ping_publisher')
        self.publisher_ = self.create_publisher(String, 'ping', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Ping {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    ping_publisher = PingPublisher()
    rclpy.spin(ping_publisher)
    ping_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### The Subscriber Node (`pong_subscriber.py`)

This node will subscribe to the `/ping` topic and print the received messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PongSubscriber(Node):
    def __init__(self):
        super().__init__('pong_subscriber')
        self.subscription = self.create_subscription(
            String,
            'ping',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    pong_subscriber = PongSubscriber()
    rclpy.spin(pong_subscriber)
    pong_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run the Agent

1.  **Open two separate terminals and source your ROS 2 environment in each.**
2.  **In the first terminal, run the publisher:**
    ```bash
    python3 ping_publisher.py
    ```
3.  **In the second terminal, run the subscriber:**
    ```bash
    python3 pong_subscriber.py
    ```

You should see the `pong_subscriber` receiving and printing the messages published by the `ping_publisher`. This example demonstrates basic topic-based communication between two Python ROS 2 nodes.
