---
title: ROS 2 Nodes, Topics, and Services
---

# ROS 2 Nodes, Topics, and Services

## Introduction to ROS 2

The Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It is not an operating system in the traditional sense, but rather a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot applications. ROS 2 is designed for distributed systems, making it suitable for a wide range of robotics platforms, from embedded systems to supercomputers (Quigley et al., 2009).

At its core, ROS 2 facilitates communication between various independent software modules, enabling modular and reusable robot software components. This chapter will introduce three fundamental communication concepts in ROS 2: Nodes, Topics, and Services.

## 1. ROS 2 Nodes

In ROS 2, a **Node** is the basic unit of computation. A node is essentially an executable process that performs a specific task. For example, a robot's software might have a node responsible for reading sensor data, another for controlling motors, and yet another for performing navigation computations.

Nodes are designed to be modular and decoupled, meaning they can be developed, tested, and run independently. This modularity enhances code reusability and simplifies debugging of complex robotic systems. Each node communicates with other nodes using ROS 2 communication mechanisms, without needing to know the internal implementation details of other nodes.

## 2. ROS 2 Topics

**Topics** are a fundamental communication mechanism in ROS 2 used for asynchronous, many-to-many messaging. Topics implement a publish/subscribe model, where nodes publish data to a named topic, and other nodes can subscribe to that topic to receive the data.

This mechanism is ideal for streaming continuous data, such as sensor readings (e.g., camera images, LiDAR scans), motor commands, or robot pose updates. Publishers and subscribers do not directly know about each other; they only interact through the topic.

### 2.1. `rclpy` Publisher Example

Here's a simple `rclpy` (ROS 2 client library for Python) example of a node publishing string messages to a topic:

```python
# simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from SimplePublisher: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # Keep node alive
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.2. `rclpy` Subscriber Example

And here's a corresponding `rclpy` example of a node subscribing to messages from the same topic:

```python
# simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. ROS 2 Services

**Services** are a communication mechanism in ROS 2 used for synchronous, request/reply interactions. Unlike topics, services are designed for scenarios where a node needs to request a specific computation or action from another node and wait for a response. This is similar to a function call in a distributed system.

Services are useful for tasks that require a one-time response, such as querying a robot's battery level, triggering a specific action (e.g., "take a picture"), or performing a complex calculation.

### 3.1. `rclpy` Service Server Example

Here's a `rclpy` example of a service server that adds two integers:

```python
# simple_service_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Standard ROS 2 service message

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server "add_two_ints" ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.2. `rclpy` Service Client Example

And a corresponding `rclpy` example of a service client requesting the sum:

```python
# simple_service_client.py
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print(f'Usage: {sys.argv[0]} <int_a> <int_b>')
        sys.exit(1)

    simple_service_client = SimpleServiceClient()
    response = simple_service_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    simple_service_client.get_logger().info(
        f'Result of add_two_ints: {response.sum}')
    
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

Nodes, Topics, and Services are foundational building blocks for communication in ROS 2. Understanding these concepts is crucial for developing modular, robust, and scalable robotic applications. Topics provide asynchronous data streaming, while services offer synchronous request/reply interactions, catering to diverse communication needs in a distributed robotics environment.

## References

Quigley, M., Conley, K., Gerkey, B., Faust, J., Foote, T., Fiedler, J., ... & Smith, A. (2009, May). ROS: an open-source robot operating system. In *ICRA workshop on open source software* (Vol. 3, No. 12).
