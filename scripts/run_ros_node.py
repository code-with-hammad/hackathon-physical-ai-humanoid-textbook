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
