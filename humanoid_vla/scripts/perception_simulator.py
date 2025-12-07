import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Using String for simplicity;
                                # in real app, define custom PerceptionFeedbackMsg

class PerceptionSimulatorNode(Node):
    def __init__(self):
        super().__init__('perception_simulator_node')
        self.perception_publisher = self.create_publisher(String, 'perception_feedback', 10)
        self.timer = self.create_timer(1.0, self.publish_perception_feedback) # Publish every 1 second
        self.get_logger().info('Perception Simulator Node has been started.')

        self.simulated_objects = [
            {"id": "red_ball", "pose": {"x": 1.0, "y": 0.5}, "color": "red"},
            {"id": "blue_cube", "pose": {"x": -0.8, "y": 1.2}, "color": "blue"},
            {"id": "table", "pose": {"x": 2.0, "y": 1.0}, "color": "brown"},
        ]
        self.current_robot_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def publish_perception_feedback(self):
        # Simulate current perception feedback for the LLM
        perception_data = {
            "robot_pose": self.current_robot_pose,
            "detected_objects": self.simulated_objects,
            "environment_state": "clear", # Example
        }
        
        # Convert dictionary to JSON string for publishing
        import json
        perception_msg = String()
        perception_msg.data = json.dumps(perception_data)
        self.perception_publisher.publish(perception_msg)
        self.get_logger().info('Published simulated perception feedback.')

    # In a real system, you might have subscribers to update robot pose
    # and dynamic object states for more realistic simulation.

def main(args=None):
    rclpy.init(args=args)
    perception_simulator_node = PerceptionSimulatorNode()
    rclpy.spin(perception_simulator_node)
    perception_simulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
