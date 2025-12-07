import rclpy
from rclpy.node import Node
from example_interfaces.msg import String as RobotActionMsg
from geometry_msgs.msg import Twist # For movement commands
from std_msgs.msg import String as PhraseMsg # For robot to "speak"

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor_node')
        self.action_subscriber = self.create_subscription(
            RobotActionMsg,
            'robot_action',
            self.action_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.phrase_publisher = self.create_publisher(PhraseMsg, '/robot_phrase', 10) # Placeholder for speech
        self.get_logger().info('Action Executor Node has been started.')

    def execute_action(self, action_type, params):
        self.get_logger().info(f"Executing action: {action_type} with params: {params}")

        if action_type == "move_forward":
            distance = float(params.get("distance", 0.0))
            twist_msg = Twist()
            twist_msg.linear.x = distance # Simplified: assume linear.x directly controls distance for now
            self.cmd_vel_publisher.publish(twist_msg)
            self.get_logger().info(f"Robot moving forward by {distance}m")
        elif action_type == "turn":
            angle = float(params.get("angle", 0.0))
            twist_msg = Twist()
            twist_msg.angular.z = np.deg2rad(angle) # Simplified: assume angular.z directly controls angle
            self.cmd_vel_publisher.publish(twist_msg)
            self.get_logger().info(f"Robot turning by {angle} degrees")
        elif action_type == "grasp_object":
            object_name = params.get("object_name", "unknown object")
            self.get_logger().info(f"Robot attempting to grasp: {object_name}")
            # In a real system, this would involve publishing to a grasping action server
        elif action_type == "release_object":
            self.get_logger().info("Robot releasing object")
            # In a real system, this would involve publishing to a grasping action server
        elif action_type == "navigate_to_pose":
            x = float(params.get("x", 0.0))
            y = float(params.get("y", 0.0))
            yaw = float(params.get("yaw", 0.0))
            self.get_logger().info(f"Robot navigating to x:{x}, y:{y}, yaw:{yaw}")
            # In a real system, this would involve sending a goal to a navigation action server
        elif action_type == "say_phrase":
            phrase = params.get("phrase", "Hello World")
            phrase_msg = PhraseMsg()
            phrase_msg.data = phrase
            self.phrase_publisher.publish(phrase_msg)
            self.get_logger().info(f"Robot says: {phrase}")
        elif action_type == "stop":
            twist_msg = Twist() # Send zero velocity command
            self.cmd_vel_publisher.publish(twist_msg)
            self.get_logger().info("Robot stopping.")
        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")

    def action_callback(self, msg):
        action_data = msg.data
        self.get_logger().info(f"Received action command: {action_data}")
        
        # Parse action_data, e.g., "action_type:param1=value1,param2=value2"
        parts = action_data.split(':')
        action_type = parts[0]
        params = {}
        if len(parts) > 1 and parts[1]: # Check if params part exists and is not empty
            param_pairs = parts[1].split(',')
            for pair in param_pairs:
                if '=' in pair:
                    key, value = pair.split('=')
                    params[key.strip()] = value.strip()
                elif pair: # Handle single parameter without explicit key
                    params['value'] = pair.strip() # Assign to a generic 'value' key
        
        self.execute_action(action_type, params)

def main(args=None):
    rclpy.init(args=args)
    action_executor_node = ActionExecutorNode()
    rclpy.spin(action_executor_node)
    action_executor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
