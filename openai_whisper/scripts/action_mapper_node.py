import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.msg import String as RobotActionMsg # Using String for simplicity;
                                                        # in real app, define custom action msg

class ActionMapperNode(Node):
    def __init__(self):
        super().__init__('action_mapper_node')
        self.transcription_subscriber = self.create_subscription(
            String,
            'transcribed_text',
            self.transcription_callback,
            10
        )
        self.robot_action_publisher = self.create_publisher(RobotActionMsg, 'robot_action', 10)
        self.get_logger().info('Action Mapper Node has been started.')

        # Define a simple action mapping
        self.action_map = {
            "move forward five steps": "move_forward:5",
            "turn left": "turn_left:90",
            "turn right": "turn_right:90",
            "stop": "stop:",
            "pick up the ball": "pick_up_object:ball"
        }

    def transcription_callback(self, msg):
        received_text = msg.data.lower().strip()
        self.get_logger().info(f'Received transcription: "{received_text}"')

        robot_action_command = self.action_map.get(received_text)

        if robot_action_command:
            self.get_logger().info(f'Mapped to action: "{robot_action_command}"')
            action_msg = RobotActionMsg()
            action_msg.data = robot_action_command
            self.robot_action_publisher.publish(action_msg)
        else:
            self.get_logger().warn(f'No direct action mapping found for: "{received_text}"')

def main(args=None):
    rclpy.init(args=args)
    action_mapper_node = ActionMapperNode()
    rclpy.spin(action_mapper_node)
    action_mapper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
