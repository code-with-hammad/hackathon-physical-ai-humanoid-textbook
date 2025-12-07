using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Example for std_msgs. For specific robot messages, you'd generate them.

public class Ros2RobotSubscriber : MonoBehaviour
{
    ROSConnection rosConnection;
    public string ros2TopicName = "/robot_state"; // Replace with your actual ROS 2 topic

    void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.RegisterSubscriber<StringMsg>(ros2TopicName, Ros2RobotStateCallback);
        Debug.Log($"Subscribing to ROS 2 topic: {ros2TopicName}");
    }

    void Ros2RobotStateCallback(StringMsg msg)
    {
        // This is a placeholder. In a real scenario, you would parse
        // specific robot state messages (e.g., JointState, Odometry)
        // and apply them to your Unity robot model.
        Debug.Log($"Received ROS 2 message on {ros2TopicName}: {msg.data}");

        // Example: If msg.data contains joint angles, update your robot's joints here.
    }
}
