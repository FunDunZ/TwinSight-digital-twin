using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Standard ROS 2 messages

public class SimpleSubscriber : MonoBehaviour
{
    public string topicName = "ros_hello";

    void Start()
    {
        // Get the connection and tell it which function to trigger when a message arrives
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(topicName, MessageReceived);
        Debug.Log("Subscribed to " + topicName);
    }

    // This callback function fires every time a packet hits the TwinSight bridge
    void MessageReceived(StringMsg incomingMessage)
    {
        Debug.Log("Received from ROS 2: " + incomingMessage.data);
    }
}
