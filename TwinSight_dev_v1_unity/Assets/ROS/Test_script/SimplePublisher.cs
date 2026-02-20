using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Standard ROS 2 messages

public class SimplePublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "unity_hello";

    void Start()
    {
        // Start the TCP connection and register the topic
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
        Debug.Log("Publisher Registered!");
    }

    void Update()
    {
        // Pressing the Spacebar sends the message
        if (Input.GetKeyDown(KeyCode.Space))
        {
            StringMsg msg = new StringMsg("Hello ROS 2! The bridge is working.");
            ros.Publish(topicName, msg);
            Debug.Log("Message sent!");
        }
    }
}
