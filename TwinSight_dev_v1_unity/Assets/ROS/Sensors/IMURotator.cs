using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor; // The namespace for sensor_msgs/Imu

public class IMURotator : MonoBehaviour
{
    // The ROS topic your TurboPi is publishing IMU data to
    public string imuTopic = "/ros_robot_controller/imu_raw";

    void Start()
    {
        // Get the ROS connection and subscribe to the topic
        ROSConnection.GetOrCreateInstance().Subscribe<ImuMsg>(imuTopic, ImuCallback);
        Debug.Log("Subscribed to " + imuTopic);
    }

    // This function runs every time a new IMU message arrives
    void ImuCallback(ImuMsg imuMessage)
    {
        // 1. Grab the raw orientation data from the message
        var rosOrientation = imuMessage.orientation;

        // 2. Convert ROS coordinates (FLU) to Unity coordinates (RUF) and apply it to the cube
        transform.rotation = rosOrientation.From<FLU>();
    }
}
