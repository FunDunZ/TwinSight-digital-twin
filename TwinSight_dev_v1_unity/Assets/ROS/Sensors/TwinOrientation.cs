using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class TwinOrientation : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/odom";

    [Header("The Digital Twin")]
    [Tooltip("Drag the 3D model of your TurboPi here")]
    public Transform robotModel;

    private ROSConnection ros;

    void Start()
    {
        // Open the bridge and subscribe to the /odom topic
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(topicName, SyncRotation);
    }

    void SyncRotation(OdometryMsg msg)
    {
        // 1. Drill down into the message to grab the raw IMU Quaternion
        var rosQuat = msg.pose.pose.orientation;

        // 2. Translate ROS coordinates (Z-Up) to Unity coordinates (Y-Up)
        // Unity(x, y, z, w) = ROS(-y, z, -x, w)
        Quaternion unityRotation = new Quaternion(
            (float)-rosQuat.y, 
            (float)rosQuat.z, 
            (float)-rosQuat.x, 
            (float)rosQuat.w
        );

        // 3. Apply the absolute rotation to the 3D model
        robotModel.rotation = unityRotation;
    }
}