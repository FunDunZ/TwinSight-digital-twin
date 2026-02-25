using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class TwinSightTeleop : MonoBehaviour
{
    public string cmdVelTopic = "/cmd_vel";
    // We will tweak these multipliers later to perfectly match the physical robot's speed
    public float linearSpeedMultiplier = 1.0f;
    public float angularSpeedMultiplier = 1.0f;

    void Start()
    {
        // Tell Unity to listen to the exact same driving topic the physical robot uses
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(cmdVelTopic, CmdVelCallback);
    }

    void CmdVelCallback(TwistMsg msg)
    {
        // ROS X is forward. In Unity, Z is forward.
        float moveDistance = (float)msg.linear.x * linearSpeedMultiplier * Time.deltaTime;

        // ROS Z is rotation. In Unity, Y is rotation.
        float turnAngle = (float)-msg.angular.z * Mathf.Rad2Deg * angularSpeedMultiplier * Time.deltaTime;

        // Apply the movement to the Unity Cube
        transform.Translate(0, 0, moveDistance);
        transform.Rotate(0, turnAngle, 0);
    }
}
