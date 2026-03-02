using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class TurboPiTeleop : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/cmd_vel";

    // Speed settings for the REAL robot (keep these safe/low)
    public float maxLinearSpeed = 0.3f;   // Meters per second
    public float maxTurnSpeed = 1.0f;     // Radians per second

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        TwistMsg cmdVel = new TwistMsg();

        // 1. FORWARD / BACK (W/S)
        // Unity Vertical: +1 (W) / -1 (S)
        // ROS Linear X: + (Forward) / - (Back)
        cmdVel.linear.x = -Input.GetAxis("Vertical") * maxLinearSpeed;

        // 2. STRAFE LEFT / RIGHT (A/D)
        // Unity Horizontal: +1 (D/Right) / -1 (A/Left)
        // ROS Linear Y: + (Left) / - (Right)
        // We invert this so pressing A (Negative) becomes Positive Y (Left)
        //cmdVel.linear.y = Input.GetAxis("Horizontal") * maxLinearSpeed;

        // 3. TURN LEFT / RIGHT (Q/E)
        // Unity keys: Q (Turn Left) / E (Turn Right)
        // ROS Angular Z: + (Left) / - (Right)
        if (Input.GetKey(KeyCode.Q))
        {
            cmdVel.angular.z = maxTurnSpeed; // Positive = Left Turn
        }
        else if (Input.GetKey(KeyCode.E))
        {
            cmdVel.angular.z = -maxTurnSpeed; // Negative = Right Turn
        }

        ros.Publish(topicName, cmdVel);
    }
}
