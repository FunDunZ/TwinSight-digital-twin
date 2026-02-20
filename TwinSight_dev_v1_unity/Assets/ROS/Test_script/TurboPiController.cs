using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; 

public class TurboPiTeleop : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/cmd_vel"; // Update this to match Step 1!

    public float speed = 0.2f;       
    public float turnSpeed = 1.0f;   

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        TwistMsg cmdVel = new TwistMsg();

        // Map WASD to Forward/Back and Turn
        cmdVel.linear.x = Input.GetAxis("Vertical") * speed;
        cmdVel.angular.z = -Input.GetAxis("Horizontal") * turnSpeed;

        // Map Q/E to Strafe Left/Right (because you have Mecanum wheels!)
        if (Input.GetKey(KeyCode.Q)) cmdVel.linear.y = speed;
        if (Input.GetKey(KeyCode.E)) cmdVel.linear.y = -speed;

        ros.Publish(topicName, cmdVel);
    }
}
