using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VRTurboPiTeleop : MonoBehaviour
{
    [Header("VR Inputs")]
    public InputActionReference leftThumbstick;

    [Header("Speed Settings")]
    public float maxLinearSpeed = 0.5f;
    public float maxTurnSpeed = 3.0f;

    [Header("ROS Settings")]
    public string topicName = "/cmd_vel";
    public float publishFrequency = 0.1f; 

    private ROSConnection ros;
    private float timeElapsed;

    void OnEnable()
    {
        if (leftThumbstick != null && leftThumbstick.action != null)
        {
            leftThumbstick.action.Enable();
            Debug.Log("VR Teleop: Left Thumbstick Action Enabled.");
        }
    }

    void OnDisable()
    {
        if (leftThumbstick != null && leftThumbstick.action != null)
        {
            leftThumbstick.action.Disable();
        }
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishFrequency)
        {
            Vector2 leftStick = leftThumbstick != null ? leftThumbstick.action.ReadValue<Vector2>() : Vector2.zero;

            TwistMsg cmdVel = new TwistMsg();

            // --- THE INVERSION FIX ---
            // Added a negative sign to the Y-axis to flip forward/backward
            cmdVel.linear.x = -leftStick.y * maxLinearSpeed; 
            
            cmdVel.linear.y = 0.0;
            cmdVel.linear.z = 0.0;

            // Turning (Horizontal X)
            cmdVel.angular.x = 0.0;
            cmdVel.angular.y = 0.0;
            cmdVel.angular.z = -leftStick.x * maxTurnSpeed;

            ros.Publish(topicName, cmdVel);

            // Log movement to the console for confirmation
            if (leftStick.magnitude > 0.05f)
            {
                Debug.Log($"[ROS] Publishing: Linear X (Forward): {cmdVel.linear.x}, Angular Z (Turn): {cmdVel.angular.z}");
            }

            timeElapsed = 0f;
        }
    }
}