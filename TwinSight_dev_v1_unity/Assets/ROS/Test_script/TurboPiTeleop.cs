using UnityEngine;
using UnityEngine.InputSystem; // Added to talk to the New Input System
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class TurboPiTeleop : MonoBehaviour
{
    [Header("Speed Settings")]
    public float maxLinearSpeed = 0.3f;   
    public float maxTurnSpeed = 3.0f;     

    [Header("ROS Settings")]
    public string topicName = "/cmd_vel";
    public float publishFrequency = 0.1f; 

    private ROSConnection ros;
    private float timeElapsed;
    
    // The new input listener
    private InputAction moveAction;

    void Awake()
    {
        // 1. Primary: Bind to the Quest's Left Hand Joystick
        moveAction = new InputAction("Move", binding: "<XRController>{LeftHand}/joystick");
        
        // 2. Fallback: Bind to a standard Gamepad (Xbox/PlayStation)
        moveAction.AddBinding("<Gamepad>/leftStick");

        // 3. Fallback: Bind to PC Keyboard (WASD) for quick Editor testing
        moveAction.AddCompositeBinding("2DVector")
            .With("Up", "<Keyboard>/w")
            .With("Down", "<Keyboard>/s")
            .With("Left", "<Keyboard>/a")
            .With("Right", "<Keyboard>/d");
    }

    void OnEnable()
    {
        moveAction.Enable(); // Turn the listener on
    }

    void OnDisable()
    {
        moveAction.Disable(); // Turn the listener off to save memory
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
            TwistMsg cmdVel = new TwistMsg();

            // Read the 2D vector coordinates from the physical thumbstick or keyboard
            Vector2 joystickInput = moveAction.ReadValue<Vector2>();

            // 1. FORWARD / BACK
            // Preserving your original negative sign for linear.x
            cmdVel.linear.x = -joystickInput.y * maxLinearSpeed; 
            cmdVel.linear.y = 0.0;
            cmdVel.linear.z = 0.0;

            // 2. TURN LEFT / RIGHT
            // D/Right is +1. Inverted so Right creates a negative Z (ROS Standard Right Turn)
            cmdVel.angular.x = 0.0;
            cmdVel.angular.y = 0.0;
            cmdVel.angular.z = -joystickInput.x * maxTurnSpeed;

            ros.Publish(topicName, cmdVel);
            timeElapsed = 0f;
        }
    }
}