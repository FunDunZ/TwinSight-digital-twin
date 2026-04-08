using UnityEngine;
using UnityEngine.InputSystem; // We must add this to talk to the New Input System

public class MecanumTeleop : MonoBehaviour
{
    [Header("Wheel Joints")]
    public ArticulationBody frontLeft;
    public ArticulationBody frontRight;
    public ArticulationBody backLeft;
    public ArticulationBody backRight;
    
    [Header("Speed Settings")]
    public float speed = 500.0f; 
    public float turnMultiplier = 2.5f;

    // This creates an input listener for the left thumbstick
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
        SetupWheel(frontLeft);
        SetupWheel(frontRight);
        SetupWheel(backLeft);
        SetupWheel(backRight);
    }

    void SetupWheel(ArticulationBody wheel)
    {
        var drive = wheel.xDrive;
        drive.damping = 100f;     
        drive.stiffness = 0f;     
        drive.forceLimit = 10000f; 
        drive.driveType = ArticulationDriveType.Velocity;
        wheel.xDrive = drive;
    }

    void Update()
    {
        // Read the 2D vector coordinates from the physical thumbstick
        Vector2 joystickInput = moveAction.ReadValue<Vector2>();

        // Y is forward/backward, X is left/right
        float forward = joystickInput.y; 
        float strafe = 0f; 
        float turn = -joystickInput.x * turnMultiplier;

        // Mecanum Math
        float fl_speed = (forward + strafe - turn) * speed * -1;
        float fr_speed = (forward - strafe + turn) * speed * -1;
        float bl_speed = (forward - strafe - turn) * speed;
        float br_speed = (forward + strafe + turn) * speed;

        // Apply Speed
        SetTargetVelocity(frontLeft, fl_speed);
        SetTargetVelocity(frontRight, fr_speed);
        SetTargetVelocity(backLeft, bl_speed);
        SetTargetVelocity(backRight, br_speed);
    }

    void SetTargetVelocity(ArticulationBody joint, float velocity)
    {
        var drive = joint.xDrive;
        drive.targetVelocity = velocity;
        joint.xDrive = drive;
    }
}