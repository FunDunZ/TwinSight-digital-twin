using UnityEngine;

public class MecanumTeleop : MonoBehaviour
{
    public ArticulationBody frontLeft, frontRight, backLeft, backRight;
    public float speed = 10.0f; // Set this back to a normal number like 10
    public float rotationSpeed = 100.0f;

    void Start()
    {
        // ---------------------------------------------------------
        // AUTOMATIC PHYSICS FIX
        // This runs once when you hit Play to unlock the "brakes"
        // ---------------------------------------------------------
        SetupWheel(frontLeft);
        SetupWheel(frontRight);
        SetupWheel(backLeft);
        SetupWheel(backRight);
    }

    void SetupWheel(ArticulationBody wheel)
    {
        // 1. Get the current drive settings
        var drive = wheel.xDrive;

        // 2. FORCE the damping down from "Infinity" to 100
        drive.damping = 100f;     // The "Parking Brake" is now off
        drive.stiffness = 0f;     // No springiness
        drive.forceLimit = 10000f; // Lots of torque available

        // 3. Ensure we are in Velocity mode
        drive.driveType = ArticulationDriveType.Velocity;

        // 4. Apply the changes back to the wheel
        wheel.xDrive = drive;
    }

    void Update()
    {
        // Standard Input
        float forward = Input.GetAxis("Vertical");
        float strafe = Input.GetAxis("Horizontal");
        float turn = 0;

        if (Input.GetKey(KeyCode.Q)) turn = 2f;
        if (Input.GetKey(KeyCode.E)) turn = -2f;

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
