using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; 

public class VRVideoFeed : MonoBehaviour
{
    [Header("ROS Settings")]
    public string imageTopic = "/image_raw/compressed";

    [Header("UI Display")]
    public RawImage displayScreen;

    [Header("Optimization")]
    [Tooltip("Maximum frames to draw per second to prevent VR lag")]
    public float maxFPS = 15f; // Throttle the UI to 15 FPS

    private ROSConnection ros;
    private Texture2D videoTexture;

    // Buffer variables to separate receiving from drawing
    private byte[] latestImageData = null;
    private bool isNewFrameAvailable = false;
    private float timeSinceLastFrame = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CompressedImageMsg>(imageTopic, ReceiveImage);

        videoTexture = new Texture2D(1, 1);
        if (displayScreen != null) displayScreen.texture = videoTexture;
    }

    // 1. THE RECEIVER: This just catches the data and puts it in the bucket.
    // It does NOT do the heavy lifting of decoding the image.
    void ReceiveImage(CompressedImageMsg msg)
    {
        if (msg.data != null && msg.data.Length > 0)
        {
            latestImageData = msg.data;
            isNewFrameAvailable = true; 
        }
    }

    // 2. THE MAIN LOOP: This is tied to your VR headset framerate.
    // It only decodes the image if enough time has passed.
    void Update()
    {
        timeSinceLastFrame += Time.deltaTime;
        
        // Calculate the cooldown (e.g., 1/15 = 0.066 seconds per frame)
        float frameCooldown = 1f / maxFPS; 

        if (isNewFrameAvailable && timeSinceLastFrame >= frameCooldown)
        {
            // Now we do the heavy lifting
            videoTexture.LoadImage(latestImageData);
            videoTexture.Apply();

            // Reset the flags and timer
            isNewFrameAvailable = false;
            timeSinceLastFrame = 0f;
        }
    }
}