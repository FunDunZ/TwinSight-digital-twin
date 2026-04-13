using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor; // Required to read the CompressedImageMsg

public class VRVideoFeed : MonoBehaviour
{
    [Header("ROS Settings")]
    public string imageTopic = "/image_raw/compressed";

    [Header("UI Display")]
    [Tooltip("Drag the Raw Image component from your Canvas here")]
    public RawImage displayScreen;

    private ROSConnection ros;
    private Texture2D videoTexture;

    void Start()
    {
        // 1. Establish the bridge to the Pi
        ros = ROSConnection.GetOrCreateInstance();
        
        // 2. Subscribe to the camera topic
        ros.Subscribe<CompressedImageMsg>(imageTopic, ReceiveImage);

        // 3. Initialize a blank texture (size doesn't matter, LoadImage will resize it)
        videoTexture = new Texture2D(1, 1);
        
        // 4. Assign the texture to your virtual monitor
        if (displayScreen != null)
        {
            displayScreen.texture = videoTexture;
        }
        else
        {
            Debug.LogError("VRVideoFeed: You forgot to assign the Raw Image in the Inspector!");
        }
    }

    // This function triggers automatically every time a new frame arrives
    void ReceiveImage(CompressedImageMsg msg)
    {
        if (msg.data != null && msg.data.Length > 0)
        {
            // Debug.Log($"Incoming Video Format: {msg.format}");
            // Unity's built-in LoadImage function automatically decodes the JPG byte array 
            // from the ROS message and turns it into a visible texture!
            videoTexture.LoadImage(msg.data);
            videoTexture.Apply();
        }
    }
}