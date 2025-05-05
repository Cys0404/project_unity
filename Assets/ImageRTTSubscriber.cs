using UnityEngine;
using TMPro;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;

public class ImageRTTSubscriber : MonoBehaviour
{
    [Header("影像 RTT 顯示 UI")]
    public TextMeshProUGUI imageRTTText;

    private ROSConnection ros;
    private string imageRTTTopic = "/image_rtt";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32Msg>(imageRTTTopic, OnReceiveImageRTT);
    }

    void OnReceiveImageRTT(Float32Msg msg)
    {
        float rtt = msg.data;
        imageRTTText.text = $"RTT-Image: {rtt:F1} ms";
        Debug.Log($"收到 /image_rtt：{rtt:F1} ms");
    }
}
