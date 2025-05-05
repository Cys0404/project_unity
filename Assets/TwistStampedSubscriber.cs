using RosMessageTypes.Std;
using UnityEngine;
using TMPro;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;

public class TwistStampedSubscriber : MonoBehaviour
{
    [Header("控制 RTT 顯示 UI")]
    public TextMeshProUGUI controlRTTText;

    public float sendInterval = 1.0f;
    private float timeSinceLastSend = 0.0f;

    private ROSConnection ros;
    private string controlTopic = "/twist_control";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<TwistStampedMsg>(controlTopic);
        ros.Subscribe<TwistStampedMsg>(controlTopic, OnReceiveControl);
    }

    void Update()
    {
        timeSinceLastSend += Time.deltaTime;
        if (timeSinceLastSend >= sendInterval)
        {
            PublishControlMessage();
            timeSinceLastSend = 0f;
        }
    }

    void PublishControlMessage()
    {
        double now = Time.realtimeSinceStartupAsDouble;

        var rosTime = new TimeMsg
        {
            sec = (int)now,
            nanosec = (uint)((now % 1.0) * 1e9)
        };

        var msg = new TwistStampedMsg
        {
            header = new HeaderMsg
            {
                stamp = rosTime,
                frame_id = "unity_control"
            },
            twist = new TwistMsg()
        };

        ros.Publish(controlTopic, msg);
        Debug.Log("發送 /twist_control");
    }

    void OnReceiveControl(TwistStampedMsg msg)
    {
        double now = Time.realtimeSinceStartupAsDouble;
        double msgTime = (double)msg.header.stamp.sec + (double)msg.header.stamp.nanosec / 1e9;
        double rtt = (now - msgTime) * 1000.0;

        controlRTTText.text = $"RTT-Control: {rtt:F1} ms";
        Debug.Log($"收到 /twist_control，RTT: {rtt:F1} ms");
    }
}
