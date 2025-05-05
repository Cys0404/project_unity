using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using TMPro;

public class NetworkStatsDisplay : MonoBehaviour
{
    public TextMeshProUGUI latencyText;
    public TextMeshProUGUI throughputText;
    public TextMeshProUGUI packetLossText;
    public TextMeshProUGUI jitterText;
    public TextMeshProUGUI fpsText;

    private float deltaTime = 0.0f;

    void Start()
    {
        // 訂閱 ROS topic
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>("/network_test/response", OnReceiveStats);
    }

    void OnReceiveStats(StringMsg msg)
    {
        // 預期格式範例：
        // "2025-05-01 20:00:00, Control Response, 113.456, 50.0, 56.728, 2, 3.0"
        var parts = msg.data.Split(',');
        if (parts.Length < 7)
        {
            Debug.LogWarning("回傳格式錯誤: " + msg.data);
            return;
        }

        // 取出各項資訊並顯示
        string latency = parts[4].Trim();
        string throughput = parts[3].Trim();
        string packetLoss = parts[5].Trim();
        string jitter = parts[6].Trim();

        if (latencyText != null) latencyText.text = $"Latency: {latency} ms";
        if (throughputText != null) throughputText.text = $"Throughput: {throughput} Mbps";
        if (packetLossText != null) packetLossText.text = $"Pkt Loss: {packetLoss}";
        if (jitterText != null) jitterText.text = $"Jitter: {jitter} ms";
    }

    void Update()
    {
        // 顯示 FPS
        deltaTime += (Time.unscaledDeltaTime - deltaTime) * 0.1f;
        float fps = 1.0f / deltaTime;
        if (fpsText != null) fpsText.text = $"FPS: {fps:F1}";
    }
}
