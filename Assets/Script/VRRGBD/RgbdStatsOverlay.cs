using System.Text;
using TMPro;
using UnityEngine;

namespace VRRGBD
{
    public sealed class RgbdStatsOverlay : MonoBehaviour
    {
        public RgbdCameraManager cameraManager;
        public RgbdTcpReceiver receiver;
        public TextMeshProUGUI text;
        public float updateInterval = 0.25f;

        private readonly StringBuilder builder = new StringBuilder(1024);
        private float nextUpdate;
        private float smoothedDelta;

        private void Awake()
        {
            if (cameraManager == null) cameraManager = FindObjectOfType<RgbdCameraManager>();
            if (receiver == null) receiver = FindObjectOfType<RgbdTcpReceiver>();
        }

        private void Update()
        {
            smoothedDelta += (Time.unscaledDeltaTime - smoothedDelta) * 0.1f;
            if (text == null || Time.unscaledTime < nextUpdate) return;
            nextUpdate = Time.unscaledTime + updateInterval;

            builder.Clear();
            float fps = smoothedDelta > 0f ? 1f / smoothedDelta : 0f;
            builder.Append("VR FPS: ").Append(fps.ToString("F1"));
            if (receiver != null)
            {
                builder.Append("  TCP: ").Append(receiver.IsConnected ? "Connected" : "Disconnected");
                builder.Append("  RX: ").Append(receiver.ReceivedFrames);
                builder.Append("  Drop: ").Append(receiver.DroppedFrames);
            }

            if (cameraManager != null)
            {
                foreach (var kvp in cameraManager.Stats)
                {
                    var s = kvp.Value;
                    builder.AppendLine();
                    builder.Append(s.cameraId)
                        .Append("  fps ").Append(s.streamFps.ToString("F1"))
                        .Append("  latency ").Append(s.latencyMs.ToString("F1")).Append(" ms")
                        .Append("  pts ").Append(s.pointCount)
                        .Append("  align ").Append(s.aligned ? "OK" : "WAIT");
                }
            }

            text.text = builder.ToString();
        }
    }
}
