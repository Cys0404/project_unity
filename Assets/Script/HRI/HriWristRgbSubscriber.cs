using System;
using System.Threading;
using RosSharp.RosBridgeClient;
using UnityEngine;

namespace HRI
{
    [RequireComponent(typeof(RosConnector))]
    public sealed class HriWristRgbSubscriber : MonoBehaviour
    {
        public string topic = "/gripper_camera/gripper_camera/color/image_rect_raw/compressed";
        public HriFloatingMenu menu;
        private RosConnector connector;
        private string subscription;
        private byte[] pending;
        private Texture2D texture;
        private void Awake() { connector = GetComponent<RosConnector>(); }
        private void Update()
        {
            if (subscription == null && connector.IsConnected.WaitOne(0))
                subscription = connector.RosSocket.Subscribe<RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage>(
                    topic, Receive, 100, queue_length: 1);
            byte[] image = Interlocked.Exchange(ref pending, null);
            if (image == null) return;
            if (texture == null) texture = new Texture2D(2, 2, TextureFormat.RGB24, false);
            if (texture.LoadImage(image, false) && menu != null) menu.wristView = texture;
        }
        private void Receive(RosSharp.RosBridgeClient.MessageTypes.Sensor.CompressedImage message)
        {
            if (message?.data == null || message.data.Length > 8 * 1024 * 1024) return;
            Interlocked.Exchange(ref pending, message.data);
        }
        private void OnDisable()
        {
            if (subscription != null && connector != null && connector.RosSocket != null)
            {
                try { connector.RosSocket.Unsubscribe(subscription); }
                catch (Exception error) { Debug.LogWarning("Wrist unsubscribe: " + error.Message); }
            }
            subscription = null; Interlocked.Exchange(ref pending, null);
        }
        private void OnDestroy() { if (texture != null) Destroy(texture); }
    }
}
