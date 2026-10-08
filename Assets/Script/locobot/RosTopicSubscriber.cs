using UnityEngine;
using RosSharp.RosBridgeClient;

// 注意：這裡使用 ROS# 專用的 Message 型別
namespace RosSharp.RosBridgeClient
{
    public class TCPTopicSubscriber : UnitySubscriber<MessageTypes.Geometry.Twist>
    {
        protected override void Start()
        {
            // 設定你要訂閱的 Topic 名稱 (預設為 /cmd_vel)
            Topic = "/cmd_vel";
            base.Start();
        }

        // 當收到 ROS 端傳來的 Twist 速度訊息時觸發
        protected override void ReceiveMessage(MessageTypes.Geometry.Twist message)
        {
            float linearX = (float)message.linear.x;
            float angularZ = (float)message.angular.z;

            Debug.Log($"[ROS# Subscriber] 收到 Topic ({Topic}) -> 線速度 Linear.X: {linearX:F2}, 角速度 Angular.Z: {angularZ:F2}");
        }
    }
}