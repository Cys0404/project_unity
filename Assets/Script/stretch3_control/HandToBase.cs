using UnityEngine;
using UnityEngine.XR.Hands;
using Unity.Mathematics;
using UnityEngine.Serialization;   // 為了 FormerlySerializedAs
using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public class HandToBase : UnityPublisher<MessageTypes.Geometry.Twist>
    {
        [Header("Hand Tracking")]
        public XRHandSubsystem handSubsystem;

        [Header("Per-Finger Thresholds (meters)")]
        [Tooltip("食指：<=觸發，>=釋放（回滯）")]
        public float indexTouch = 0.030f, indexRelease = 0.040f;
        [Tooltip("中指：<=觸發，>=釋放（回滯）")]
        public float middleTouch = 0.030f, middleRelease = 0.040f;
        [Tooltip("無名指：<=觸發，>=釋放（回滯）—放寬一點")]
        public float ringTouch = 0.040f, ringRelease = 0.055f;
        [Tooltip("小拇指：<=觸發，>=釋放（回滯）—放寬一點")]
        public float littleTouch = 0.042f, littleRelease = 0.058f;

        [Header("Gesture Filtering")]
        [Tooltip("手勢判讀最短間隔（秒）")]
        public float gestureDebounceTime = 0.10f;

        [Header("Movement Settings")]
        [FormerlySerializedAs("max_speed")]
        [Tooltip("前/後直線速度（m/s）")]
        public float linearSpeed = 0.25f;     // 比 1.0 溫和許多
        [Tooltip("旋轉角速度（rad/s）")]
        public float angularSpeed = 0.60f;

        [Header("Debug")]
        public bool debug_message = false;

        private MessageTypes.Geometry.Twist message;
        private float _lastDecisionTime;
        // 帶回滯的手勢狀態
        private bool pinchIndex, pinchMiddle, pinchRing, pinchLittle;

        protected override void Start()
        {
            base.Start();
            message = new MessageTypes.Geometry.Twist
            {
                linear = new MessageTypes.Geometry.Vector3(),
                angular = new MessageTypes.Geometry.Vector3()
            };

            // 若沒手動指定，抓第一個 XRHandSubsystem
            if (handSubsystem == null)
            {
                var list = new List<XRHandSubsystem>();
                SubsystemManager.GetSubsystems(list);
                if (list.Count > 0) handSubsystem = list[0];
            }
        }

        private void Update()
        {
            if (handSubsystem == null || !handSubsystem.running) return;

            var left = handSubsystem.leftHand;
            if (!left.isTracked) return;

            if (!TryGetJointPos(left, XRHandJointID.ThumbTip, out float3 thumb)) return;
            if (!TryGetJointPos(left, XRHandJointID.IndexTip, out float3 index)) return;
            if (!TryGetJointPos(left, XRHandJointID.MiddleTip, out float3 middle)) return;
            if (!TryGetJointPos(left, XRHandJointID.RingTip, out float3 ring)) return;
            if (!TryGetJointPos(left, XRHandJointID.LittleTip, out float3 little)) return;

            // 依各手指門檻（含回滯）更新狀態
            UpdatePinch(ref pinchIndex, math.distance(thumb, index), indexTouch, indexRelease);
            UpdatePinch(ref pinchMiddle, math.distance(thumb, middle), middleTouch, middleRelease);
            UpdatePinch(ref pinchRing, math.distance(thumb, ring), ringTouch, ringRelease);
            UpdatePinch(ref pinchLittle, math.distance(thumb, little), littleTouch, littleRelease);

            // 防抖：限制判讀頻率
            if (Time.time - _lastDecisionTime < gestureDebounceTime) return;
            _lastDecisionTime = Time.time;

            // 產生 Twist：預設停止
            message.linear.x = 0f;
            message.angular.z = 0f;

            // 手勢優先序：食指 > 中指 > 無名指 > 小拇指
            if (pinchIndex) message.linear.x = linearSpeed;   // 前進
            else if (pinchMiddle) message.linear.x = -linearSpeed;   // 後退
            else if (pinchRing) message.angular.z = -angularSpeed;  // 順時針（右轉）
            else if (pinchLittle) message.angular.z = angularSpeed;  // 逆時針（左轉）

            if (debug_message)
            {
                Debug.Log($"[HandToBase] I:{pinchIndex} M:{pinchMiddle} R:{pinchRing} L:{pinchLittle} | " +
                          $"linX={message.linear.x:F2} m/s  angZ={message.angular.z:F2} rad/s");
            }

            Publish(message);
        }

        // XR Hands 1.4.3：GetJoint + TryGetPose
        private static bool TryGetJointPos(XRHand hand, XRHandJointID id, out float3 pos)
        {
            pos = default;
            XRHandJoint joint = hand.GetJoint(id);
            if (!joint.TryGetPose(out Pose p)) return false;
            Vector3 v = p.position;
            pos = new float3(v.x, v.y, v.z);
            return true;
        }

        private static void UpdatePinch(ref bool state, float dist, float touch, float release)
        {
            if (!state && dist <= touch) state = true;           // 進入接觸
            else if (state && dist >= release) state = false;    // 退出接觸
        }
    }
}
