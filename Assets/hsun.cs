using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.SubsystemsImplementation;
using Unity.Robotics.ROSTCPConnector;
public class XRHandThumbsUpDrive : MonoBehaviour
{
    [Header("現有的 LoCoBot 移動控制器 (發送 ROS cmd_vel 的組件)")]
    [SerializeField] private MonoBehaviour locobotController;

    [Header("視角參考 (主相機，用於對齊前進方向)")]
    [SerializeField] private Transform headCamera;

    [Header("比讚手勢判斷閾值 (單位: 公尺)")]
    [Tooltip("食指~小指指尖到手腕的距離小於此值視為握拳")]
    [SerializeField] private float fingerCurlDistance = 0.08f;
    [Tooltip("拇指指尖到手腕的距離大於此值視為拇指豎起")]
    [SerializeField] private float thumbExtendDistance = 0.10f;

    [Header("拇指傾斜角度靈敏度")]
    [Tooltip("拇指偏離垂直向上的死區角度 (度)")]
    [SerializeField] private float angleDeadZone = 12f;
    [Tooltip("拇指傾斜達到此角度時輸出最大速度 (度)")]
    [SerializeField] private float maxTiltAngle = 45f;

    [Header("速度限制")]
    [SerializeField] private float maxLinearSpeed = 0.4f;   // 公尺/秒
    [SerializeField] private float maxAngularSpeed = 1.0f;  // 弧度/秒

    private XRHandSubsystem handSubsystem;
    private bool isDriving = false;

    void Start()
    {
        if (headCamera == null && Camera.main != null)
        {
            headCamera = Camera.main.transform;
        }
    }

    void Update()
    {
        // 1. 取得執行中的 XRHandSubsystem
        if (handSubsystem == null || !handSubsystem.running)
        {
            var handSubsystems = new List<XRHandSubsystem>();
            SubsystemManager.GetSubsystems(handSubsystems);
            foreach (var sub in handSubsystems)
            {
                if (sub.running)
                {
                    handSubsystem = sub;
                    break;
                }
            }
            return;
        }

        // 2. 追蹤右手
        XRHand hand = handSubsystem.rightHand;
        if (!hand.isTracked)
        {
            if (isDriving) StopDriving();
            return;
        }

        // 3. 比讚手勢判定
        if (IsThumbsUp(hand, out Vector3 thumbDirection))
        {
            isDriving = true;
            ProcessThumbSteering(thumbDirection);
        }
        else
        {
            if (isDriving)
            {
                StopDriving();
            }
        }
    }

    /// <summary>
    /// 判斷右手是否為比讚姿態：四指握拳，拇指伸直
    /// </summary>
    private bool IsThumbsUp(XRHand hand, out Vector3 thumbDirection)
    {
        thumbDirection = Vector3.zero;

        // 取得手腕與拇指關節
        if (!hand.GetJoint(XRHandJointID.Wrist).TryGetPose(out Pose wristPose) ||
            !hand.GetJoint(XRHandJointID.ThumbTip).TryGetPose(out Pose thumbTipPose) ||
            !hand.GetJoint(XRHandJointID.ThumbProximal).TryGetPose(out Pose thumbProximalPose))
        {
            return false;
        }

        // 檢查四指是否都處於彎曲握拳狀態 (Tip 靠近 Wrist)
        XRHandJointID[] fingerTips = new XRHandJointID[]
        {
            XRHandJointID.IndexTip,
            XRHandJointID.MiddleTip,
            XRHandJointID.RingTip,
            XRHandJointID.LittleTip
        };

        foreach (var jointId in fingerTips)
        {
            if (hand.GetJoint(jointId).TryGetPose(out Pose tipPose))
            {
                if (Vector3.Distance(tipPose.position, wristPose.position) > fingerCurlDistance)
                {
                    return false; // 任一指沒握緊，判定不成立
                }
            }
        }

        // 檢查大拇指是否充分伸出
        float thumbDist = Vector3.Distance(thumbTipPose.position, wristPose.position);
        if (thumbDist < thumbExtendDistance)
        {
            return false; // 拇指沒伸直
        }

        // 計算拇指當前指向向量 (從近端關節指向指尖)
        thumbDirection = (thumbTipPose.position - thumbProximalPose.position).normalized;
        return true;
    }

    /// <summary>
    /// 根據拇指指向計算線速度與角速度
    /// </summary>
    private void ProcessThumbSteering(Vector3 thumbDir)
    {
        // 基準視角方向（消除相機抬頭/低頭的 Y 軸分量）
        Vector3 camForward = Vector3.ProjectOnPlane(headCamera != null ? headCamera.forward : Vector3.forward, Vector3.up).normalized;
        Vector3 camRight = Vector3.ProjectOnPlane(headCamera != null ? headCamera.right : Vector3.right, Vector3.up).normalized;

        // 拇指相對於正上方的傾斜情況
        float forwardDot = Vector3.Dot(thumbDir, camForward); // 向前/向後傾斜
        float lateralDot = Vector3.Dot(thumbDir, camRight);   // 向右/向左傾斜

        // 計算偏向角（度）
        float pitchAngle = Mathf.Asin(Mathf.Clamp(forwardDot, -1f, 1f)) * Mathf.Rad2Deg;
        float rollAngle = Mathf.Asin(Mathf.Clamp(lateralDot, -1f, 1f)) * Mathf.Rad2Deg;

        // 前後線速度計算
        float linear = 0f;
        if (Mathf.Abs(pitchAngle) > angleDeadZone)
        {
            float normLinear = Mathf.Clamp((Mathf.Abs(pitchAngle) - angleDeadZone) / (maxTiltAngle - angleDeadZone), 0f, 1f);
            linear = Mathf.Sign(pitchAngle) * normLinear * maxLinearSpeed;
        }

        // 左右轉向角速度計算
        float angular = 0f;
        if (Mathf.Abs(rollAngle) > angleDeadZone)
        {
            float normAngular = Mathf.Clamp((Mathf.Abs(rollAngle) - angleDeadZone) / (maxTiltAngle - angleDeadZone), 0f, 1f);
            angular = -Mathf.Sign(rollAngle) * normAngular * maxAngularSpeed;
        }

        SendVelocity(linear, angular);
    }

    private void StopDriving()
    {
        isDriving = false;
        SendVelocity(0f, 0f);
    }

    private void SendVelocity(float linear, float angular)
    {
        if (locobotController != null)
        {
            var keyboardScript = locobotController.GetComponent<KeyboardToBase>();
            if (keyboardScript == null)
            {
               // 移除型別前的 RosSharp.RosBridgeClient. 前綴
keyboardScript = locobotController.GetComponentInChildren<KeyboardToBase>();
            }

            if (keyboardScript != null)
            {
                keyboardScript.SetHandVelocity(linear, angular);
            }
            else
            {
                Debug.LogWarning("[ThumbsUp Teleop] 找不到 KeyboardToBase 元件！請確認 Inspector 中的 locobotController 拉線是否正確。");
            }
        }

        Debug.Log($"[ThumbsUp Teleop] Linear: {linear:F2} m/s, Angular: {angular:F2} rad/s");
    }

    void OnDisable()
    {
        if (isDriving) StopDriving();
    }
}