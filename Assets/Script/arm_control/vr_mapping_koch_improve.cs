using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class vr_mapping_koch_improve : MonoBehaviour
{
    [Header("=== Target Objects ===")]
    [Tooltip("機器手臂末端執行器的目標 (FABRIK 的 Target)")]
    public Transform targetEndEffector;

    [Tooltip("Pitch 控制目標")]
    public GameObject targetPitch;

    [Tooltip("Roll 控制目標")]
    public GameObject targetRoll;

    [Header("=== Control Sources ===")]
    [Tooltip("XR Hands 的手腕關節 (例如 L_Wrist)")]
    public Transform handWrist;

    [Header("=== Position Settings ===")]
    [Tooltip("位置偏移 (加在手腕位置上)")]
    public Vector3 positionOffset = Vector3.zero;

    [Tooltip("是否啟用位置映射")]
    public bool enablePositionMapping = true;

    [Tooltip("是否啟用旋轉映射")]
    public bool enableRotationMapping = true;

    [Header("=== Rotation Settings ===")]
    [Tooltip("旋轉偏移 (Euler 角度)")]
    public Vector3 rotationOffset = Vector3.zero;

    [Tooltip("旋轉角度限制 (X軸: Pitch)")]
    public Vector2 rotationLimitsX = new Vector2(-90f, 90f);

    [Tooltip("旋轉角度限制 (Y軸: Yaw)")]
    public Vector2 rotationLimitsY = new Vector2(-180f, 180f);

    [Tooltip("旋轉角度限制 (Z軸: Roll)")]
    public Vector2 rotationLimitsZ = new Vector2(-160f, 160f);

    [Tooltip("平滑速度")]
    public float smoothSpeed = 10f;

    [Tooltip("是否使用平滑插值")]
    public bool useSmoothing = true;

    [Header("=== Pitch/Roll Settings ===")]
    [Tooltip("是否啟用獨立的 Pitch/Roll 控制")]
    public bool enablePitchRollControl = false;

    [Tooltip("Pitch 角度偏移 (joint4: 手腕俯仰)")]
    public float pitchOffset = -90f;

    [Tooltip("Roll 角度偏移 (joint5: 手腕翻轉)")]
    public float rollOffset = 90f;

    [Header("=== Debug ===")]
    public bool debugMessages = false;

    [Header("=== Debug Info (Read Only) ===")]
    [Tooltip("VR 手腕的世界座標位置")]
    [SerializeField] private Vector3 _debugHandWorldPos;

    [Tooltip("位置偏移量")]
    [SerializeField] private Vector3 _debugRelativePos;

    [Tooltip("計算後要設給 targetEndEffector 的位置")]
    [SerializeField] private Vector3 _debugTargetPos;

    [Tooltip("targetEndEffector 實際的位置")]
    [SerializeField] private Vector3 _debugActualEndEffectorPos;

    [Tooltip("VR 手腕的旋轉角度 (X, Y, Z)")]
    [SerializeField] private Vector3 _debugHandEuler;

    [Tooltip("腳本是否正在執行 Update")]
    [SerializeField] private bool _debugIsUpdating = false;

    private Quaternion targetRotation;
    private Vector3 targetPosition;

    void Start()
    {
        ValidateReferences();
    }

    void ValidateReferences()
    {
        if (handWrist == null)
            Debug.LogError("[VR_mapping_koch] handWrist 尚未被指定！");
        if (targetEndEffector == null)
            Debug.LogError("[VR_mapping_koch] targetEndEffector 尚未被指定！");
        if (targetPitch == null)
            Debug.LogError("[VR_mapping_koch] targetPitch 尚未被指定！");
        if (targetRoll == null)
            Debug.LogError("[VR_mapping_koch] targetRoll 尚未被指定！");
    }

    void Update()
    {
        if (handWrist == null || targetEndEffector == null)
        {
            _debugIsUpdating = false;
            return;
        }

        _debugIsUpdating = true;
        _debugHandWorldPos = handWrist.position;
        _debugHandEuler = handWrist.eulerAngles;
        _debugActualEndEffectorPos = targetEndEffector.position;

        if (enablePositionMapping)
        {
            UpdateEndEffectorPosition();
        }

        if (enableRotationMapping)
        {
            UpdateEndEffectorRotation();
        }

        if (enablePitchRollControl)
        {
            UpdatePitch();
            UpdateRoll();
        }
    }

    void UpdateEndEffectorPosition()
    {
        targetPosition = handWrist.position + positionOffset;
        _debugTargetPos = targetPosition;
        _debugRelativePos = positionOffset;

        if (useSmoothing)
        {
            targetEndEffector.position = Vector3.Lerp(
                targetEndEffector.position,
                targetPosition,
                Time.deltaTime * smoothSpeed
            );
        }
        else
        {
            targetEndEffector.position = targetPosition;
        }

        if (debugMessages)
        {
            Debug.Log("[Position] Hand: " + handWrist.position + ", Target: " + targetPosition);
        }
    }

    void UpdateEndEffectorRotation()
    {
        Vector3 euler = handWrist.eulerAngles;

        float normalizedX = NormalizeAngle(euler.x);
        float normalizedY = NormalizeAngle(euler.y);
        float normalizedZ = NormalizeAngle(euler.z);

        normalizedX += rotationOffset.x;
        normalizedY += rotationOffset.y;
        normalizedZ += rotationOffset.z;

        normalizedX = Mathf.Clamp(normalizedX, rotationLimitsX.x, rotationLimitsX.y);
        normalizedY = Mathf.Clamp(normalizedY, rotationLimitsY.x, rotationLimitsY.y);
        normalizedZ = Mathf.Clamp(normalizedZ, rotationLimitsZ.x, rotationLimitsZ.y);

        targetRotation = Quaternion.Euler(normalizedX, normalizedY, normalizedZ);

        if (useSmoothing)
        {
            targetEndEffector.rotation = Quaternion.Slerp(
                targetEndEffector.rotation,
                targetRotation,
                Time.deltaTime * smoothSpeed
            );
        }
        else
        {
            targetEndEffector.rotation = targetRotation;
        }

        if (debugMessages)
        {
            Debug.Log("[Rotation] X=" + normalizedX + ", Y=" + normalizedY + ", Z=" + normalizedZ);
        }
    }

    void UpdatePitch()
    {
        if (targetPitch == null) return;

        float xVRr = handWrist.eulerAngles.x;
        float PitchAngle_targetPitch = xVRr + pitchOffset;

        if (debugMessages)
        {
            Debug.Log("[Pitch] PitchAngle_targetPitch: " + PitchAngle_targetPitch);
        }

        targetPitch.transform.rotation = Quaternion.Euler(
            targetPitch.transform.eulerAngles.x,
            targetPitch.transform.eulerAngles.y,
            PitchAngle_targetPitch
        );
    }

    void UpdateRoll()
    {
        if (targetRoll == null) return;

        float zVRr = handWrist.eulerAngles.z;
        float rollAngle_targetRoll = -zVRr + rollOffset;

        if (debugMessages)
        {
            Debug.Log("[Roll] rollAngle: " + rollAngle_targetRoll);
        }

        Quaternion rollRotation = Quaternion.AngleAxis(rollAngle_targetRoll, Vector3.right);
        targetRoll.transform.localRotation = rollRotation;
    }

    private float NormalizeAngle(float angle)
    {
        angle = angle % 360f;
        if (angle > 180f) angle -= 360f;
        return angle;
    }

    void OnDrawGizmos()
    {
        if (handWrist != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(handWrist.position, 0.02f);
            DrawAxisGizmo(handWrist, 0.1f);
        }

        if (targetEndEffector != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(targetEndEffector.position, 0.02f);
        }

        if (handWrist != null && targetEndEffector != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(handWrist.position, targetEndEffector.position);
        }
    }

    void DrawAxisGizmo(Transform t, float size)
    {
        Gizmos.color = Color.red;
        Gizmos.DrawLine(t.position, t.position + t.right * size);
        Gizmos.color = Color.green;
        Gizmos.DrawLine(t.position, t.position + t.up * size);
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(t.position, t.position + t.forward * size);
    }
}
