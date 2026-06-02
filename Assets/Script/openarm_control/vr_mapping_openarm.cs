using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class vr_mapping_openarm123 : MonoBehaviour
{
    [Header("=== Target Objects ===")]
    [Tooltip("FABRIK IK Target")]
    public Transform targetEndEffector;

    [Header("=== Control Sources ===")]
    [Tooltip("XR Hands wrist transform (e.g. L_Wrist)")]
    public Transform handWrist;

    [Header("=== Position Settings ===")]
    [Tooltip("Enable position mapping")]
    public bool enablePositionMapping = true;

    [Tooltip("Enable rotation mapping")]
    public bool enableRotationMapping = true;

    [Header("=== Rotation Settings ===")]
    [Tooltip("Smoothing speed")]
    public float smoothSpeed = 10f;

    [Tooltip("Enable smoothing")]
    public bool useSmoothing = true;

    [Header("=== Debug ===")]
    public bool debugMessages = false;

    [Header("=== Debug Info (Read Only) ===")]
    [SerializeField] private Vector3 _debugHandWorldPos;
    [SerializeField] private Vector3 _debugTargetPos;
    [SerializeField] private Vector3 _debugHandEuler;
    [SerializeField] private bool _debugIsUpdating = false;
    [SerializeField] private bool _isHandTrackingInitialized = false;

    private Quaternion targetRotation;
    private Vector3 targetPosition;
    private Vector3 _lastHandPosition;

    void Start()
    {
        ValidateReferences();
    }

    void ValidateReferences()
    {
        if (handWrist == null)
            Debug.LogError("[vr_mapping_openarm] handWrist is not assigned!");
        if (targetEndEffector == null)
            Debug.LogError("[vr_mapping_openarm] targetEndEffector is not assigned!");
    }

    void Update()
    {
        if (handWrist == null || targetEndEffector == null)
        {
            _debugIsUpdating = false;
            return;
        }

        if (!_isHandTrackingInitialized)
        {
            if (_lastHandPosition != Vector3.zero &&
                Vector3.Distance(handWrist.position, _lastHandPosition) > 0.001f)
            {
                _isHandTrackingInitialized = true;
                if (debugMessages)
                    Debug.Log("[vr_mapping_openarm] XR Hands initialized!");
            }
            _lastHandPosition = handWrist.position;
            return;
        }

        _debugIsUpdating = true;
        _debugHandWorldPos = handWrist.position;
        _debugHandEuler = handWrist.eulerAngles;

        if (enablePositionMapping)
            UpdateEndEffectorPosition();

        if (enableRotationMapping)
            UpdateEndEffectorRotation();
    }

    void UpdateEndEffectorPosition()
    {
        targetPosition = handWrist.position;
        _debugTargetPos = targetPosition;

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
            Debug.Log("[Position] Hand: " + handWrist.position + ", Target: " + targetPosition);
    }

    void UpdateEndEffectorRotation()
    {
        Vector3 euler = handWrist.eulerAngles;

        float normalizedX = NormalizeAngle(euler.x);
        float normalizedY = NormalizeAngle(euler.y);
        float normalizedZ = NormalizeAngle(euler.z);

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
            Debug.Log("[Rotation] X=" + normalizedX + ", Y=" + normalizedY + ", Z=" + normalizedZ);
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
