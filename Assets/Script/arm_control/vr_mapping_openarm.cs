using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class vr_mapping_openarm : MonoBehaviour
{
    [Header("=== Target Objects ===")]
    [Tooltip("FABRIK IK Target")]
    public Transform targetEndEffector;

    [Header("=== Control Sources ===")]
    [Tooltip("XR Hands wrist transform (e.g. L_Wrist)")]
    public Transform handWrist;

    [Header("=== Cylinder Safety Guard (圓柱防撞機制) ===")]
    [Tooltip("請放入機器人底座 (openarm_body_link0)，將以此為圓柱中心")]
    public Transform openarmBodyLink0;

    [Tooltip("【防撞自己】圓柱體的半徑 (公尺)。只在水平方向推開，不影響上下移動！")]
    public float cylinderRadius = 0.15f;

    [Tooltip("【防脫臼】手臂最長極限距離 (公尺) - 這是3D球體限制")]
    public float maxReach = 0.5f;

    [Tooltip("【防砸桌子】絕對地板高度 (世界座標 Y)")]
    public float minHeight = 0.05f;

    [Header("=== Position / Rotation Settings ===")]
    public bool enablePositionMapping = true;
    public bool enableRotationMapping = true;
    public float smoothSpeed = 10f;
    public bool useSmoothing = true;

    [Header("=== Debug ===")]
    public bool debugMessages = false;

    private Quaternion targetRotation;
    private Vector3 targetPosition;
    private Vector3 _lastHandPosition;
    private bool _isHandTrackingInitialized = false;

    void Start()
    {
        if (handWrist == null) Debug.LogError("[vr_mapping_openarm] handWrist is not assigned!");
        if (targetEndEffector == null) Debug.LogError("[vr_mapping_openarm] targetEndEffector is not assigned!");
        if (openarmBodyLink0 == null) Debug.LogWarning("[vr_mapping_openarm] openarmBodyLink0 尚未綁定！防撞機制將不會啟動。");
    }

    void Update()
    {
        if (handWrist == null || targetEndEffector == null) return;

        if (!_isHandTrackingInitialized)
        {
            if (_lastHandPosition != Vector3.zero && Vector3.Distance(handWrist.position, _lastHandPosition) > 0.001f)
            {
                _isHandTrackingInitialized = true;
            }
            _lastHandPosition = handWrist.position;
            return;
        }

        if (enablePositionMapping) UpdateEndEffectorPosition();
        if (enableRotationMapping) UpdateEndEffectorRotation();
    }

    void UpdateEndEffectorPosition()
    {
        Vector3 boundedPosition = handWrist.position;

        if (openarmBodyLink0 != null)
        {
            // 1. 【防砸桌】優先處理地板高度限制 (Y軸)
            if (boundedPosition.y < minHeight)
            {
                boundedPosition.y = minHeight;
            }

            // 2. 【核心：圓柱體防撞】(只計算 X 和 Z 軸的水平距離)
            Vector3 center2D = new Vector3(openarmBodyLink0.position.x, 0, openarmBodyLink0.position.z);
            Vector3 target2D = new Vector3(boundedPosition.x, 0, boundedPosition.z);

            float dist2D = Vector3.Distance(center2D, target2D);

            if (dist2D < cylinderRadius)
            {
                // 如果進入圓柱體內，只在水平方向(XZ)往外推
                Vector3 pushDir2D = (target2D - center2D).normalized;
                if (pushDir2D == Vector3.zero) pushDir2D = Vector3.forward; // 防止奇異點

                // 修改 X 和 Z，但完美保留原本的 Y (高度)！
                boundedPosition.x = center2D.x + pushDir2D.x * cylinderRadius;
                boundedPosition.z = center2D.z + pushDir2D.z * cylinderRadius;
            }

            // 3. 【防脫臼】最大伸展極限 (維持 3D 距離判斷)
            float distToShoulder = Vector3.Distance(openarmBodyLink0.position, boundedPosition);
            if (distToShoulder > maxReach)
            {
                Vector3 pullDir = (boundedPosition - openarmBodyLink0.position).normalized;
                boundedPosition = openarmBodyLink0.position + pullDir * maxReach;
            }
        }

        targetPosition = boundedPosition;

        if (useSmoothing)
        {
            targetEndEffector.position = Vector3.Lerp(targetEndEffector.position, targetPosition, Time.deltaTime * smoothSpeed);
        }
        else
        {
            targetEndEffector.position = targetPosition;
        }
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
            targetEndEffector.rotation = Quaternion.Slerp(targetEndEffector.rotation, targetRotation, Time.deltaTime * smoothSpeed);
        }
        else
        {
            targetEndEffector.rotation = targetRotation;
        }
    }

    private float NormalizeAngle(float angle)
    {
        angle = angle % 360f;
        if (angle > 180f) angle -= 360f;
        return angle;
    }

    void OnDrawGizmos()
    {
        if (openarmBodyLink0 != null)
        {
            // 畫出紅色的「圓柱體力場」輔助線
            Gizmos.color = Color.red;
            int segments = 24;
            float heightDraw = 0.5f; // 僅供視覺參考的圓柱高度
            for (int i = 0; i < segments; i++)
            {
                float angle = i * Mathf.PI * 2 / segments;
                float nextAngle = (i + 1) * Mathf.PI * 2 / segments;

                Vector3 p1 = openarmBodyLink0.position + new Vector3(Mathf.Cos(angle) * cylinderRadius, 0, Mathf.Sin(angle) * cylinderRadius);
                Vector3 p2 = openarmBodyLink0.position + new Vector3(Mathf.Cos(nextAngle) * cylinderRadius, 0, Mathf.Sin(nextAngle) * cylinderRadius);

                // 畫底部圓圈、頂部圓圈、和垂直柱子
                Gizmos.DrawLine(p1, p2);
                Gizmos.DrawLine(p1 + Vector3.up * heightDraw, p2 + Vector3.up * heightDraw);
                if (i % 4 == 0) Gizmos.DrawLine(p1, p1 + Vector3.up * heightDraw);
            }

            // 畫出綠色的防脫臼外圈
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(openarmBodyLink0.position, maxReach);
        }

        if (handWrist != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(handWrist.position, 0.02f);
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
}