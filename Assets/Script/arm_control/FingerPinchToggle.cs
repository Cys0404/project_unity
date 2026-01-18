using UnityEngine;
using System.Collections.Generic;

public class FingerPinchToggle : MonoBehaviour
{
    [Header("Finger References")]
    [Tooltip("右手中指指尖的 Transform")]
    public Transform rightMiddleTip;

    [Tooltip("右手大拇指指尖的 Transform")]
    public Transform rightThumbTip;

    [Header("Target Scripts (Toggle List)")]
    [Tooltip("想要被啟用/停用的腳本，可自行新增多個")]
    public List<MonoBehaviour> targetScripts = new List<MonoBehaviour>();

    [Header("Pinch Settings")]
    [Tooltip("兩指距離小於這個值時視為捏合（單位:公尺）")]
    public float pinchDistanceThreshold = 0.02f;

    private bool isInsidePinch = false;

    private void Update()
    {
        if (rightMiddleTip == null || rightThumbTip == null)
            return;

        float distance = Vector3.Distance(rightMiddleTip.position, rightThumbTip.position);

        if (distance < pinchDistanceThreshold)
        {
            if (!isInsidePinch)
            {
                ToggleScripts();
                isInsidePinch = true;
            }
        }
        else
        {
            isInsidePinch = false;
        }
    }

    private void ToggleScripts()
    {
        // 一次反轉全部腳本
        foreach (var script in targetScripts)
        {
            if (script != null)
            {
                script.enabled = !script.enabled;
                // Debug.Log($"Toggled {script.name} → {script.enabled}");
            }
        }
    }

#if UNITY_EDITOR
    private void OnDrawGizmosSelected()
    {
        if (rightMiddleTip != null && rightThumbTip != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(rightMiddleTip.position, rightThumbTip.position);
        }
    }
#endif
}
