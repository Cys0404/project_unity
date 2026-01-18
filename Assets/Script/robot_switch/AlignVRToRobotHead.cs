using UnityEngine;

/// <summary>
/// 將 VR 相機對齊到機器人頭部位置，實現機器人第一視角
/// 通過在初始化後設置 XR Origin 位置實現
/// </summary>
public class AlignVRToRobotHead : MonoBehaviour
{
    [Header("XR Origin（雙機器人配置）")]
    [Tooltip("Stretch3 的 XR Origin")]
    public Transform stretch3XROrigin;
    [Tooltip("Koch 的 XR Origin")]
    public Transform kochXROrigin;

    [Header("機器人頭部參考點")]
    [Tooltip("Stretch3 的 VR Anchor")]
    public Transform stretch3HeadAnchor;
    [Tooltip("Koch 的 VR Anchor")]
    public Transform kochHeadAnchor;

    // 當前使用的物件
    private Transform currentXROrigin;
    private Transform currentHeadAnchor;

    void Start()
    {
        // 等待 XR Origin 初始化完成後再設置位置
        StartCoroutine(InitializeAfterXROrigin());
    }

    System.Collections.IEnumerator InitializeAfterXROrigin()
    {
        // 等待幾幀，確保 XR Origin 初始化完成
        yield return new WaitForSeconds(0.5f);

        // 初始化為 Stretch3（索引 0）
        SwitchToRobot(true);
    }

    /// <summary>
    /// 切換機器人時呼叫，直接設置 XR Origin 位置
    /// </summary>
    /// <param name="isStretch3">是否為 Stretch3（true=Stretch3, false=Koch）</param>
    public void SwitchToRobot(bool isStretch3)
    {
        // 切換到對應的物件
        currentXROrigin = isStretch3 ? stretch3XROrigin : kochXROrigin;
        currentHeadAnchor = isStretch3 ? stretch3HeadAnchor : kochHeadAnchor;

        if (currentXROrigin != null && currentHeadAnchor != null)
        {
            // 直接設置 XR Origin 的位置到 Anchor 位置
            currentXROrigin.position = currentHeadAnchor.position;
            currentXROrigin.rotation = currentHeadAnchor.rotation;

            Debug.Log($"[AlignVRToRobotHead] 切換到 {(isStretch3 ? "Stretch3" : "Koch")} - 設置 XR Origin 位置: {currentXROrigin.position}");
        }
    }
}