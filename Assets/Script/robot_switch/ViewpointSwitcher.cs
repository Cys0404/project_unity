using UnityEngine;
/// <summary>
/// 按 Space 依序在 GodView / stretch3 / koch 攝影機之間切換視角。<br/>
/// - 拖曳三台 Camera 到 Inspector<br/>
/// - 腳本掛在任何 GameObject 上即可<br/>
/// </summary>
public class ViewpointSwitcher : MonoBehaviour
{
    [Header("Robot Cameras (順序請一致)")]
    public Camera GodViewCamera;
    public Camera stretch3Camera;
    public Camera kochCamera;
    /// <summary>0 = viper, 1 = stretch3, 2 = koch</summary>
    private int currentIndex = 0;
    private Camera[] cams;
    void Awake()
    {
        cams = new Camera[] { GodViewCamera, stretch3Camera, kochCamera };
        // 若有漏填，給出警告
        foreach (Camera cam in cams)
        {
            if (cam == null)
                Debug.LogWarning("[ViewpointSwitcher] 有 Camera 沒拖到 Inspector，請檢查！");
        }
        Activate(currentIndex);
    }
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            currentIndex = (currentIndex + 1) % cams.Length;
            Activate(currentIndex);
        }
    }
    private void Activate(int idx)
    {
        for (int i = 0; i < cams.Length; i++)
        {
            Camera cam = cams[i];
            if (cam == null) continue;
            bool isActive = (i == idx);
            // 啟用 / 停用攝影機本身
            cam.enabled = isActive;
            // 同步 AudioListener（避免多支 listener）
            var listener = cam.GetComponent<AudioListener>();
            if (listener) listener.enabled = isActive;
            // 只有當前相機掛 MainCamera 標籤，其他改回 Untagged
            cam.tag = isActive ? "MainCamera" : "Untagged";
        }
        Debug.Log($"[ViewpointSwitcher] Now viewing: {(idx switch { 0 => "GodView", 1 => "stretch3", _ => "koch" })}");
    }
}





