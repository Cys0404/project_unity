using UnityEngine;
/// <summary>
/// 按 Space 在 stretch3 / koch 攝影機之間切換視角。<br/>
/// - 拖曳相機到 Inspector<br/>
/// - 腳本掛在任何 GameObject 上即可<br/>
/// </summary>
public class ViewpointSwitcher : MonoBehaviour
{
    [Header("Robot Cameras (順序請一致)")]
    public Camera stretch3Camera;
    public Camera kochCamera;

    [Header("靜態錄影相機")]
    [Tooltip("不追蹤 Head Pose 的靜態相機，用於錄影")]
    public Camera stretch3StaticCamera;
    public Camera kochStaticCamera;

    /// <summary>0 = stretch3, 1 = koch</summary>
    private int currentIndex = 0;
    private Camera[] cams;
    private Camera[] staticCams;
    void Awake()
    {
        cams = new Camera[] { stretch3Camera, kochCamera };
        staticCams = new Camera[] { stretch3StaticCamera, kochStaticCamera };

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
        // 切換主相機（Head Pose 追蹤）
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

        // 同步切換靜態錄影相機
        for (int i = 0; i < staticCams.Length; i++)
        {
            Camera staticCam = staticCams[i];
            if (staticCam == null) continue;
            bool isActive = (i == idx);
            staticCam.enabled = isActive;
        }

        Debug.Log($"[ViewpointSwitcher] Now viewing: {(idx == 0 ? "stretch3" : "koch")}");
    }
}





