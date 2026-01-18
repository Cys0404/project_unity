using UnityEngine;
using TMPro;

public class ToggleStatusText : MonoBehaviour
{
    [Header("Source")]
    public FingerPinchToggle pinchToggle;

    [Header("Text")]
    public string onText = "ON";
    public string offText = "OFF";

    [Header("Colors")]
    public Color onColor = Color.green;
    public Color offColor = Color.red;

    public TMP_Text statusText;

    void Awake()
    {
        statusText = GetComponent<TMP_Text>();
        if (pinchToggle == null)
            pinchToggle = FindFirstObjectByType<FingerPinchToggle>();
    }

    void Start()
    {
        // 一開始就顯示正確狀態，避免顯示 0/舊字
        RefreshStatus();
    }

    void Update()
    {
        RefreshStatus(); // 每幀刷新最穩（你之後要最佳化再改成事件也行）
    }

    void RefreshStatus()
    {
        if (statusText == null || pinchToggle == null)
            return;

        bool allEnabled = AreAllTargetScriptsEnabled();

        statusText.text = allEnabled ? onText : offText;
        statusText.color = allEnabled ? onColor : offColor;
    }

    bool AreAllTargetScriptsEnabled()
    {
        var list = pinchToggle.targetScripts;
        if (list == null || list.Count == 0) return false;

        foreach (var s in list)
        {
            if (s == null) return false;
            if (!s.enabled) return false;
        }
        return true;
    }
}
