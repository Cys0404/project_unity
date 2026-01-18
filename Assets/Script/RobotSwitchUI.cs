using System.Collections;
using UnityEngine;

public class RobotSwitchUI : MonoBehaviour
{
    public enum DefaultRobot { Stretch3, Koch }

    [Header("UI References (RectTransform)")]
    public RectTransform panelStretch3;
    public RectTransform panelKoch;
    public RectTransform selectionFrame;

    [Header("Default")]
    public DefaultRobot defaultRobot = DefaultRobot.Stretch3;

    [Header("Animation")]
    public float moveDuration = 0.22f;
    public float padding = 12f;

    private RectTransform currentTarget;
    private Coroutine moveCoroutine;

    void Start()
    {
        Canvas.ForceUpdateCanvases(); // 先讓 Canvas/Layout 立即更新一次
        // 起始位置直接強制貼到你指定的那個 panel
        var startTarget = (defaultRobot == DefaultRobot.Stretch3) ? panelStretch3 : panelKoch;
        SetTargetInstant(startTarget);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            var next = (currentTarget == panelStretch3) ? panelKoch : panelStretch3;
            SwitchTo(next);
        }
    }

    public void SwitchTo(RectTransform targetPanel)
    {
        if (targetPanel == null || selectionFrame == null) return;

        if (moveCoroutine != null) StopCoroutine(moveCoroutine);
        moveCoroutine = StartCoroutine(MoveFrameTo(targetPanel));
    }

    IEnumerator MoveFrameTo(RectTransform targetPanel)
    {
        currentTarget = targetPanel;

        Vector2 startPos = selectionFrame.anchoredPosition;
        Vector2 endPos = targetPanel.anchoredPosition;

        Vector2 endSize = targetPanel.sizeDelta + new Vector2(padding * 2f, padding * 2f);
        Vector2 startSize = selectionFrame.sizeDelta;

        float t = 0f;
        while (t < 1f)
        {
            t += Time.unscaledDeltaTime / Mathf.Max(0.0001f, moveDuration);
            float eased = 1f - Mathf.Pow(1f - t, 3f);

            selectionFrame.anchoredPosition = Vector2.Lerp(startPos, endPos, eased);
            selectionFrame.sizeDelta = Vector2.Lerp(startSize, endSize, eased);
            yield return null;
        }

        selectionFrame.anchoredPosition = endPos;
        selectionFrame.sizeDelta = endSize;
        moveCoroutine = null;
    }

    void SetTargetInstant(RectTransform targetPanel)
    {
        if (targetPanel == null || selectionFrame == null) return;

        currentTarget = targetPanel;
        selectionFrame.anchoredPosition = targetPanel.anchoredPosition;
        selectionFrame.sizeDelta = targetPanel.sizeDelta + new Vector2(padding * 2f, padding * 2f);
    }
}
