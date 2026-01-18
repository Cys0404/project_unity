using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

public class RobotSwitcherManager : MonoBehaviour
{
    [System.Serializable]
    public class RobotRig
    {
        public string robotName;              // "Stretch3" / "Koch"
        public Transform viewAnchor;          // 第一視角 anchor
        public Transform recordAnchor;        // 錄影相機 anchor
    }

    [Header("Robots")]
    public RobotRig[] robots;

    [Header("XR References")]
    public Transform xrOrigin;               // XR Origin 的 Transform
    public Camera xrMainCamera;              // XR Origin 底下的 Main Camera

    [Header("Recording Camera (GameView)")]
    public Camera recordingCamera;           // 你用來錄影的固定相機

    [Header("Default")]
    public int defaultRobotIndex = 0;

    [Header("Switch Trigger (temp)")]
    public KeyCode switchKey = KeyCode.Space;

    private int activeIndex = -1;

    void Start()
    {
        // 初始切到預設 robot（保證一開始就到位）
        SwitchTo(defaultRobotIndex, instant: true);
    }

    void Update()
    {
        // 先用空白鍵切換
        if (Input.GetKeyDown(switchKey))
        {
            int next = (activeIndex + 1) % robots.Length;
            SwitchTo(next, instant: false);
        }
    }

    public void SwitchTo(int index, bool instant)
    {
        if (robots == null || robots.Length == 0) return;
        if (index < 0 || index >= robots.Length) return;

        var rig = robots[index];
        if (rig.viewAnchor == null) return;

        // 1) VR 第一視角瞬移：讓「Main Camera」對齊 viewAnchor
        AlignXROriginToViewAnchor(rig.viewAnchor);

        // 2) 錄影相機同步切換
        if (recordingCamera != null && rig.recordAnchor != null)
        {
            recordingCamera.transform.SetPositionAndRotation(
                rig.recordAnchor.position,
                rig.recordAnchor.rotation
            );
        }

        // 3) 設定 active robot（之後你把控制路由接在這裡）
        activeIndex = index;

        // TODO:
        // - ControlRouter.SetActive(rig.robotName);
        // - UISelectionFrame.MoveTo(rig.robotName);
        // - 顯示 "A -> B" toast 等等
    }

    void AlignXROriginToViewAnchor(Transform viewAnchor)
    {
        if (xrOrigin == null || xrMainCamera == null) return;

        // (A) 位置：把 XR Origin 平移，讓 Main Camera 落到 viewAnchor
        Vector3 delta = viewAnchor.position - xrMainCamera.transform.position;
        xrOrigin.position += delta;

        // (B) 旋轉：只對齊水平 yaw（避免改到頭的 pitch/roll）
        Vector3 camForward = xrMainCamera.transform.forward;
        camForward.y = 0f;
        camForward.Normalize();

        Vector3 anchorForward = viewAnchor.forward;
        anchorForward.y = 0f;
        anchorForward.Normalize();

        if (camForward.sqrMagnitude > 0.0001f && anchorForward.sqrMagnitude > 0.0001f)
        {
            float yawDelta = Vector3.SignedAngle(camForward, anchorForward, Vector3.up);
            xrOrigin.Rotate(0f, yawDelta, 0f, Space.World);
        }
    }
}
