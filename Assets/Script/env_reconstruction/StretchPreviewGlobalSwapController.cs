using UnityEngine;

public class StretchPreviewGlobalSwapController : MonoBehaviour
{
    [Header("Viewers")]
    public MultiRobotPlyMapViewer globalMapViewer;
    public SingleRobotFramePointCloudViewer stretchPreviewViewer;
    public GameObject stretchPreviewGameObject;

    [Header("Swap Condition")]
    public int requiredGlobalRobotCount = 2;
    public bool disableStretchGameObject = true;

    [Header("Transform Reset On Global Swap")]
    public GameObject transformResetGameObject;
    public Vector3 targetLocalPosition = Vector3.zero;
    public Vector3 targetLocalEulerAngles = Vector3.zero;
    public Vector3 targetLocalScale = Vector3.one;

    [Header("Automatic Setup")]
    public bool configureViewersOnAwake = true;
    public string stretchPointcloudTopic = "/robot2/mast3r/frame_pointcloud";

    private bool switchedToGlobal;

    void Awake()
    {
        if (!configureViewersOnAwake) return;

        if (globalMapViewer != null)
        {
            // Show the anchor robot PLY immediately. The controller removes the
            // temporary stretch preview only after a two-robot PLY is displayed.
            globalMapViewer.waitForCrossRobotAlignment = false;
            globalMapViewer.enableLocalPreviewWhileWaiting = false;
        }

        if (stretchPreviewViewer != null)
        {
            stretchPreviewViewer.pointcloudTopic = stretchPointcloudTopic;
        }
    }

    void OnEnable()
    {
        if (globalMapViewer != null)
        {
            globalMapViewer.MapDisplayed += OnGlobalMapDisplayed;
        }
    }

    void Start()
    {
        if (globalMapViewer != null)
        {
            TrySwap(globalMapViewer.DisplayedVersion, globalMapViewer.DisplayedRobotCount);
        }
    }

    void OnDisable()
    {
        if (globalMapViewer != null)
        {
            globalMapViewer.MapDisplayed -= OnGlobalMapDisplayed;
        }
    }

    private void OnGlobalMapDisplayed(int version, int robotCount)
    {
        TrySwap(version, robotCount);
    }

    private void TrySwap(int version, int robotCount)
    {
        if (switchedToGlobal || robotCount < requiredGlobalRobotCount) return;

        switchedToGlobal = true;

        if (transformResetGameObject != null)
        {
            Transform targetTransform = transformResetGameObject.transform;
            targetTransform.localPosition = targetLocalPosition;
            targetTransform.localRotation = Quaternion.Euler(targetLocalEulerAngles);
            targetTransform.localScale = targetLocalScale;
        }

        if (stretchPreviewViewer != null)
        {
            stretchPreviewViewer.ClearAndStop();
        }

        GameObject target = stretchPreviewGameObject != null
            ? stretchPreviewGameObject
            : stretchPreviewViewer != null ? stretchPreviewViewer.gameObject : null;

        if (disableStretchGameObject && target != null)
        {
            if (globalMapViewer != null && target == globalMapViewer.gameObject)
            {
                Debug.LogWarning("[GlobalSwap] Stretch preview and global viewer share one GameObject; only the stretch component was stopped.");
            }
            else
            {
                target.SetActive(false);
            }
        }

        Debug.Log($"[GlobalSwap] Global map displayed version={version} robots={robotCount}; stretch preview cleared and disabled.");
    }
}
