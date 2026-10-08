using UnityEngine;

/// <summary>
/// Creates two independent, side-by-side reconstructions from one MASt3R run:
/// complete-map replacement on the left and keyframe deltas on the right.
/// This mode owns only the two children it creates while this component is
/// enabled in Play Mode. It never enables or disables standalone viewers.
/// </summary>
[DefaultExecutionOrder(-100)]
[DisallowMultipleComponent]
public sealed class Mast3rParallelTransportComparison : MonoBehaviour
{
    [Header("ROS comparison topics")]
    public string fullMapTopic = "/koch/mast3r/compare/full/map_chunks";
    public string fullTrajectoryTopic = "/koch/mast3r/compare/full/camera_trajectory";
    public string deltaMetadataTopic = "/koch/mast3r/compare/delta/keyframe_metadata";
    public string deltaCloudTopic = "/koch/mast3r/compare/delta/keyframe_cloud_local";
    public string deltaTrajectoryTopic = "/koch/mast3r/compare/delta/camera_trajectory";

    [Header("Side-by-side layout")]
    [Min(0f)] public float sceneSpacing = 8f;
    public Vector3 fullMapLocalOffset = new Vector3(-4f, 0f, 0f);
    public Vector3 deltaMapLocalOffset = new Vector3(4f, 0f, 0f);
    public bool deriveOffsetsFromSpacing = true;

    [Header("Point rendering")]
    public Material fullMapPointMaterial;
    public Material deltaPointMaterial;
    [Min(0.1f)] public float pointPixelSize = 3f;
    [Min(1f)] public float drawBoundsSize = 10000f;

    [Header("Two mini-maps")]
    [Range(128, 1024)] public int miniMapResolution = 384;
    [Min(100f)] public float miniMapSizePixels = 260f;
    [Min(0f)] public float miniMapTop = 230f;
    [Min(0f)] public float miniMapEdgeMargin = 10f;
    [Min(1000)] public int maximumMiniMapSamples = 120000;

    [Header("Diagnostics")]
    public bool verboseLogs = true;

    private GameObject fullObject;
    private GameObject deltaObject;

    private void OnEnable()
    {
        if (!Application.isPlaying) return;
        BuildComparison();
    }

    private void OnDisable()
    {
        DestroyComparison();
    }

    [ContextMenu("Rebuild Parallel Comparison")]
    public void BuildComparison()
    {
        DestroyComparison();

        Vector3 fullOffset = deriveOffsetsFromSpacing
            ? new Vector3(-sceneSpacing * 0.5f, 0f, 0f)
            : fullMapLocalOffset;
        Vector3 deltaOffset = deriveOffsetsFromSpacing
            ? new Vector3(sceneSpacing * 0.5f, 0f, 0f)
            : deltaMapLocalOffset;

        fullObject = CreateFullMapObject(fullOffset);
        deltaObject = CreateDeltaObject(deltaOffset);

        if (verboseLogs)
        {
            Debug.Log(
                "[Mast3rCompare] Created FULL (left) and DELTA (right). " +
                "Both streams use the same ROSConnection and separate topics/buffers/mini-maps.");
        }
    }

    private GameObject CreateFullMapObject(Vector3 localOffset)
    {
        GameObject child = CreateChild("FULL_complete_map_left", localOffset);
        RuntimePointCloudCollider miniMap = AddComparisonMiniMap(child, "FULL MAP", false);
        MultiRobotRosTcpMapViewer viewer = child.AddComponent<MultiRobotRosTcpMapViewer>();
        viewer.fullMapTopic = fullMapTopic;
        viewer.cameraTrajectoryTopic = fullTrajectoryTopic;
        viewer.useWorldAdjust = true;
        viewer.worldAdjust = child.transform;
        viewer.lockWorldAdjustDuringPlay = true;
        viewer.pointMaterial = fullMapPointMaterial;
        viewer.pointPixelSize = pointPixelSize;
        viewer.drawBoundsSize = drawBoundsSize;
        viewer.buildRuntimeCollision = true;
        viewer.runtimeCollider = miniMap;
        viewer.navigationRobot = null;
        viewer.comparisonLabel = "FULL / LEFT";
        viewer.statsLeft = miniMapEdgeMargin;
        viewer.statsTop = 10f;
        viewer.statsWidth = 500f;
        viewer.showStats = true;
        viewer.verboseLogs = verboseLogs;
        child.SetActive(true);
        return child;
    }

    private GameObject CreateDeltaObject(Vector3 localOffset)
    {
        GameObject child = CreateChild("DELTA_keyframes_right", localOffset);
        RuntimePointCloudCollider miniMap = AddComparisonMiniMap(child, "DELTA", true);
        Mast3rKeyframeDeltaViewer viewer = child.AddComponent<Mast3rKeyframeDeltaViewer>();
        viewer.syncTrajectoryToCloudUpdates = true;
        viewer.metadataTopic = deltaMetadataTopic;
        viewer.localCloudTopic = deltaCloudTopic;
        viewer.cameraTrajectoryTopic = deltaTrajectoryTopic;
        viewer.useWorldAdjust = true;
        viewer.worldAdjust = child.transform;
        viewer.lockWorldAdjustDuringPlay = true;
        viewer.pointMaterial = deltaPointMaterial;
        viewer.pointPixelSize = pointPixelSize;
        viewer.drawBoundsSize = drawBoundsSize;
        viewer.buildRuntimeMiniMap = true;
        viewer.runtimeCollider = miniMap;
        viewer.comparisonLabel = "DELTA / RIGHT";
        viewer.statsLeft = Mathf.Max(10f, Screen.width - 470f);
        viewer.statsTop = 10f;
        viewer.statsWidth = 460f;
        viewer.showStats = true;
        viewer.verboseLogs = verboseLogs;
        child.SetActive(true);
        return child;
    }

    private GameObject CreateChild(string objectName, Vector3 localOffset)
    {
        GameObject child = new GameObject(objectName);
        child.SetActive(false);
        child.transform.SetParent(transform, false);
        child.transform.localPosition = localOffset;
        child.transform.localRotation = Quaternion.identity;
        child.transform.localScale = Vector3.one;
        return child;
    }

    private RuntimePointCloudCollider AddComparisonMiniMap(
        GameObject child,
        string title,
        bool alignRight)
    {
        RuntimePointCloudCollider miniMap = child.AddComponent<RuntimePointCloudCollider>();
        miniMap.miniMapTitle = title;
        miniMap.showMiniMap = true;
        miniMap.alignMiniMapToRight = alignRight;
        miniMap.miniMapLeft = miniMapEdgeMargin;
        // Keep the maps below the expanded timing HUD.
        miniMap.miniMapTop = Mathf.Max(miniMapTop, 275f);
        miniMap.miniMapSizePixels = miniMapSizePixels;
        miniMap.miniMapResolution = miniMapResolution;
        miniMap.maximumGroundSamples = maximumMiniMapSamples;
        miniMap.showGroundBase = false;
        miniMap.enableGroundCollider = false;
        miniMap.enableClickToMove = false;
        miniMap.autoPlaceRobotOnFirstGround = false;
        miniMap.holdStarterPlayerUntilFirstGround = false;
        miniMap.unlockCursorForMiniMap = false;
        miniMap.verboseLogs = verboseLogs;
        return miniMap;
    }

    private static void DestroyRuntimeObject(GameObject value)
    {
        if (value == null) return;
        if (Application.isPlaying) Destroy(value);
        else DestroyImmediate(value);
    }

    private void DestroyComparison()
    {
        DestroyRuntimeObject(fullObject);
        DestroyRuntimeObject(deltaObject);
        fullObject = null;
        deltaObject = null;
    }

    private void OnDestroy()
    {
        DestroyComparison();
    }
}
