using System;
using System.Collections.Concurrent;
using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using UnityEditor;
#endif
using Vector3 = UnityEngine.Vector3;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Transform = UnityEngine.Transform;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class map_tcp : MonoBehaviour
{
    [Header("ROS TCP Topics")]
    public string fullMapTopic = "/koch/mast3r/unity_map_points_full";
    public string robotAlignmentsTopic = "/multi_robot/robot_alignments";

    [Header("Cross-Robot Presentation Gate")]
    public bool waitForCrossRobotAlignment = true;
    public int minAlignedRobots = 2;
    public int minUsedConstraints = 1;
    public bool clearLocalPreviewOnGlobalReady = true;

    [Header("Local Preview Before Global Merge")]
    public bool enableLocalPreviewWhileWaiting = true;
    public string[] localPreviewPointcloudTopics = new string[]
    {
        "/robot1/mast3r/frame_pointcloud",
        "/robot2/mast3r/frame_pointcloud"
    };
    public int maxLocalPreviewPoints = 2_000_000;
    public int localPreviewPointStride = 1;

    [Header("Memory")]
    public int maxGlobalPoints = 10_000_000;

    [Header("World Adjust")]
    public bool useWorldAdjust = true;
    public Transform worldAdjust;
    public bool useLocalPreviewWorldAdjust = false;
    public Transform localPreviewWorldAdjust;

    [Header("Rendering")]
    public Material pointMaterial;
    public float pointPixelSize = 3f;
    public float drawBoundsSize = 10000f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;
    public List<Camera> extraRenderCameras = new List<Camera>();

    [Header("Editor View")]
    public bool showInSceneView = true;

    [Header("Debug")]
    public bool showStats = true;
    public bool verboseLogs = true;
    public bool logRenderCameras = false;

    private struct CloudBatch
    {
        public int version;
        public Vector3[] positions;
        public uint[] colors;
        public int count;
        public int sourcePointCount;
        public string frameId;
    }

    private struct PreviewBatch
    {
        public Vector3[] positions;
        public uint[] colors;
        public int count;
        public string topic;
        public string frameId;
    }

    private ROSConnection ros;
    private readonly ConcurrentQueue<PointCloud2Msg> fullMapQueue = new ConcurrentQueue<PointCloud2Msg>();
    private readonly ConcurrentQueue<PreviewBatch> localPreviewQueue = new ConcurrentQueue<PreviewBatch>();

    private ComputeBuffer[] posBuffers = new ComputeBuffer[2];
    private ComputeBuffer[] colBuffers = new ComputeBuffer[2];
    private int[] bufferCounts = new int[2];
    private int activeBufferIdx = 0;
    private int backBufferIdx = 1;

    private ComputeBuffer localPreviewPosBuffer;
    private ComputeBuffer localPreviewColBuffer;
    private int localPreviewPointCount = 0;

    private ComputeBuffer dummyIdxBuffer;
    private ComputeBuffer dummyPoseBuffer;

    private bool isLoading = false;
    private bool hasPendingLatestCloud = false;
    private PointCloud2Msg pendingLatestCloud;
    private int latestRequestedVersion = -1;
    private int displayedVersion = -1;
    private string displayedFrameId = "";
    private float lastParseSec = 0f;
    private int receivedFullMaps = 0;
    private int displayedFullMaps = 0;
    private int droppedUpdates = 0;
    private int failedUpdates = 0;
    private bool crossRobotAlignmentReady = false;
    private int latestAlignedRobotCount = 0;
    private int latestUsedConstraintCount = 0;
    private int localPreviewMessages = 0;
    private int localPreviewDroppedMessages = 0;
    private int lastExtraRenderFrame = -1;
    private Camera lastExtraRenderCamera = null;
    private float lastRenderCameraLogTime = -999f;

    void Awake()
    {
        Debug.Log("[MapTCP] Awake");

        if (pointMaterial == null)
        {
            var sh = Shader.Find("Hidden/PointCloudGPU");
            if (sh == null) sh = Shader.Find("Hidden/Point Cloud GPU");
            if (sh != null) pointMaterial = new Material(sh);
        }

        for (int i = 0; i < 2; i++)
        {
            posBuffers[i] = new ComputeBuffer(maxGlobalPoints, 12);
            colBuffers[i] = new ComputeBuffer(maxGlobalPoints, 4);
            bufferCounts[i] = 0;
        }

        localPreviewPosBuffer = new ComputeBuffer(maxLocalPreviewPoints, 12);
        localPreviewColBuffer = new ComputeBuffer(maxLocalPreviewPoints, 4);

        int dummyCapacity = Mathf.Max(maxGlobalPoints, maxLocalPreviewPoints);
        dummyIdxBuffer = new ComputeBuffer(dummyCapacity, 4);
        uint[] zeroIndices = new uint[dummyCapacity];
        dummyIdxBuffer.SetData(zeroIndices);

        dummyPoseBuffer = new ComputeBuffer(1, 64);
        dummyPoseBuffer.SetData(new Matrix4x4[] { Matrix4x4.identity });
    }

    void OnEnable()
    {
        Camera.onPostRender += OnCameraPostRender;
#if UNITY_EDITOR
        SceneView.duringSceneGui += OnSceneViewGui;
#endif
    }

    void OnDisable()
    {
        Camera.onPostRender -= OnCameraPostRender;
#if UNITY_EDITOR
        SceneView.duringSceneGui -= OnSceneViewGui;
#endif
    }

#if UNITY_EDITOR
    private void OnSceneViewGui(SceneView sceneView)
    {
        if (!showInSceneView || !Application.isPlaying) return;
        if (Event.current == null || Event.current.type != EventType.Repaint) return;
        if (!HasDrawablePoints()) return;

        DrawActiveBuffer();
        sceneView.Repaint();
    }
#endif

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        Debug.Log("[MapTCP] Initialize ROS TCP subscriptions");

        if (waitForCrossRobotAlignment)
        {
            Debug.Log($"[MapTCP] Waiting for cross-robot alignment: robots>={minAlignedRobots}, used_constraints>={minUsedConstraints}. Local preview will be shown while waiting.");
        }

        if (!string.IsNullOrWhiteSpace(fullMapTopic))
        {
            ros.Subscribe<PointCloud2Msg>(fullMapTopic, ReceiveFullMapPointCloud);
            Debug.Log($"[MapTCP] Subscribed full map topic {fullMapTopic}");
        }

        if (!string.IsNullOrWhiteSpace(robotAlignmentsTopic))
        {
            ros.Subscribe<StringMsg>(robotAlignmentsTopic, ReceiveRobotAlignments);
            Debug.Log($"[MapTCP] Subscribed alignment topic {robotAlignmentsTopic}");
        }

        if (enableLocalPreviewWhileWaiting && localPreviewPointcloudTopics != null)
        {
            foreach (string topic in localPreviewPointcloudTopics)
            {
                if (string.IsNullOrWhiteSpace(topic)) continue;
                string topicCopy = topic;
                ros.Subscribe<PointCloud2Msg>(topicCopy, msg => ReceiveLocalPreviewPointCloud(topicCopy, msg));
                Debug.Log($"[MapTCP] Subscribed local preview pointcloud {topicCopy}");
            }
        }
    }

    private void ReceiveFullMapPointCloud(PointCloud2Msg msg)
    {
        receivedFullMaps++;
        fullMapQueue.Enqueue(msg);
        if (verboseLogs)
        {
            Debug.Log($"[MapTCP] RX full map frame={msg.header.frame_id} points={(long)msg.width * msg.height:N0} queue={fullMapQueue.Count}");
        }
    }

    private void ReceiveRobotAlignments(StringMsg msg)
    {
        int usedConstraints = ExtractInt(msg.data, "used_constraint_count", 0);
        int alignedRobots = CountRobotAlignments(msg.data);
        latestUsedConstraintCount = usedConstraints;
        latestAlignedRobotCount = alignedRobots;

        if (!crossRobotAlignmentReady && usedConstraints >= minUsedConstraints && alignedRobots >= minAlignedRobots)
        {
            crossRobotAlignmentReady = true;
            Debug.Log($"[MapTCP] Cross-robot alignment ready: robots={alignedRobots}, used_constraints={usedConstraints}. Swapping from local preview to global TCP full map.");
            if (clearLocalPreviewOnGlobalReady)
            {
                localPreviewPointCount = 0;
            }
        }
    }

    private void ReceiveLocalPreviewPointCloud(string topic, PointCloud2Msg msg)
    {
        if (!enableLocalPreviewWhileWaiting) return;
        if (ShouldShowGlobalMap()) return;

        try
        {
            PreviewBatch batch = DecodePointCloudToPreviewBatch(msg, topic, maxLocalPreviewPoints, Mathf.Max(1, localPreviewPointStride));
            if (batch.count > 0)
            {
                localPreviewQueue.Enqueue(batch);
            }
        }
        catch (Exception exc)
        {
            localPreviewDroppedMessages++;
            Debug.LogWarning($"[MapTCP] Drop local preview topic={topic}: {exc.Message}");
        }
    }

    void Update()
    {
        ProcessLocalPreviewQueue();

        PointCloud2Msg newestCloud = null;
        while (fullMapQueue.TryDequeue(out PointCloud2Msg msg))
        {
            newestCloud = msg;
            if (fullMapQueue.Count > 0) droppedUpdates++;
        }

        if (newestCloud != null)
        {
            if (isLoading)
            {
                pendingLatestCloud = newestCloud;
                hasPendingLatestCloud = true;
            }
            else
            {
                StartCoroutine(ParseAndSwapFullMap(newestCloud));
            }
        }

        if (!isLoading && hasPendingLatestCloud)
        {
            PointCloud2Msg next = pendingLatestCloud;
            pendingLatestCloud = null;
            hasPendingLatestCloud = false;
            if (next != null)
            {
                StartCoroutine(ParseAndSwapFullMap(next));
            }
        }
    }

    private void ProcessLocalPreviewQueue()
    {
        if (!enableLocalPreviewWhileWaiting) return;
        if (ShouldShowGlobalMap())
        {
            if (clearLocalPreviewOnGlobalReady) localPreviewPointCount = 0;
            while (localPreviewQueue.TryDequeue(out PreviewBatch discardedBatch)) { }
            return;
        }

        while (localPreviewQueue.TryDequeue(out PreviewBatch batch))
        {
            int free = maxLocalPreviewPoints - localPreviewPointCount;
            if (free <= 0)
            {
                localPreviewDroppedMessages++;
                continue;
            }

            int insertCount = Mathf.Min(batch.count, free);
            localPreviewPosBuffer.SetData(batch.positions, 0, localPreviewPointCount, insertCount);
            localPreviewColBuffer.SetData(batch.colors, 0, localPreviewPointCount, insertCount);
            localPreviewPointCount += insertCount;
            localPreviewMessages++;
        }
    }

    private IEnumerator ParseAndSwapFullMap(PointCloud2Msg msg)
    {
        isLoading = true;
        int version = ++latestRequestedVersion;
        string frameId = msg.header.frame_id;
        int sourcePointCount = SafePointCount(msg);
        float parseStart = Time.realtimeSinceStartup;

        if (verboseLogs)
        {
            Debug.Log($"[MapTCP] Parsing full map version={version} frame={frameId} points={sourcePointCount:N0}");
        }

        Task<CloudBatch> parseTask = Task.Run(() => DecodePointCloudToCloudBatch(msg, version, maxGlobalPoints));
        while (!parseTask.IsCompleted) yield return null;

        lastParseSec = Time.realtimeSinceStartup - parseStart;
        if (parseTask.IsFaulted)
        {
            failedUpdates++;
            isLoading = false;
            Exception ex = parseTask.Exception != null ? parseTask.Exception.GetBaseException() : null;
            Debug.LogError($"[MapTCP] Parse failed version={version}: {ex?.Message}");
            yield break;
        }

        CloudBatch parsed = parseTask.Result;
        if (verboseLogs) Debug.Log($"[MapTCP] Parsed version={parsed.version} points={parsed.count:N0}; uploading GPU buffer");
        int writeIdx = backBufferIdx;
        posBuffers[writeIdx].SetData(parsed.positions, 0, 0, parsed.count);
        colBuffers[writeIdx].SetData(parsed.colors, 0, 0, parsed.count);
        bufferCounts[writeIdx] = parsed.count;

        int oldActive = activeBufferIdx;
        activeBufferIdx = writeIdx;
        backBufferIdx = oldActive;
        bufferCounts[backBufferIdx] = 0;

        displayedVersion = parsed.version;
        displayedFrameId = parsed.frameId;
        displayedFullMaps++;
        isLoading = false;

        if (verboseLogs)
        {
            Debug.Log($"[MapTCP] Displayed version={parsed.version} points={parsed.count:N0}/{parsed.sourcePointCount:N0} parse={lastParseSec:F3}s frame={parsed.frameId}");
        }
    }

    private static CloudBatch DecodePointCloudToCloudBatch(PointCloud2Msg msg, int version, int maxPoints)
    {
        int sourceCount = SafePointCount(msg);
        int outCount = sourceCount;
        int stride = 1;
        if (maxPoints > 0 && outCount > maxPoints)
        {
            stride = Math.Max(1, sourceCount / maxPoints);
            outCount = Math.Min(maxPoints, (sourceCount + stride - 1) / stride);
        }

        Vector3[] positions = new Vector3[outCount];
        uint[] colors = new uint[outCount];
        int written = DecodePointCloud(msg, stride, outCount, positions, colors);
        if (written != outCount)
        {
            Array.Resize(ref positions, written);
            Array.Resize(ref colors, written);
            outCount = written;
        }

        return new CloudBatch
        {
            version = version,
            positions = positions,
            colors = colors,
            count = outCount,
            sourcePointCount = sourceCount,
            frameId = msg.header.frame_id,
        };
    }

    private static PreviewBatch DecodePointCloudToPreviewBatch(PointCloud2Msg msg, string topic, int maxPoints, int stride)
    {
        int sourceCount = SafePointCount(msg);
        int decodeCount = Math.Min((sourceCount + stride - 1) / stride, maxPoints);
        Vector3[] positions = new Vector3[decodeCount];
        uint[] colors = new uint[decodeCount];
        int written = DecodePointCloud(msg, stride, decodeCount, positions, colors);
        if (written != decodeCount)
        {
            Array.Resize(ref positions, written);
            Array.Resize(ref colors, written);
        }

        return new PreviewBatch
        {
            positions = positions,
            colors = colors,
            count = written,
            topic = topic,
            frameId = msg.header.frame_id,
        };
    }

    private static int DecodePointCloud(PointCloud2Msg msg, int stride, int maxOut, Vector3[] positions, uint[] colors)
    {
        int sourceCount = SafePointCount(msg);
        if (sourceCount <= 0) return 0;

        int xOff = -1, yOff = -1, zOff = -1, rgbOff = -1;
        foreach (PointFieldMsg field in msg.fields)
        {
            if (field.name == "x") xOff = (int)field.offset;
            else if (field.name == "y") yOff = (int)field.offset;
            else if (field.name == "z") zOff = (int)field.offset;
            else if (field.name == "rgb" || field.name == "rgba") rgbOff = (int)field.offset;
        }
        if (xOff < 0 || yOff < 0 || zOff < 0)
        {
            throw new InvalidOperationException($"PointCloud2 missing xyz fields frame={msg.header.frame_id}");
        }

        int step = (int)msg.point_step;
        byte[] data = msg.data;
        int written = 0;
        for (int i = 0; i < sourceCount && written < maxOut; i += stride)
        {
            int b = i * step;
            if (b + step > data.Length) break;
            float x = BitConverter.ToSingle(data, b + xOff);
            float y = BitConverter.ToSingle(data, b + yOff);
            float z = BitConverter.ToSingle(data, b + zOff);
            if (float.IsNaN(x) || float.IsInfinity(x) || float.IsNaN(y) || float.IsInfinity(y) || float.IsNaN(z) || float.IsInfinity(z)) continue;

            positions[written] = new Vector3(x, y, z);
            colors[written] = (rgbOff >= 0 && b + rgbOff + 4 <= data.Length)
                ? BitConverter.ToUInt32(data, b + rgbOff)
                : 0xFFFFFFFF;
            written++;
        }
        return written;
    }

    private static int SafePointCount(PointCloud2Msg msg)
    {
        long count = (long)msg.width * msg.height;
        if (count > int.MaxValue) return int.MaxValue;
        return (int)Math.Max(0, count);
    }

    private static int CountRobotAlignments(string json)
    {
        Match section = Regex.Match(json, "\\\"robot_alignments\\\"\\s*:\\s*\\{(?<body>.*?)\\}\\s*(,|$)");
        if (!section.Success) return 0;
        MatchCollection keys = Regex.Matches(section.Groups["body"].Value, "\\\"([^\\\"]+)\\\"\\s*:");
        return keys.Count;
    }

    private static int ExtractInt(string json, string key, int fallback)
    {
        long value = ExtractLong(json, key, fallback);
        if (value > int.MaxValue) return int.MaxValue;
        if (value < int.MinValue) return int.MinValue;
        return (int)value;
    }

    private static long ExtractLong(string json, string key, long fallback)
    {
        Match m = Regex.Match(json, "\\\"" + Regex.Escape(key) + "\\\"\\s*:\\s*(-?\\d+)");
        return m.Success && long.TryParse(m.Groups[1].Value, out long value) ? value : fallback;
    }

    void OnRenderObject()
    {
        Camera currentCamera = Camera.current;
        if (IsExtraRenderCamera(currentCamera))
        {
            MaybeLogRenderCamera("OnRenderObject skipped extra", currentCamera);
            return;
        }

        MaybeLogRenderCamera("OnRenderObject", currentCamera);
        DrawActiveBuffer();
    }

    private void OnCameraPostRender(Camera cam)
    {
        if (cam == null) return;
        if (!IsExtraRenderCamera(cam)) return;

        if (lastExtraRenderFrame == Time.frameCount && lastExtraRenderCamera == cam)
        {
            return;
        }

        lastExtraRenderFrame = Time.frameCount;
        lastExtraRenderCamera = cam;
        MaybeLogRenderCamera("OnCameraPostRender", cam);
        DrawActiveBuffer();
    }

    private bool IsExtraRenderCamera(Camera cam)
    {
        if (cam == null) return false;
        if (extraRenderCameras == null || extraRenderCameras.Count == 0) return false;

        for (int i = 0; i < extraRenderCameras.Count; i++)
        {
            if (extraRenderCameras[i] == cam) return true;
        }
        return false;
    }

    private void MaybeLogRenderCamera(string source, Camera cam)
    {
        if (!logRenderCameras) return;
        float now = Time.unscaledTime;
        if (now - lastRenderCameraLogTime < 1f) return;

        string cameraName = cam != null ? cam.name : "<null>";
        Debug.Log($"[MapTCP] {source} camera={cameraName} frame={Time.frameCount}");
        lastRenderCameraLogTime = now;
    }

    private void DrawActiveBuffer()
    {
        if (pointMaterial == null) return;

        if (ShouldShowGlobalMap())
        {
            DrawPointBuffer(posBuffers[activeBufferIdx], colBuffers[activeBufferIdx], bufferCounts[activeBufferIdx], GetGlobalWorldAdjust());
            return;
        }

        if (enableLocalPreviewWhileWaiting)
        {
            DrawPointBuffer(localPreviewPosBuffer, localPreviewColBuffer, localPreviewPointCount, GetLocalPreviewWorldAdjust());
        }
    }

    private bool ShouldShowGlobalMap()
    {
        return !waitForCrossRobotAlignment || crossRobotAlignmentReady;
    }

    private bool HasDrawablePoints()
    {
        if (ShouldShowGlobalMap()) return bufferCounts[activeBufferIdx] > 0;
        return enableLocalPreviewWhileWaiting && localPreviewPointCount > 0;
    }

    private Matrix4x4 GetGlobalWorldAdjust()
    {
        return (useWorldAdjust && worldAdjust != null)
            ? worldAdjust.localToWorldMatrix
            : Matrix4x4.identity;
    }

    private Matrix4x4 GetLocalPreviewWorldAdjust()
    {
        return (useLocalPreviewWorldAdjust && localPreviewWorldAdjust != null)
            ? localPreviewWorldAdjust.localToWorldMatrix
            : GetGlobalWorldAdjust();
    }

    private void DrawPointBuffer(ComputeBuffer positions, ComputeBuffer colors, int pointCount, Matrix4x4 adjust)
    {
        if (pointMaterial == null || positions == null || colors == null) return;
        if (pointCount <= 0) return;

        pointMaterial.SetMatrix("_WorldAdjust", adjust);
        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", colorTint);
        pointMaterial.SetBuffer("_Positions", positions);
        pointMaterial.SetBuffer("_Colors", colors);
        pointMaterial.SetBuffer("_KeyframeIndices", dummyIdxBuffer);
        pointMaterial.SetBuffer("_KeyframePoses", dummyPoseBuffer);

        Graphics.DrawProcedural(
            pointMaterial,
            GetDrawBounds(adjust),
            MeshTopology.Points,
            pointCount,
            1);
    }

    private Bounds GetDrawBounds(Matrix4x4 adjust)
    {
        float size = Mathf.Max(1f, drawBoundsSize);
        Vector3 center = adjust.MultiplyPoint3x4(Vector3.zero);
        float scale = Mathf.Max(
            adjust.GetColumn(0).magnitude,
            Mathf.Max(adjust.GetColumn(1).magnitude, adjust.GetColumn(2).magnitude));
        return new Bounds(center, Vector3.one * size * Mathf.Max(1f, scale));
    }

    void OnGUI()
    {
        if (!showStats) return;
        GUI.color = Color.cyan;
        GUILayout.Label($"[Map TCP Viewer] FPS: {1.0f / Time.smoothDeltaTime:0}");
        GUILayout.Label($"Mode: {(ShouldShowGlobalMap() ? "Global TCP" : "Local Preview")}, CrossReady={crossRobotAlignmentReady} robots={latestAlignedRobotCount} used={latestUsedConstraintCount}");
        GUILayout.Label($"Displayed Version: {displayedVersion}");
        GUILayout.Label($"Global Points: {bufferCounts[activeBufferIdx]:N0} / {maxGlobalPoints:N0}");
        GUILayout.Label($"Local Preview Points: {localPreviewPointCount:N0} / {maxLocalPreviewPoints:N0} messages={localPreviewMessages} dropped={localPreviewDroppedMessages}");
        GUILayout.Label($"Parse: {lastParseSec:F3}s  Loading: {isLoading}");
        GUILayout.Label($"Full Maps: {displayedFullMaps} shown, {receivedFullMaps} rx, {droppedUpdates} skipped, {failedUpdates} failed");
        if (!string.IsNullOrEmpty(displayedFrameId)) GUILayout.Label(displayedFrameId);
    }

    void OnDestroy()
    {
        if (ros != null)
        {
            if (!string.IsNullOrWhiteSpace(fullMapTopic)) ros.Unsubscribe(fullMapTopic);
            if (!string.IsNullOrWhiteSpace(robotAlignmentsTopic)) ros.Unsubscribe(robotAlignmentsTopic);
            if (localPreviewPointcloudTopics != null)
            {
                foreach (string topic in localPreviewPointcloudTopics)
                {
                    if (!string.IsNullOrWhiteSpace(topic)) ros.Unsubscribe(topic);
                }
            }
        }

        for (int i = 0; i < 2; i++)
        {
            posBuffers[i]?.Release();
            colBuffers[i]?.Release();
        }
        localPreviewPosBuffer?.Release();
        localPreviewColBuffer?.Release();
        dummyIdxBuffer?.Release();
        dummyPoseBuffer?.Release();
    }
}
