using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using UnityEditor;
#endif
using Debug = UnityEngine.Debug;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Vector3 = UnityEngine.Vector3;

using RosMessageTypes.Sensor;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;

[DisallowMultipleComponent]
public class MultiRobotRosTcpMapViewer : MonoBehaviour
{
    [Header("ROS TCP")]
    public string fullMapTopic = "/koch/mast3r/unity_map_points_full";
    public string cameraTrajectoryTopic = "/koch/mast3r/camera_trajectory";

    [Header("Memory")]
    [Tooltip("Zero keeps every finite point. A positive value rejects oversized maps instead of sampling them.")]
    public int hardPointLimit = 0;
    [Tooltip("Discard an incomplete chunked-map revision after this many seconds. Zero disables the timeout.")]
    public float chunkAssemblyTimeoutSeconds = 30f;

    [Header("World Adjust")]
    public bool useWorldAdjust = true;
    public Transform worldAdjust;
    [Tooltip("Freeze the map transform when data first arrives so moving the player cannot move the point cloud.")]
    public bool lockWorldAdjustDuringPlay = true;

    [Header("Rendering")]
    public Material pointMaterial;
    public float pointPixelSize = 3f;
    public float drawBoundsSize = 10000f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;
    [Tooltip("Compatibility only. All enabled cameras are now rendered automatically in onPreCull.")]
    public List<Camera> extraRenderCameras = new List<Camera>();

    [Header("Editor View")]
    public bool showInSceneView = true;

    [Header("Runtime Collision")]
    [Tooltip("Forward decoded maps to the separately attached RuntimePointCloudCollider. This viewer never adds or enables that component.")]
    public bool buildRuntimeCollision = true;
    public RuntimePointCloudCollider runtimeCollider;
    [Tooltip("Optional PlayerArmature/root used by mini-map click-to-move. Leave empty for automatic discovery.")]
    public Transform navigationRobot;

    [Header("Debug")]
    public bool showStats = true;
    public bool verboseLogs = true;
    public string comparisonLabel = "FULL MAP";
    public float statsLeft = 10f;
    public float statsTop = 10f;
    public float statsWidth = 470f;

    private sealed class DecodedCloud
    {
        public int version;
        public string frameId;
        public int sourcePointCount;
        public Vector3[] positions;
        public uint[] colors;
        public int count;
    }

    private sealed class ChunkAssembly
    {
        public int sourceRevision;
        public int displayVersion;
        public int totalChunks;
        public DecodedCloud[] chunks;
        public bool[] received;
        public int receivedCount;
        public long sourcePointCount;
        public long validPointCount;
        public float firstSeenTime;
        public float lastSeenTime;
    }

    private ROSConnection ros;
    private readonly ConcurrentQueue<PointCloud2Msg> receivedClouds = new ConcurrentQueue<PointCloud2Msg>();
    private readonly ConcurrentQueue<PathMsg> receivedTrajectories = new ConcurrentQueue<PathMsg>();
    private readonly Queue<PointCloud2Msg> pendingChunkClouds = new Queue<PointCloud2Msg>();
    private readonly Dictionary<int, ChunkAssembly> chunkAssemblies = new Dictionary<int, ChunkAssembly>();
    private readonly ComputeBuffer[] positionBuffers = new ComputeBuffer[2];
    private readonly ComputeBuffer[] colorBuffers = new ComputeBuffer[2];
    private readonly int[] bufferCapacities = new int[2];
    private readonly int[] bufferCounts = new int[2];
    private readonly Dictionary<int, int> lastCameraDrawFrame = new Dictionary<int, int>();

    private ComputeBuffer dummyIndexBuffer;
    private ComputeBuffer dummyPoseBuffer;
    private int dummyIndexCapacity;
    private Task<DecodedCloud> decodeTask;
    private PointCloud2Msg pendingLatest;
    private bool hasPendingLatest;
    private ChunkAssembly legacyChunkAssembly;
    private int lastDisplayedChunkRevision = -1;
    private int activeBufferIndex;
    private int backBufferIndex = 1;
    private int nextVersion;
    private int receivedMaps;
    private int displayedMaps;
    private int skippedMaps;
    private int failedMaps;
    private int receivedChunks;
    private int completedChunkedMaps;
    private int displayedVersion = -1;
    private string displayedFrameId = "";
    private float lastDecodeSeconds;
    private Stopwatch decodeStopwatch;
    private Matrix4x4 lockedWorldAdjustMatrix = Matrix4x4.identity;
    private bool hasLockedWorldAdjustMatrix;
    private long receivedBytes;
    private long lastTransportLatencyMicroseconds = -1;
    private long lastSourceToSendMicroseconds = -1;
    private long lastSchedulerQueueMicroseconds = -1;
    private PathMsg pendingTrajectory;
    private bool visibleUpdateAwaitingTrajectory;
    private float startedAt;
    private float firstVisibleSeconds = -1f;
    private float lastVisibleSeconds = -1f;
    private bool hasStarted;
    private bool subscribed;
    private bool resourcesInitialized;

    private void InitializeResources()
    {
        if (resourcesInitialized) return;
        resourcesInitialized = true;

        // Optional integrations are references only. Never create or enable
        // another mode/tool behind the Inspector checkbox.
        if (runtimeCollider == null) runtimeCollider = GetComponent<RuntimePointCloudCollider>();
        if (runtimeCollider != null && navigationRobot != null)
            runtimeCollider.controlledRobot = navigationRobot;

        if (pointMaterial == null)
        {
            Shader shader = Shader.Find("Hidden/PointCloudGPU");
            if (shader == null) shader = Shader.Find("Hidden/Point Cloud GPU");
            if (shader != null) pointMaterial = new Material(shader);
        }

        dummyPoseBuffer = new ComputeBuffer(1, sizeof(float) * 16);
        dummyPoseBuffer.SetData(new Matrix4x4[] { Matrix4x4.identity });
    }

    private void OnEnable()
    {
        InitializeResources();
        Camera.onPreCull += OnCameraPreCull;
        RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;
#if UNITY_EDITOR
        SceneView.duringSceneGui += OnSceneViewGui;
#endif
        if (Application.isPlaying && hasStarted) SubscribeRos();
    }

    private void OnDisable()
    {
        Camera.onPreCull -= OnCameraPreCull;
        RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering;
        lastCameraDrawFrame.Clear();
#if UNITY_EDITOR
        SceneView.duringSceneGui -= OnSceneViewGui;
#endif
        if (Application.isPlaying) UnsubscribeRos();
    }

    private void Start()
    {
        hasStarted = true;
        startedAt = Time.realtimeSinceStartup;
        WarnIfMapAndPlayerShareHierarchy();
        SubscribeRos();
    }

    private void SubscribeRos()
    {
        if (subscribed || !isActiveAndEnabled) return;
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PointCloud2Msg>(fullMapTopic, ReceiveFullMap);
        ros.Subscribe<PathMsg>(cameraTrajectoryTopic, ReceiveCameraTrajectory);
        subscribed = true;
        Debug.Log($"[RosTcpMapViewer] Subscribed to {fullMapTopic}");
        Debug.Log($"[RosTcpMapViewer] Subscribed to {cameraTrajectoryTopic}");
    }

    private void UnsubscribeRos()
    {
        if (!subscribed || ros == null) return;
        if (!string.IsNullOrWhiteSpace(fullMapTopic)) ros.Unsubscribe(fullMapTopic);
        if (!string.IsNullOrWhiteSpace(cameraTrajectoryTopic)) ros.Unsubscribe(cameraTrajectoryTopic);
        subscribed = false;
        while (receivedClouds.TryDequeue(out _)) { }
        while (receivedTrajectories.TryDequeue(out _)) { }
        pendingChunkClouds.Clear();
    }

    private void ReceiveFullMap(PointCloud2Msg message)
    {
        if (!isActiveAndEnabled) return;
        receivedMaps++;
        if (message != null && message.data != null)
            Interlocked.Add(ref receivedBytes, message.data.LongLength);
        receivedClouds.Enqueue(message);
        if (verboseLogs)
        {
            Debug.Log($"[RosTcpMapViewer] RX frame={message.header.frame_id} points={SafePointCount(message):N0}");
        }
    }

    private void Update()
    {
        PathMsg newestTrajectory = null;
        while (receivedTrajectories.TryDequeue(out PathMsg trajectory))
        {
            newestTrajectory = trajectory;
        }
        if (newestTrajectory != null) pendingTrajectory = newestTrajectory;
        if (visibleUpdateAwaitingTrajectory && pendingTrajectory != null)
            ApplyPendingTrajectory();

        PointCloud2Msg newestMonolithicMap = null;
        while (receivedClouds.TryDequeue(out PointCloud2Msg message))
        {
            if (TryParseChunkFrameId(message.header.frame_id, out _, out _, out _))
            {
                // Every chunk matters. Do not apply the monolithic-map latest-only
                // policy to a chunk stream.
                pendingChunkClouds.Enqueue(message);
                receivedChunks++;
            }
            else
            {
                if (newestMonolithicMap != null) skippedMaps++;
                newestMonolithicMap = message;
            }
        }

        if (newestMonolithicMap != null)
        {
            if (hasPendingLatest) skippedMaps++;
            pendingLatest = newestMonolithicMap;
            hasPendingLatest = true;
        }

        FinishDecodeIfReady();
        if (decodeTask == null)
        {
            if (pendingChunkClouds.Count > 0)
            {
                StartDecode(pendingChunkClouds.Dequeue());
            }
            else if (hasPendingLatest)
            {
                PointCloud2Msg next = pendingLatest;
                pendingLatest = null;
                hasPendingLatest = false;
                StartDecode(next);
            }
        }

        PruneStaleChunkAssemblies();
    }

    private void ApplyCameraTrajectory(PathMsg message)
    {
        if (runtimeCollider == null || message.poses == null) return;

        Vector3[] points = new Vector3[message.poses.Length];
        int written = 0;
        for (int i = 0; i < message.poses.Length; i++)
        {
            if (message.poses[i] == null || message.poses[i].pose == null) continue;
            double px = message.poses[i].pose.position.x;
            double py = message.poses[i].pose.position.y;
            double pz = message.poses[i].pose.position.z;
            if (!IsFinite((float)px) || !IsFinite((float)py) || !IsFinite((float)pz)) continue;
            points[written++] = new Vector3((float)px, (float)py, (float)pz);
        }

        if (written != points.Length) Array.Resize(ref points, written);
        runtimeCollider.SetMapMatrix(GetMapToWorldMatrix());
        runtimeCollider.SetCameraTrajectory(points);
        if (verboseLogs)
        {
            Debug.Log($"[RosTcpMapViewer] Camera trajectory poses={written:N0} frame={message.header.frame_id}");
        }
    }

    private void ReceiveCameraTrajectory(PathMsg message)
    {
        if (!isActiveAndEnabled) return;
        if (message != null) receivedTrajectories.Enqueue(message);
    }

    private void StartDecode(PointCloud2Msg message)
    {
        int version = nextVersion++;
        bool isChunk = TryParseChunkFrameId(message.header.frame_id, out _, out _, out _);
        decodeStopwatch = Stopwatch.StartNew();
        // hardPointLimit applies to the assembled map, not to each individual chunk.
        int decodePointLimit = isChunk ? 0 : hardPointLimit;
        decodeTask = Task.Run(() => DecodeCloud(message, version, decodePointLimit));
    }

    private void FinishDecodeIfReady()
    {
        if (decodeTask == null || !decodeTask.IsCompleted) return;

        decodeStopwatch.Stop();
        lastDecodeSeconds = (float)decodeStopwatch.Elapsed.TotalSeconds;
        if (decodeTask.IsFaulted)
        {
            failedMaps++;
            Exception exception = decodeTask.Exception != null ? decodeTask.Exception.GetBaseException() : null;
            Debug.LogError($"[RosTcpMapViewer] Decode failed: {exception?.Message}");
            decodeTask = null;
            return;
        }

        DecodedCloud cloud = decodeTask.Result;
        decodeTask = null;
        if (TryParseChunkFrameId(cloud.frameId, out int revision, out int chunkN, out int totalChunks))
        {
            AcceptDecodedChunk(cloud, revision, chunkN, totalChunks);
        }
        else
        {
            DisplayMonolithicCloud(cloud);
        }
    }

    private void DisplayMonolithicCloud(DecodedCloud cloud)
    {
        try
        {
            EnsureBufferCapacity(backBufferIndex, cloud.count);
            positionBuffers[backBufferIndex].SetData(cloud.positions, 0, 0, cloud.count);
            colorBuffers[backBufferIndex].SetData(cloud.colors, 0, 0, cloud.count);
            SwapDisplayedBuffer(cloud.count);
            displayedVersion = cloud.version;
            displayedFrameId = cloud.frameId;
            displayedMaps++;
            UpdateVisibleTiming(cloud.frameId);
            MarkVisibleUpdate();

            Debug.Log(
                $"[RosTcpMapViewer] Displayed version={cloud.version} frame={cloud.frameId} " +
                $"points={cloud.count:N0}/{cloud.sourcePointCount:N0} decode={lastDecodeSeconds:F3}s");
            RequestRuntimeCollider(
                cloud.version,
                new[] { cloud.positions },
                new[] { cloud.colors });
        }
        catch (Exception exception)
        {
            failedMaps++;
            Debug.LogError($"[RosTcpMapViewer] GPU upload failed for frame={cloud.frameId}: {exception.Message}");
        }
    }

    private void AcceptDecodedChunk(DecodedCloud cloud, int revision, int chunkN, int totalChunks)
    {
        if (revision > 0 && revision <= lastDisplayedChunkRevision)
        {
            skippedMaps++;
            if (verboseLogs)
            {
                Debug.Log(
                    $"[RosTcpMapViewer] Drop old chunk rev={revision} " +
                    $"chunk={chunkN}/{totalChunks}; displayedRev={lastDisplayedChunkRevision}");
            }
            return;
        }

        ChunkAssembly assembly = GetOrCreateChunkAssembly(revision, chunkN, totalChunks);
        if (assembly == null) return;

        if (assembly.received[chunkN])
        {
            skippedMaps++;
            if (verboseLogs)
            {
                Debug.LogWarning(
                    $"[RosTcpMapViewer] Duplicate chunk ignored rev={revision} chunk={chunkN}/{totalChunks}");
            }
            return;
        }

        assembly.chunks[chunkN] = cloud;
        assembly.received[chunkN] = true;
        assembly.receivedCount++;
        assembly.sourcePointCount += cloud.sourcePointCount;
        assembly.validPointCount += cloud.count;
        assembly.lastSeenTime = Time.realtimeSinceStartup;

        if (verboseLogs)
        {
            Debug.Log(
                $"[RosTcpMapViewer] Chunk progress rev={revision} " +
                $"{assembly.receivedCount}/{assembly.totalChunks}, " +
                $"points={assembly.validPointCount:N0}");
        }

        if (assembly.receivedCount != assembly.totalChunks) return;

        try
        {
            DisplayChunkAssembly(assembly);
        }
        catch (Exception exception)
        {
            failedMaps++;
            Debug.LogError(
                $"[RosTcpMapViewer] Failed to display assembled map rev={revision}: {exception.Message}");
        }
        finally
        {
            RemoveChunkAssembly(assembly);
        }
    }

    private ChunkAssembly GetOrCreateChunkAssembly(int revision, int chunkN, int totalChunks)
    {
        ChunkAssembly assembly;
        if (revision > 0)
        {
            if (!chunkAssemblies.TryGetValue(revision, out assembly))
            {
                assembly = CreateChunkAssembly(revision, totalChunks);
                chunkAssemblies.Add(revision, assembly);
            }
        }
        else
        {
            // Legacy chunk IDs do not include a revision. A repeated chunk 1 marks
            // the start of the next map.
            bool startsNewLegacyMap =
                legacyChunkAssembly == null ||
                legacyChunkAssembly.totalChunks != totalChunks ||
                (chunkN == 1 && legacyChunkAssembly.received[1]);

            if (startsNewLegacyMap)
            {
                if (legacyChunkAssembly != null && legacyChunkAssembly.receivedCount < legacyChunkAssembly.totalChunks)
                {
                    failedMaps++;
                    Debug.LogWarning(
                        $"[RosTcpMapViewer] Replaced incomplete legacy chunk map " +
                        $"{legacyChunkAssembly.receivedCount}/{legacyChunkAssembly.totalChunks}");
                }
                legacyChunkAssembly = CreateChunkAssembly(0, totalChunks);
            }
            assembly = legacyChunkAssembly;
        }

        if (assembly.totalChunks != totalChunks)
        {
            failedMaps++;
            Debug.LogWarning(
                $"[RosTcpMapViewer] Chunk total mismatch rev={revision}: " +
                $"expected={assembly.totalChunks}, received={totalChunks}");
            return null;
        }

        return assembly;
    }

    private ChunkAssembly CreateChunkAssembly(int revision, int totalChunks)
    {
        float now = Time.realtimeSinceStartup;
        return new ChunkAssembly
        {
            sourceRevision = revision,
            displayVersion = nextVersion++,
            totalChunks = totalChunks,
            chunks = new DecodedCloud[totalChunks + 1],
            received = new bool[totalChunks + 1],
            firstSeenTime = now,
            lastSeenTime = now,
        };
    }

    private void DisplayChunkAssembly(ChunkAssembly assembly)
    {
        if (assembly.validPointCount <= 0)
        {
            throw new InvalidOperationException("Assembled map contains no finite xyz points");
        }
        if (assembly.validPointCount > int.MaxValue || assembly.sourcePointCount > int.MaxValue)
        {
            throw new InvalidOperationException("Assembled map exceeds Int32 point capacity");
        }
        if (hardPointLimit > 0 && assembly.sourcePointCount > hardPointLimit)
        {
            throw new InvalidOperationException(
                $"Assembled map has {assembly.sourcePointCount:N0} points, " +
                $"above hard limit {hardPointLimit:N0}");
        }

        int totalValidPoints = (int)assembly.validPointCount;
        EnsureBufferCapacity(backBufferIndex, totalValidPoints);

        int bufferOffset = 0;
        for (int chunkN = 1; chunkN <= assembly.totalChunks; chunkN++)
        {
            DecodedCloud chunk = assembly.chunks[chunkN];
            if (chunk == null)
            {
                throw new InvalidOperationException(
                    $"Missing chunk {chunkN}/{assembly.totalChunks} during final upload");
            }

            positionBuffers[backBufferIndex].SetData(chunk.positions, 0, bufferOffset, chunk.count);
            colorBuffers[backBufferIndex].SetData(chunk.colors, 0, bufferOffset, chunk.count);
            bufferOffset += chunk.count;
        }

        SwapDisplayedBuffer(totalValidPoints);
        displayedVersion = assembly.sourceRevision > 0
            ? assembly.sourceRevision
            : assembly.displayVersion;
        displayedFrameId = assembly.sourceRevision > 0
            ? $"full_map_rev_{assembly.sourceRevision}"
            : $"full_map_legacy_chunks_{assembly.totalChunks}";
        displayedMaps++;
        completedChunkedMaps++;
        UpdateVisibleTiming(assembly.chunks[assembly.totalChunks].frameId);
        MarkVisibleUpdate();
        if (assembly.sourceRevision > 0)
        {
            lastDisplayedChunkRevision = Math.Max(lastDisplayedChunkRevision, assembly.sourceRevision);
            RemoveOlderChunkAssemblies(lastDisplayedChunkRevision);
        }

        float assemblySeconds = Time.realtimeSinceStartup - assembly.firstSeenTime;
        Debug.Log(
            $"[RosTcpMapViewer] Displayed assembled map rev={assembly.sourceRevision} " +
            $"chunks={assembly.totalChunks} points={totalValidPoints:N0}/{assembly.sourcePointCount:N0} " +
            $"assembly={assemblySeconds:F3}s");

        Vector3[][] collisionChunks = new Vector3[assembly.totalChunks][];
        uint[][] collisionColorChunks = new uint[assembly.totalChunks][];
        for (int chunkN = 1; chunkN <= assembly.totalChunks; chunkN++)
        {
            collisionChunks[chunkN - 1] = assembly.chunks[chunkN].positions;
            collisionColorChunks[chunkN - 1] = assembly.chunks[chunkN].colors;
        }
        RequestRuntimeCollider(displayedVersion, collisionChunks, collisionColorChunks);
    }

    private void RequestRuntimeCollider(
        int revision,
        Vector3[][] pointChunks,
        uint[][] colorChunks)
    {
        if (!buildRuntimeCollision || runtimeCollider == null) return;
        runtimeCollider.SetMapMatrix(GetMapToWorldMatrix());
        runtimeCollider.RequestRebuild(revision, pointChunks, colorChunks);
    }

    private void SwapDisplayedBuffer(int count)
    {
        bufferCounts[backBufferIndex] = count;
        int previousActive = activeBufferIndex;
        activeBufferIndex = backBufferIndex;
        backBufferIndex = previousActive;
        bufferCounts[backBufferIndex] = 0;
    }

    private void RemoveChunkAssembly(ChunkAssembly assembly)
    {
        if (assembly == null) return;
        if (assembly.sourceRevision > 0)
        {
            chunkAssemblies.Remove(assembly.sourceRevision);
        }
        else if (ReferenceEquals(legacyChunkAssembly, assembly))
        {
            legacyChunkAssembly = null;
        }
    }

    private void RemoveOlderChunkAssemblies(int displayedRevision)
    {
        if (chunkAssemblies.Count == 0) return;
        List<int> revisionsToRemove = new List<int>();
        foreach (KeyValuePair<int, ChunkAssembly> pair in chunkAssemblies)
        {
            if (pair.Key <= displayedRevision) revisionsToRemove.Add(pair.Key);
        }
        foreach (int revision in revisionsToRemove)
        {
            chunkAssemblies.Remove(revision);
        }
    }

    private void PruneStaleChunkAssemblies()
    {
        if (chunkAssemblyTimeoutSeconds <= 0f) return;
        float now = Time.realtimeSinceStartup;

        if (chunkAssemblies.Count > 0)
        {
            List<int> staleRevisions = new List<int>();
            foreach (KeyValuePair<int, ChunkAssembly> pair in chunkAssemblies)
            {
                if (now - pair.Value.lastSeenTime > chunkAssemblyTimeoutSeconds)
                {
                    staleRevisions.Add(pair.Key);
                    Debug.LogWarning(
                        $"[RosTcpMapViewer] Timed out incomplete map rev={pair.Key}: " +
                        $"{pair.Value.receivedCount}/{pair.Value.totalChunks} chunks");
                }
            }
            foreach (int revision in staleRevisions)
            {
                chunkAssemblies.Remove(revision);
                failedMaps++;
            }
        }

        if (legacyChunkAssembly != null &&
            now - legacyChunkAssembly.lastSeenTime > chunkAssemblyTimeoutSeconds)
        {
            Debug.LogWarning(
                $"[RosTcpMapViewer] Timed out incomplete legacy map: " +
                $"{legacyChunkAssembly.receivedCount}/{legacyChunkAssembly.totalChunks} chunks");
            legacyChunkAssembly = null;
            failedMaps++;
        }
    }

    private static bool TryParseChunkFrameId(
        string frameId,
        out int revision,
        out int chunkN,
        out int totalChunks)
    {
        revision = chunkN = totalChunks = 0;
        if (string.IsNullOrEmpty(frameId)) return false;

        string core = frameId;
        int pipeIndex = core.IndexOf('|');
        if (pipeIndex >= 0) core = core.Substring(0, pipeIndex);
        if (!core.StartsWith("kf_", StringComparison.Ordinal)) return false;

        string remainder = core.Substring(3);
        if (remainder.StartsWith("999999_", StringComparison.Ordinal))
        {
            string[] parts = remainder.Substring(7).Split('_');
            if (parts.Length == 3 &&
                int.TryParse(parts[0], out int parsedRevision) &&
                int.TryParse(parts[1], out int parsedChunk) &&
                int.TryParse(parts[2], out int parsedTotal) &&
                parsedRevision > 0 &&
                IsValidChunkIndex(parsedChunk, parsedTotal))
            {
                revision = parsedRevision;
                chunkN = parsedChunk;
                totalChunks = parsedTotal;
                return true;
            }

            if (parts.Length == 2 &&
                int.TryParse(parts[0], out int legacyChunk) &&
                int.TryParse(parts[1], out int legacyTotal) &&
                IsValidChunkIndex(legacyChunk, legacyTotal))
            {
                chunkN = legacyChunk;
                totalChunks = legacyTotal;
                return true;
            }
        }

        if (int.TryParse(remainder, out int encodedId) && encodedId > 65535)
        {
            int encodedChunk = encodedId >> 16;
            int encodedTotal = encodedId & 0xFFFF;
            if (IsValidChunkIndex(encodedChunk, encodedTotal))
            {
                chunkN = encodedChunk;
                totalChunks = encodedTotal;
                return true;
            }
        }

        return false;
    }

    private static bool IsValidChunkIndex(int chunkN, int totalChunks)
    {
        return chunkN >= 1 && totalChunks >= 2 && chunkN <= totalChunks && totalChunks < 1024;
    }

    private void EnsureBufferCapacity(int index, int required)
    {
        if (required <= 0) throw new InvalidOperationException("Cannot allocate an empty point cloud buffer");
        if (bufferCapacities[index] >= required) return;

        positionBuffers[index]?.Release();
        colorBuffers[index]?.Release();
        positionBuffers[index] = new ComputeBuffer(required, sizeof(float) * 3);
        colorBuffers[index] = new ComputeBuffer(required, sizeof(uint));
        bufferCapacities[index] = required;
        EnsureDummyIndexCapacity(required);
    }

    private void EnsureDummyIndexCapacity(int required)
    {
        if (dummyIndexCapacity >= required) return;

        dummyIndexBuffer?.Release();
        dummyIndexBuffer = new ComputeBuffer(required, sizeof(uint));
        const int fillBatchSize = 65536;
        uint[] zeros = new uint[Math.Min(fillBatchSize, required)];
        for (int offset = 0; offset < required; offset += zeros.Length)
        {
            int count = Math.Min(zeros.Length, required - offset);
            dummyIndexBuffer.SetData(zeros, 0, offset, count);
        }
        dummyIndexCapacity = required;
    }

    private static DecodedCloud DecodeCloud(PointCloud2Msg message, int version, int pointLimit)
    {
        if (message.is_bigendian) throw new InvalidOperationException("Big-endian PointCloud2 is not supported");

        int sourceCount = SafePointCount(message);
        if (sourceCount <= 0) throw new InvalidOperationException("PointCloud2 is empty");
        if (pointLimit > 0 && sourceCount > pointLimit)
        {
            throw new InvalidOperationException($"PointCloud2 has {sourceCount:N0} points, above hard limit {pointLimit:N0}");
        }

        FindFieldOffsets(message, out int xOffset, out int yOffset, out int zOffset, out int rgbOffset);
        int pointStep = checked((int)message.point_step);
        byte[] data = message.data;
        Vector3[] positions = new Vector3[sourceCount];
        uint[] colors = new uint[sourceCount];
        int written = 0;

        for (int i = 0; i < sourceCount; i++)
        {
            int offset = checked(i * pointStep);
            if (offset + pointStep > data.Length) break;

            float x = BitConverter.ToSingle(data, offset + xOffset);
            float y = BitConverter.ToSingle(data, offset + yOffset);
            float z = BitConverter.ToSingle(data, offset + zOffset);
            if (!IsFinite(x) || !IsFinite(y) || !IsFinite(z)) continue;

            positions[written] = new Vector3(x, y, z);
            colors[written] = rgbOffset >= 0 && offset + rgbOffset + sizeof(uint) <= data.Length
                ? BitConverter.ToUInt32(data, offset + rgbOffset)
                : 0x00808080;
            written++;
        }

        if (written <= 0) throw new InvalidOperationException("PointCloud2 contains no finite xyz points");
        if (written != sourceCount)
        {
            Array.Resize(ref positions, written);
            Array.Resize(ref colors, written);
        }

        return new DecodedCloud
        {
            version = version,
            frameId = message.header.frame_id,
            sourcePointCount = sourceCount,
            positions = positions,
            colors = colors,
            count = written,
        };
    }

    private static void FindFieldOffsets(
        PointCloud2Msg message,
        out int xOffset,
        out int yOffset,
        out int zOffset,
        out int rgbOffset)
    {
        xOffset = yOffset = zOffset = rgbOffset = -1;
        foreach (PointFieldMsg field in message.fields)
        {
            if (field.name == "x") xOffset = (int)field.offset;
            else if (field.name == "y") yOffset = (int)field.offset;
            else if (field.name == "z") zOffset = (int)field.offset;
            else if (string.Equals(field.name, "rgb", StringComparison.OrdinalIgnoreCase) ||
                    string.Equals(field.name, "rgba", StringComparison.OrdinalIgnoreCase) ||
                    string.Equals(field.name, "color", StringComparison.OrdinalIgnoreCase)){
                rgbOffset = (int)field.offset;
            }
        }

        if (xOffset < 0 || yOffset < 0 || zOffset < 0)
        {
            throw new InvalidOperationException($"PointCloud2 is missing xyz fields: {message.header.frame_id}");
        }
    }

    private static int SafePointCount(PointCloud2Msg message)
    {
        long count = (long)message.width * message.height;
        if (count > int.MaxValue) throw new InvalidOperationException("PointCloud2 point count exceeds Int32 capacity");
        return (int)Math.Max(0, count);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private void OnCameraPreCull(Camera camera)
    {
        DrawForCameraOnce(camera);
    }

    private void OnBeginCameraRendering(ScriptableRenderContext context, Camera camera)
    {
        DrawForCameraOnce(camera);
    }

    private void DrawForCameraOnce(Camera camera)
    {
        if (!Application.isPlaying || camera == null) return;
        if (camera.cameraType == CameraType.Preview) return;
#if UNITY_EDITOR
        if (camera.cameraType == CameraType.SceneView && !showInSceneView) return;
        // Unity drives the editor Scene View camera manually, so its enabled
        // property can be false even while it is actively rendering.
        if (camera.cameraType != CameraType.SceneView && !camera.enabled) return;
#else
        if (!camera.enabled) return;
#endif

        int cameraId = camera.GetInstanceID();
        if (lastCameraDrawFrame.TryGetValue(cameraId, out int frame) && frame == Time.frameCount) return;
        lastCameraDrawFrame[cameraId] = Time.frameCount;
        DrawActiveBuffer(camera);
    }

#if UNITY_EDITOR
    private void OnSceneViewGui(SceneView sceneView)
    {
        if (!showInSceneView || !Application.isPlaying) return;
        // Keep the editor camera updating while ROS maps or worldAdjust change.
        sceneView.Repaint();
    }
#endif

    private void DrawActiveBuffer(Camera targetCamera)
    {
        int count = bufferCounts[activeBufferIndex];
        if (pointMaterial == null || count <= 0) return;

        Matrix4x4 adjust = GetMapToWorldMatrix();
        pointMaterial.SetMatrix("_WorldAdjust", adjust);
        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", colorTint);
        pointMaterial.SetBuffer("_Positions", positionBuffers[activeBufferIndex]);
        pointMaterial.SetBuffer("_Colors", colorBuffers[activeBufferIndex]);
        pointMaterial.SetBuffer("_KeyframeIndices", dummyIndexBuffer);
        pointMaterial.SetBuffer("_KeyframePoses", dummyPoseBuffer);

        Vector3 center = adjust.MultiplyPoint3x4(Vector3.zero);
        float scale = Mathf.Max(
            adjust.GetColumn(0).magnitude,
            Mathf.Max(adjust.GetColumn(1).magnitude, adjust.GetColumn(2).magnitude));
        Bounds bounds = new Bounds(center, Vector3.one * Mathf.Max(1f, drawBoundsSize * scale));
        Graphics.DrawProcedural(
            pointMaterial,
            bounds,
            MeshTopology.Points,
            count,
            1,
            targetCamera,
            null,
            ShadowCastingMode.Off,
            false,
            gameObject.layer);
    }

    private Matrix4x4 GetMapToWorldMatrix()
    {
        if (!useWorldAdjust || worldAdjust == null) return Matrix4x4.identity;
        if (!lockWorldAdjustDuringPlay) return worldAdjust.localToWorldMatrix;

        if (!hasLockedWorldAdjustMatrix)
        {
            lockedWorldAdjustMatrix = worldAdjust.localToWorldMatrix;
            hasLockedWorldAdjustMatrix = true;
            if (verboseLogs) Debug.Log("[RosTcpMapViewer] Locked worldAdjust for stable map rendering.");
        }
        return lockedWorldAdjustMatrix;
    }

    [ContextMenu("Recapture World Adjust")]
    public void RecaptureWorldAdjust()
    {
        lockedWorldAdjustMatrix = useWorldAdjust && worldAdjust != null
            ? worldAdjust.localToWorldMatrix
            : Matrix4x4.identity;
        hasLockedWorldAdjustMatrix = true;
    }

    private void WarnIfMapAndPlayerShareHierarchy()
    {
        if (worldAdjust == null || navigationRobot == null) return;
        bool linked =
            worldAdjust == navigationRobot ||
            worldAdjust.IsChildOf(navigationRobot) ||
            navigationRobot.IsChildOf(worldAdjust);
        if (linked)
        {
            Debug.LogWarning(
                "[RosTcpMapViewer] worldAdjust and navigationRobot share a transform hierarchy. " +
                "The map matrix will be locked so player movement cannot move the point cloud.");
        }
    }

    private void OnGUI()
    {
        if (!showStats) return;
        Rect area = new Rect(statsLeft, statsTop, Mathf.Max(260f, statsWidth), 240f);
        GUILayout.BeginArea(area, GUI.skin.box);
        GUI.color = Color.cyan;
        double latencyMs = Interlocked.Read(ref lastTransportLatencyMicroseconds) / 1000.0;
        double sourceToSendMs = Interlocked.Read(ref lastSourceToSendMicroseconds) / 1000.0;
        double schedulerQueueMs = Interlocked.Read(ref lastSchedulerQueueMicroseconds) / 1000.0;
        GUILayout.Label($"[{comparisonLabel}] complete-map replacement  FPS: {1f / Time.smoothDeltaTime:0}");
        GUILayout.Label($"First visible: {(firstVisibleSeconds < 0f ? "waiting" : firstVisibleSeconds.ToString("F2") + " s")}  Last: {(lastVisibleSeconds < 0f ? "-" : lastVisibleSeconds.ToString("F2") + " s")}");
        GUILayout.Label($"Topic: {fullMapTopic}");
        GUILayout.Label($"Points: {bufferCounts[activeBufferIndex]:N0}  Capacity: {bufferCapacities[activeBufferIndex]:N0}");
        GUILayout.Label($"RX: {Interlocked.Read(ref receivedBytes) / 1048576.0:F1} MiB");
        GUILayout.Label($"ROS source→send: {(sourceToSendMs < 0 ? "-" : sourceToSendMs.ToString("F1") + " ms")}  queue: {(schedulerQueueMs < 0 ? "-" : schedulerQueueMs.ToString("F1") + " ms")}");
        GUILayout.Label($"send→Unity callback*: {(latencyMs < 0 ? "-" : latencyMs.ToString("F1") + " ms")}");
        GUILayout.Label($"Version: {displayedVersion}  Decode: {lastDecodeSeconds:F3}s");
        GUILayout.Label(
            $"Maps: {displayedMaps} shown ({completedChunkedMaps} assembled), " +
            $"{receivedMaps} messages received, {skippedMaps} skipped, {failedMaps} failed");
        if (TryGetNewestChunkProgress(out int revision, out int received, out int total, out long points))
        {
            GUILayout.Label(
                $"Assembling: rev={revision} chunks={received}/{total} " +
                $"points={points:N0} queued={pendingChunkClouds.Count}");
        }
        if (buildRuntimeCollision && runtimeCollider != null)
        {
            GUILayout.Label($"Collider: {runtimeCollider.StatusText}");
        }
        if (!string.IsNullOrEmpty(displayedFrameId)) GUILayout.Label($"Frame: {displayedFrameId}");
        GUILayout.EndArea();
    }

    private void MarkVisibleUpdate()
    {
        float visibleAt = Time.realtimeSinceStartup - startedAt;
        if (firstVisibleSeconds < 0f) firstVisibleSeconds = visibleAt;
        lastVisibleSeconds = visibleAt;
        visibleUpdateAwaitingTrajectory = true;
        if (pendingTrajectory != null) ApplyPendingTrajectory();
    }

    private void ApplyPendingTrajectory()
    {
        ApplyCameraTrajectory(pendingTrajectory);
        pendingTrajectory = null;
        visibleUpdateAwaitingTrajectory = false;
    }

    private void UpdateVisibleTiming(string frameId)
    {
        if (TryReadTaggedNanoseconds(frameId, "source_to_sent_ns:", out long sourceToSend))
            Interlocked.Exchange(ref lastSourceToSendMicroseconds, sourceToSend / 1000L);
        if (TryReadTaggedNanoseconds(frameId, "scheduler_queue_ns:", out long queue))
            Interlocked.Exchange(ref lastSchedulerQueueMicroseconds, queue / 1000L);
        if (TryReadTransportSentNs(frameId, out long sentNs))
        {
            long latencyUs = Math.Max(0L, (UnixTimeNanoseconds() - sentNs) / 1000L);
            Interlocked.Exchange(ref lastTransportLatencyMicroseconds, latencyUs);
        }
    }

    private static bool TryReadTaggedNanoseconds(string frameId, string prefix, out long value)
    {
        value = 0L;
        if (string.IsNullOrEmpty(frameId)) return false;
        foreach (string part in frameId.Split('|'))
            if (part.StartsWith(prefix, StringComparison.Ordinal) &&
                long.TryParse(part.Substring(prefix.Length), out value)) return true;
        return false;
    }

    private static bool TryReadTransportSentNs(string frameId, out long sentNs)
    {
        sentNs = 0L;
        if (string.IsNullOrEmpty(frameId)) return false;
        const string prefix = "transport_sent_ns:";
        foreach (string part in frameId.Split('|'))
        {
            if (part.StartsWith(prefix, StringComparison.Ordinal) &&
                long.TryParse(part.Substring(prefix.Length), out sentNs)) return true;
        }
        return false;
    }

    private static long UnixTimeNanoseconds()
    {
        const long unixEpochTicks = 621355968000000000L;
        return (DateTime.UtcNow.Ticks - unixEpochTicks) * 100L;
    }

    private bool TryGetNewestChunkProgress(
        out int revision,
        out int received,
        out int total,
        out long points)
    {
        ChunkAssembly newest = legacyChunkAssembly;
        foreach (KeyValuePair<int, ChunkAssembly> pair in chunkAssemblies)
        {
            if (newest == null || pair.Value.sourceRevision > newest.sourceRevision)
            {
                newest = pair.Value;
            }
        }

        if (newest == null)
        {
            revision = received = total = 0;
            points = 0;
            return false;
        }

        revision = newest.sourceRevision;
        received = newest.receivedCount;
        total = newest.totalChunks;
        points = newest.validPointCount;
        return true;
    }

    private void OnDestroy()
    {
        UnsubscribeRos();
        for (int i = 0; i < 2; i++)
        {
            positionBuffers[i]?.Release();
            colorBuffers[i]?.Release();
        }
        dummyIndexBuffer?.Release();
        dummyPoseBuffer?.Release();
    }
}
