using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Vector3 = UnityEngine.Vector3;
using Quaternion = UnityEngine.Quaternion;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Transform = UnityEngine.Transform;

using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

[RequireComponent(typeof(RosConnector))]
public class NativePoseAwareCrossRobotMap : MonoBehaviour
{
    [Header("ROS Topics")]
    public string keyframeCloudTopic = "/multi_robot/native_unity_keyframe_clouds";
    public string poseGraphTopic = "/multi_robot/native_unity_pose_graph";

    [Header("Memory")]
    public int maxKeyframes = 1000;
    public int maxGlobalPoints = 5_000_000;

    [Header("World Adjust")]
    public bool useWorldAdjust = true;
    public Transform worldAdjust;

    [Header("Rendering")]
    public Material pointMaterial;
    public float pointPixelSize = 3f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;

    [Header("Debug")]
    public bool showStats = true;
    public bool verboseCloudLogs = true;

    private RosConnector rosConnector;
    private string cloudSubId;
    private string poseSubId;

    private struct CloudBatch
    {
        public uint keyframeId;
        public Vector3[] positions;
        public uint[] colors;
        public int count;
    }

    private readonly ConcurrentQueue<CloudBatch> cloudQueue = new ConcurrentQueue<CloudBatch>();
    private readonly ConcurrentQueue<string> poseJsonQueue = new ConcurrentQueue<string>();

    private class KeyframeBlock
    {
        public uint id;
        public int bufferIndex;
        public int startPoint;
        public int pointCount;
    }

    private readonly Dictionary<uint, KeyframeBlock> keyframes = new Dictionary<uint, KeyframeBlock>();
    private readonly Dictionary<uint, int> keyframeToBufferIndex = new Dictionary<uint, int>();
    private readonly Queue<int> freeBufferIndices = new Queue<int>();
    private readonly Dictionary<uint, Matrix4x4> pendingPoses = new Dictionary<uint, Matrix4x4>();

    private Matrix4x4[] keyframePoseMatrices;
    private bool posesDirty = false;
    private int totalPoints = 0;
    private int poseRevision = 0;
    private int cloudCallbackCount = 0;
    private int cloudEnqueueCount = 0;

    private ComputeBuffer posBuffer;
    private ComputeBuffer colBuffer;
    private ComputeBuffer idxBuffer;
    private ComputeBuffer poseBuffer;

    [Serializable]
    private class PoseGraphMessage
    {
        public string schema;
        public int revision;
        public int pose_count;
        public string sim3_layout;
        public PoseEntry[] poses;
    }

    [Serializable]
    private class PoseEntry
    {
        public string robot_id;
        public int kf_id;
        public string keyframe_uid;
        public uint global_index;
        public float[] sim3;
    }

    void Awake()
    {
        rosConnector = GetComponent<RosConnector>();
        if (pointMaterial == null)
        {
            var sh = Shader.Find("Hidden/PointCloudGPU");
            if (sh == null) sh = Shader.Find("Hidden/Point Cloud GPU");
            if (sh != null) pointMaterial = new Material(sh);
        }

        keyframePoseMatrices = new Matrix4x4[maxKeyframes];
        for (int i = 0; i < maxKeyframes; i++)
        {
            keyframePoseMatrices[i] = Matrix4x4.identity;
            freeBufferIndices.Enqueue(i);
        }
        AllocateBuffers();
    }

    void Start()
    {
        cloudSubId = rosConnector.RosSocket.Subscribe<PointCloud2>(keyframeCloudTopic, OnKeyframeCloud);
        poseSubId = rosConnector.RosSocket.Subscribe<StringMsg>(poseGraphTopic, OnPoseGraph);
        Debug.Log($"[NativePoseAware] Subscribed clouds={keyframeCloudTopic}, poses={poseGraphTopic}");
    }

    private void OnKeyframeCloud(PointCloud2 msg)
    {
        try
        {
            cloudCallbackCount++;
            string frameId = msg.header != null ? msg.header.frame_id : "";
            uint kfId = ParseKeyframeId(frameId);
            int pointCount = (int)(msg.width * msg.height);
            int dataLength = msg.data != null ? msg.data.Length : 0;
            if (verboseCloudLogs)
            {
                Debug.Log($"[NativePoseAware] Cloud callback #{cloudCallbackCount} frame={frameId} parsed={kfId} points={pointCount:N0} step={msg.point_step} bytes={dataLength:N0}");
            }
            if (kfId == UInt32.MaxValue)
            {
                Debug.LogWarning($"[NativePoseAware] Drop cloud: cannot parse frame_id '{frameId}'");
                return;
            }
            if (pointCount <= 0 || msg.data == null || msg.data.Length == 0)
            {
                Debug.LogWarning($"[NativePoseAware] Drop cloud kf_{kfId}: empty point/data payload");
                return;
            }

            int xOff = -1, yOff = -1, zOff = -1, rgbOff = -1;
            foreach (var field in msg.fields)
            {
                if (field.name == "x") xOff = (int)field.offset;
                else if (field.name == "y") yOff = (int)field.offset;
                else if (field.name == "z") zOff = (int)field.offset;
                else if (field.name == "rgb" || field.name == "rgba") rgbOff = (int)field.offset;
            }
            if (xOff < 0 || yOff < 0 || zOff < 0)
            {
                Debug.LogWarning($"[NativePoseAware] Drop cloud kf_{kfId}: missing xyz fields x={xOff} y={yOff} z={zOff}");
                return;
            }

            Vector3[] positions = new Vector3[pointCount];
            uint[] colors = new uint[pointCount];
            int step = (int)msg.point_step;
            byte[] data = msg.data;
            int valid = 0;
            for (int i = 0; i < pointCount; i++)
            {
                int b = i * step;
                if (b + Math.Max(zOff + 4, rgbOff + 4) > data.Length)
                {
                    Debug.LogWarning($"[NativePoseAware] Truncated cloud kf_{kfId}: stopped at {i:N0}/{pointCount:N0} points");
                    break;
                }
                float x = BitConverter.ToSingle(data, b + xOff);
                float y = BitConverter.ToSingle(data, b + yOff);
                float z = BitConverter.ToSingle(data, b + zOff);
                positions[valid] = new Vector3(x, y, z);
                colors[valid] = (rgbOff >= 0) ? (BitConverter.ToUInt32(data, b + rgbOff) & 0x00FFFFFFu) : 0x00FFFFFFu;
                valid++;
            }
            if (valid <= 0)
            {
                Debug.LogWarning($"[NativePoseAware] Drop cloud kf_{kfId}: decoded zero valid points");
                return;
            }

            if (valid != pointCount)
            {
                Array.Resize(ref positions, valid);
                Array.Resize(ref colors, valid);
            }
            cloudQueue.Enqueue(new CloudBatch { keyframeId = kfId, positions = positions, colors = colors, count = valid });
            cloudEnqueueCount++;
            if (verboseCloudLogs)
            {
                Debug.Log($"[NativePoseAware] Enqueued cloud kf_{kfId} valid={valid:N0} queued={cloudQueue.Count} enqueued={cloudEnqueueCount}");
            }
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"[NativePoseAware] Failed to decode keyframe cloud: {ex.Message}");
        }
    }

    private void OnPoseGraph(StringMsg msg)
    {
        if (msg != null && !string.IsNullOrEmpty(msg.data)) poseJsonQueue.Enqueue(msg.data);
    }

    private uint ParseKeyframeId(string frameId)
    {
        if (string.IsNullOrEmpty(frameId)) return UInt32.MaxValue;
        string core = frameId;
        int pipeIdx = core.IndexOf('|');
        if (pipeIdx >= 0) core = core.Substring(0, pipeIdx);
        if (!core.StartsWith("kf_")) return UInt32.MaxValue;
        if (uint.TryParse(core.Substring(3), out uint id)) return id;
        return UInt32.MaxValue;
    }

    void Update()
    {
        int processedPoints = 0;
        while (cloudQueue.TryDequeue(out CloudBatch batch))
        {
            AddOrReplaceKeyframe(batch);
            processedPoints += batch.count;
            if (processedPoints >= 300_000) break;
        }

        while (poseJsonQueue.TryDequeue(out string json))
        {
            ApplyPoseGraph(json);
        }

        if (posesDirty) UploadPoseBuffer();
        DrawActiveBuffer();

        if (showStats && Time.frameCount % 120 == 0)
        {
            Debug.Log($"[NativePoseAware] pts={totalPoints:N0}/{maxGlobalPoints:N0} keyframes={keyframes.Count}/{maxKeyframes} poseRev={poseRevision} cloudCallbacks={cloudCallbackCount} cloudEnqueued={cloudEnqueueCount} cloudQueue={cloudQueue.Count}");
        }
    }

    private void AddOrReplaceKeyframe(CloudBatch batch)
    {
        if (keyframes.ContainsKey(batch.keyframeId)) return;
        if (freeBufferIndices.Count == 0)
        {
            Debug.LogWarning("[NativePoseAware] No free keyframe pose slots; increase maxKeyframes.");
            return;
        }
        if (totalPoints + batch.count > maxGlobalPoints)
        {
            Debug.LogWarning("[NativePoseAware] maxGlobalPoints reached; dropping new keyframe cloud.");
            return;
        }

        int bufferIndex = freeBufferIndices.Dequeue();
        int startPoint = totalPoints;
        KeyframeBlock block = new KeyframeBlock
        {
            id = batch.keyframeId,
            bufferIndex = bufferIndex,
            startPoint = startPoint,
            pointCount = batch.count
        };
        keyframes[batch.keyframeId] = block;
        keyframeToBufferIndex[batch.keyframeId] = bufferIndex;

        if (pendingPoses.TryGetValue(batch.keyframeId, out Matrix4x4 pose))
        {
            keyframePoseMatrices[bufferIndex] = pose;
            pendingPoses.Remove(batch.keyframeId);
        }
        else
        {
            keyframePoseMatrices[bufferIndex] = Matrix4x4.identity;
        }

        uint[] keyframeIndices = new uint[batch.count];
        for (int i = 0; i < keyframeIndices.Length; i++) keyframeIndices[i] = (uint)bufferIndex;

        posBuffer.SetData(batch.positions, 0, startPoint, batch.count);
        colBuffer.SetData(batch.colors, 0, startPoint, batch.count);
        idxBuffer.SetData(keyframeIndices, 0, startPoint, batch.count);
        totalPoints += batch.count;
        posesDirty = true;
        Debug.Log($"[NativePoseAware] Added kf_{batch.keyframeId} slot={bufferIndex} range=[{startPoint:N0},{totalPoints:N0}) points={batch.count:N0} total={totalPoints:N0}");
    }

    private void ApplyPoseGraph(string json)
    {
        PoseGraphMessage graph = JsonUtility.FromJson<PoseGraphMessage>(json);
        if (graph == null || graph.poses == null) return;
        poseRevision = graph.revision;
        int applied = 0;
        foreach (PoseEntry entry in graph.poses)
        {
            if (entry == null || entry.sim3 == null || entry.sim3.Length < 8) continue;
            Matrix4x4 pose = Sim3ToMatrix(entry.sim3);
            uint key = entry.global_index;
            if (keyframeToBufferIndex.TryGetValue(key, out int bufferIndex))
            {
                keyframePoseMatrices[bufferIndex] = pose;
                applied++;
            }
            else
            {
                pendingPoses[key] = pose;
            }
        }
        posesDirty = true;
        if (showStats)
        {
            Debug.Log($"[NativePoseAware] Pose graph rev={graph.revision} poses={graph.pose_count} applied={applied} pending={pendingPoses.Count}");
        }
    }

    private Matrix4x4 Sim3ToMatrix(float[] sim3)
    {
        Vector3 t = new Vector3(sim3[0], sim3[1], sim3[2]);
        Quaternion q = new Quaternion(sim3[3], sim3[4], sim3[5], sim3[6]);
        float s = sim3[7];
        return Matrix4x4.TRS(t, q, new Vector3(s, s, s));
    }

    private void AllocateBuffers()
    {
        ReleaseBuffers();
        int pointCapacity = Mathf.Max(maxGlobalPoints, 1);
        posBuffer = new ComputeBuffer(pointCapacity, sizeof(float) * 3);
        colBuffer = new ComputeBuffer(pointCapacity, sizeof(uint));
        idxBuffer = new ComputeBuffer(pointCapacity, sizeof(uint));
        poseBuffer = new ComputeBuffer(maxKeyframes, sizeof(float) * 16);
        posesDirty = true;
    }

    private void UploadPoseBuffer()
    {
        if (posesDirty && poseBuffer != null)
        {
            poseBuffer.SetData(keyframePoseMatrices);
            posesDirty = false;
        }
    }

    private void DrawActiveBuffer()
    {
        if (pointMaterial == null || posBuffer == null || totalPoints == 0) return;
        Matrix4x4 adjust = (useWorldAdjust && worldAdjust != null) ? worldAdjust.localToWorldMatrix : Matrix4x4.identity;
        pointMaterial.SetBuffer("_Positions", posBuffer);
        pointMaterial.SetBuffer("_Colors", colBuffer);
        pointMaterial.SetBuffer("_KeyframeIndices", idxBuffer);
        pointMaterial.SetBuffer("_KeyframePoses", poseBuffer);
        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", colorTint);
        pointMaterial.SetMatrix("_WorldAdjust", adjust);
        Graphics.DrawProcedural(pointMaterial, new Bounds(Vector3.zero, Vector3.one * 10000f), MeshTopology.Points, totalPoints, 1);
    }

    private void ReleaseBuffers()
    {
        posBuffer?.Release(); posBuffer = null;
        colBuffer?.Release(); colBuffer = null;
        idxBuffer?.Release(); idxBuffer = null;
        poseBuffer?.Release(); poseBuffer = null;
    }

    void OnDestroy()
    {
        if (rosConnector != null && rosConnector.RosSocket != null)
        {
            if (!string.IsNullOrEmpty(cloudSubId)) rosConnector.RosSocket.Unsubscribe(cloudSubId);
            if (!string.IsNullOrEmpty(poseSubId)) rosConnector.RosSocket.Unsubscribe(poseSubId);
        }
        ReleaseBuffers();
    }
}
