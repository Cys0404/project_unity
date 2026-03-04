// =====================
//  PointCloudAccumulatorGPU_Anchored (Dynamic Pose Graph Sync)
//  - AutoAnchor: Uses the first local pose to pin the environment origin (T_anchor).
//  - UseWorldAdjust: Allows an extra controllable transform on top of the SLAM poses.
//  - Local Pointclouds: Stores raw coordinates and maps them to a Keyframe ID.
//  - Dynamic Pose Updates: Receives `geometry_msgs/PoseArray` with embedded scales.
// =====================

using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using UnityEngine;
using UnityEngine.Rendering;
using Vector3 = UnityEngine.Vector3;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Matrix4x4 = UnityEngine.Matrix4x4;

using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

[RequireComponent(typeof(RosConnector))]
public class PointCloudAccumulatorGPU_Anchored : MonoBehaviour
{
    [Header("ROS Topics")]
    public string pointcloudTopic = "/mast3r/frame_pointcloud";
    public string poseArrayTopic = "/mast3r/keyframe_pose_updates";

    [Header("Voxel & Memory (Per Keyframe)")]
    [Tooltip("Voxel size (m) for local spatial hashing.")]
    [Min(0.001f)] public float voxelSize = 0.05f;

    [Tooltip("Maximum number of Keyframes to retain in GPU memory.")]
    public int maxKeyframes = 1000;

    [Tooltip("Maximum points across all keyframes (safety limit).")]
    public int maxGlobalPoints = 5_000_000;

    [Header("Sensor -> Map (Anchor)")]
    [Tooltip("Automatically anchor the origin based on the first keyframe position.")]
    public bool autoAnchor = true;
    public bool anchored = false;

    [Header("World Adjust (Draw-Time)")]
    [Tooltip("Apply an additional overall transform matrix to the entire point cloud.")]
    public bool useWorldAdjust = true;
    public Transform worldAdjust;

    [Header("Optional Sensor Offsets (pre-anchor)")]
    public Vector3 positionOffset = Vector3.zero;
    public Vector3 rotationEuler = Vector3.zero;
    public Vector3 scale = Vector3.one;

    [Header("Rendering")]
    public Material pointMaterial;
    public float pointPixelSize = 3f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;

    [Header("Debug")]
    public bool showStats = true;

    // ---- ROS ----
    private RosConnector rosConnector;
    private string subIdPointCloud;
    private string subIdPoseArray;

    // ---- Batches ----
    private struct Batch { public uint kfId; public Vector3[] pos; public uint[] col; public int count; }
    private readonly ConcurrentQueue<Batch> batchQueue = new ConcurrentQueue<Batch>();

    private readonly ConcurrentQueue<PoseArray> poseQueue = new ConcurrentQueue<PoseArray>();

    // ---- CPU Storage: Keyframe Blocks ----
    private class KeyframeBlock
    {
        public uint id;
        public int bufferIndex;
        public List<Vector3> localPositions = new List<Vector3>();
        public List<uint> colors = new List<uint>();
        public HashSet<Vector3Int> voxelSet = new HashSet<Vector3Int>();
    }

    private Dictionary<uint, KeyframeBlock> keyframeBlocks = new Dictionary<uint, KeyframeBlock>();
    private LinkedList<uint> kfLruList = new LinkedList<uint>(); // Track eviction order
    private Dictionary<uint, int> kfToBufferIdx = new Dictionary<uint, int>();
    private Queue<int> freeBufferIndices = new Queue<int>();

    // SLAM Dynamic Poses
    private Matrix4x4[] keyframePosesArray;
    private bool posesDirty = false;

    // Global CPU Buffers built from Keyframe blocks
    private List<Vector3> flatPositions = new List<Vector3>(1024);
    private List<uint> flatColors = new List<uint>(1024);
    private List<uint> flatIndices = new List<uint>(1024); // Maps each vertex to a Keyframe ID

    // ---- GPU buffers ----
    private ComputeBuffer posBuffer;      // float3
    private ComputeBuffer colBuffer;      // uint (0xRRGGBB)
    private ComputeBuffer idxBuffer;      // uint (Keyframe ID)
    private ComputeBuffer poseBuffer;     // float4x4 array of size maxKeyframes
    private int bufferCapacity = 0;
    private bool gpuTopologyDirty = false; // Need to re-upload vertices

    private int totalPointsCount = 0;

    private Matrix4x4 sensorXform = Matrix4x4.identity;
    private Matrix4x4 T_anchor_inv = Matrix4x4.identity;

    void Awake()
    {
        rosConnector = GetComponent<RosConnector>();
        if (pointMaterial == null)
        {
            var sh = Shader.Find("Hidden/PointCloudGPU");
            if (sh == null) sh = Shader.Find("Hidden/Point Cloud GPU");
            if (sh != null) pointMaterial = new Material(sh);
        }

        RebuildSensorXform();
        keyframePosesArray = new Matrix4x4[maxKeyframes];
        for (int i = 0; i < maxKeyframes; i++)
        {
            keyframePosesArray[i] = Matrix4x4.identity;
            freeBufferIndices.Enqueue(i);
        }
    }

    void OnValidate()
    {
        voxelSize = Mathf.Max(0.001f, voxelSize);
        pointPixelSize = Mathf.Max(1f, pointPixelSize);
        RebuildSensorXform();
    }

    void RebuildSensorXform()
    {
        sensorXform = Matrix4x4.TRS(positionOffset, Quaternion.Euler(rotationEuler), scale);
    }

    IEnumerator Start()
    {
        while (rosConnector.RosSocket == null) yield return null;
        while (!rosConnector.IsConnected.WaitOne(0)) yield return null;
        try
        {
            subIdPointCloud = rosConnector.RosSocket.Subscribe<PointCloud2>(pointcloudTopic, OnPointCloudReceived);
            subIdPoseArray = rosConnector.RosSocket.Subscribe<PoseArray>(poseArrayTopic, OnPoseArrayReceived);
        }
        catch (Exception e) { Debug.LogError($"[PC-Sync] Subscribe failed: {e.Message}"); }
    }

    void Update()
    {
        // 1. Process local pointcloud batches
        int processed = 0;
        while (batchQueue.TryDequeue(out var b))
        {
            AppendBatch(b);
            processed += b.count;
            if (processed >= 250_000) break;
        }

        // 2. Process pose updates
        while (poseQueue.TryDequeue(out var poseArray))
        {
            ProcessPoseArray(poseArray);
        }

        // 3. Sync GPU Buffers
        if (gpuTopologyDirty || posesDirty)
        {
            UploadToGPU();
        }

        if (showStats && Time.frameCount % 120 == 0)
        {
            Debug.Log($"[PC-Sync] Pts:{totalPointsCount} KFs:{keyframeBlocks.Count}/{maxKeyframes} Anchored:{anchored}");
        }
    }

    void OnDestroy()
    {
        if (rosConnector?.RosSocket != null)
        {
            if (!string.IsNullOrEmpty(subIdPointCloud)) rosConnector.RosSocket.Unsubscribe(subIdPointCloud);
            if (!string.IsNullOrEmpty(subIdPoseArray)) rosConnector.RosSocket.Unsubscribe(subIdPoseArray);
        }
        ReleaseBuffers();
    }

    // ---------------- ROS Callbacks ----------------
    private void OnPointCloudReceived(PointCloud2 msg)
    {
        try
        {
            int num = (int)(msg.width * msg.height);
            if (num <= 0 || msg.data == null || msg.data.Length == 0) return;

            // Extract Keyframe ID from header.frame_id (e.g., "kf_123")
            uint kfId = 0;
            string fid = msg.header.frame_id;
            if (fid.StartsWith("kf_"))
            {
                uint.TryParse(fid.Substring(3), out kfId);
            }

            int step = (int)msg.point_step;
            var tmpPositions = new Vector3[num];
            var tmpColors = new uint[num];

            int valid = 0;
            for (int i = 0; i < num; i++)
            {
                int off = i * step; if (off + 15 >= msg.data.Length) break;
                float x = BitConverter.ToSingle(msg.data, off + 0);
                float y = BitConverter.ToSingle(msg.data, off + 4);
                float z = BitConverter.ToSingle(msg.data, off + 8);
                uint rgb = BitConverter.ToUInt32(msg.data, off + 12);

                // --------------------------------------------------------------------------------
                // ROS/OpenCV Camera Optical (Right-Handed): X Right, Y Down, Z Forward
                // Unity (Left-Handed): X Right, Y Up, Z Forward
                // Conversion Rule: (x_u, y_u, z_u) = (x_r, -y_r, z_r)
                // --------------------------------------------------------------------------------
                tmpPositions[valid] = new Vector3(x, -y, z); // Modified for Left-Handed
                tmpColors[valid] = (rgb & 0x00FFFFFFu);
                valid++;
            }
            if (valid <= 0) return;

            var pos = new Vector3[valid]; Array.Copy(tmpPositions, pos, valid);
            var col = new uint[valid]; Array.Copy(tmpColors, col, valid);
            batchQueue.Enqueue(new Batch { kfId = kfId, pos = pos, col = col, count = valid });
        }
        catch { }
    }

    private void OnPoseArrayReceived(PoseArray msg)
    {
        poseQueue.Enqueue(msg);
    }

    // ---------------- Processing ----------------

    private void AppendBatch(Batch b)
    {
        if (!keyframeBlocks.TryGetValue(b.kfId, out KeyframeBlock block))
        {
            // Evict if we have too many Keyframes
            if (keyframeBlocks.Count >= maxKeyframes) EvictOldestKeyframe();

            int bIdx = freeBufferIndices.Dequeue();
            block = new KeyframeBlock { id = b.kfId, bufferIndex = bIdx };
            keyframeBlocks[b.kfId] = block;
            kfToBufferIdx[b.kfId] = bIdx;
            kfLruList.AddLast(b.kfId);
        }
        else
        {
            // Move to back of LRU
            kfLruList.Remove(b.kfId);
            kfLruList.AddLast(b.kfId);
        }

        float inv = 1f / voxelSize;
        int newlyAdded = 0;

        for (int i = 0; i < b.count; i++)
        {
            Vector3 p = b.pos[i];

            // Local Hash
            int vx = Mathf.FloorToInt(p.x * inv);
            int vy = Mathf.FloorToInt(p.y * inv);
            int vz = Mathf.FloorToInt(p.z * inv);
            var key = new Vector3Int(vx, vy, vz);

            if (block.voxelSet.Contains(key)) continue;

            block.voxelSet.Add(key);
            block.localPositions.Add(p);
            block.colors.Add(b.col[i]);
            newlyAdded++;
        }

        if (newlyAdded > 0)
        {
            totalPointsCount += newlyAdded;

            // Simple approach for now: completely rebuild flat arrays if topology changes often.
            // For extreme performance, you append to the flat lists and only remove when a KF is evicted.
            RebuildFlatArrays();
        }
    }

    private void EvictOldestKeyframe()
    {
        if (kfLruList.Count == 0) return;
        uint oldest = kfLruList.First.Value;
        kfLruList.RemoveFirst();

        if (keyframeBlocks.TryGetValue(oldest, out KeyframeBlock block))
        {
            totalPointsCount -= block.localPositions.Count;
            freeBufferIndices.Enqueue(block.bufferIndex);
            kfToBufferIdx.Remove(oldest);
            keyframeBlocks.Remove(oldest);
        }
    }

    private void RebuildFlatArrays()
    {
        flatPositions.Clear();
        flatColors.Clear();
        flatIndices.Clear();

        if (totalPointsCount > maxGlobalPoints)
        {
            Debug.LogWarning("Exceeded max points. Will clip.");
        }

        foreach (var kvp in keyframeBlocks)
        {
            var block = kvp.Value;
            int count = block.localPositions.Count;
            uint bIdx = (uint)block.bufferIndex;

            if (flatPositions.Count + count > maxGlobalPoints) break;

            flatPositions.AddRange(block.localPositions);
            flatColors.AddRange(block.colors);

            for (int i = 0; i < count; i++)
            {
                flatIndices.Add(bIdx);
            }
        }
        gpuTopologyDirty = true;
    }

    private void ProcessPoseArray(PoseArray msg)
    {
        // msg.header.frame_id encodes kfIds and scales -> "kf_ids:0,1,5|scales:1.0,1.0,1.0"
        string frameIdInfo = msg.header.frame_id;
        string[] parts = frameIdInfo.Split('|');
        if (parts.Length < 2) return;

        string kfStr = parts[0].Replace("kf_ids:", "");
        string sStr = parts[1].Replace("scales:", "");

        string[] kfTokens = kfStr.Split(',');
        string[] sTokens = sStr.Split(',');

        for (int i = 0; i < msg.poses.Length; i++)
        {
            if (i >= kfTokens.Length || i >= sTokens.Length) break;

            if (uint.TryParse(kfTokens[i], out uint kId))
            {
                if (!kfToBufferIdx.TryGetValue(kId, out int bIdx)) continue;

                float s = 1.0f;
                float.TryParse(sTokens[i], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out s);

                var rosPose = msg.poses[i];
                Vector3 pos = new Vector3((float)rosPose.position.x, (float)rosPose.position.y, (float)rosPose.position.z);
                Quaternion rot = new Quaternion((float)rosPose.orientation.x, (float)rosPose.orientation.y, (float)rosPose.orientation.z, (float)rosPose.orientation.w);

                Matrix4x4 T_kf = Matrix4x4.TRS(pos, rot, new Vector3(s, s, s));

                // Auto anchor logic: Make the very first received pose the Map Origin
                if (!anchored && autoAnchor)
                {
                    T_anchor_inv = T_kf.inverse;
                    anchored = true;
                }

                if (autoAnchor && anchored)
                {
                    T_kf = T_anchor_inv * T_kf;
                }

                // Apply Sensor Xform
                keyframePosesArray[bIdx] = T_kf * sensorXform;
            }
        }
        posesDirty = true;
    }

    // ---------------- GPU upload & draw ----------------

    private void EnsureBuffers(int needed)
    {
        if (needed <= bufferCapacity) return;
        int cap = Mathf.NextPowerOfTwo(Mathf.Max(needed, 1024));
        ReleaseBuffers();
        posBuffer = new ComputeBuffer(cap, sizeof(float) * 3);
        colBuffer = new ComputeBuffer(cap, sizeof(uint));
        idxBuffer = new ComputeBuffer(cap, sizeof(uint));

        // Pose buffer is constant size
        poseBuffer = new ComputeBuffer(maxKeyframes, sizeof(float) * 16);
        posesDirty = true; // force pose upload

        bufferCapacity = cap;
    }

    private void UploadToGPU()
    {
        int n = flatPositions.Count;

        if (gpuTopologyDirty && n > 0)
        {
            EnsureBuffers(n);
            posBuffer.SetData(flatPositions);
            colBuffer.SetData(flatColors);
            idxBuffer.SetData(flatIndices);
            gpuTopologyDirty = false;
        }

        if (posesDirty && poseBuffer != null)
        {
            poseBuffer.SetData(keyframePosesArray);
            posesDirty = false;
        }
    }

    private void ReleaseBuffers()
    {
        if (posBuffer != null) { posBuffer.Release(); posBuffer = null; }
        if (colBuffer != null) { colBuffer.Release(); colBuffer = null; }
        if (idxBuffer != null) { idxBuffer.Release(); idxBuffer = null; }
        if (poseBuffer != null) { poseBuffer.Release(); poseBuffer = null; }
        bufferCapacity = 0;
    }

    void OnRenderObject()
    {
        if (flatPositions.Count == 0 || pointMaterial == null || posBuffer == null) return;

        Matrix4x4 W = Matrix4x4.identity;
        if (useWorldAdjust && worldAdjust != null) W = worldAdjust.localToWorldMatrix;

        pointMaterial.SetBuffer("_Positions", posBuffer);
        pointMaterial.SetBuffer("_Colors", colBuffer);
        pointMaterial.SetBuffer("_KeyframeIndices", idxBuffer);
        pointMaterial.SetBuffer("_KeyframePoses", poseBuffer);

        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", colorTint);
        pointMaterial.SetMatrix("_WorldAdjust", W);

        pointMaterial.SetPass(0);
        Graphics.DrawProceduralNow(MeshTopology.Points, flatPositions.Count);
    }
}
