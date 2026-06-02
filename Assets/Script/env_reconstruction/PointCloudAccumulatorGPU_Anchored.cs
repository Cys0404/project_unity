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
    public string pointcloudTopic = "/mast3r/pointcloud_in_map"; // [HYBRID FRAMEWORK] Listen to the global map directly
    public string poseArrayTopic = "/mast3r/keyframe_pose_updates";

    [Header("Voxel & Memory (Per Keyframe)")]
    [Tooltip("Voxel size (m) for local spatial hashing.")]
    [Min(0.001f)] public float voxelSize = 0.05f;

    [Tooltip("Maximum number of Keyframes to retain in GPU memory.")]
    public int maxKeyframes = 1000;

    [Tooltip("Maximum points across all keyframes (safety limit).")]
    public int maxGlobalPoints = 5_000_000;

    [Header("Sensor -> Map (Anchor)")]
    [Tooltip("Auto-anchor is disabled for Hybrid framework: pointclouds are already global.")]
    public bool autoAnchor = false;
    public bool anchored = false;

    [Header("World Adjust (Draw-Time)")]
    [Tooltip("Apply an additional overall transform matrix to the entire point cloud.")]
    public bool useWorldAdjust = true;
    public Transform worldAdjust;

    [Header("PGO Settings")]
    [Tooltip("OFF = Old system behaviour (identity pose, display global points as-is).\nON  = Enable delta PGO correction (T_init captured from first PoseArray, ΔT applied on updates).")]
    public bool enablePGO = false;

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
    private struct Batch
    {
        public uint kfId;
        public Vector3[] pos;
        public uint[] col;
        public int count;
    }
    private readonly ConcurrentQueue<Batch> batchQueue = new ConcurrentQueue<Batch>();

    private readonly ConcurrentQueue<PoseArray> poseQueue = new ConcurrentQueue<PoseArray>();

    // ---- CPU Storage: Keyframe Blocks ----
    private class KeyframeBlock
    {
        public uint id;
        public int bufferIndex;
        public List<Vector3> globalPositions = new List<Vector3>(); // [HYBRID] Already in global coordinates
        public List<uint> colors = new List<uint>();
        public HashSet<Vector3Int> voxelSet = new HashSet<Vector3Int>();
        public Matrix4x4 T_init = Matrix4x4.identity; // [HYBRID] Set from FIRST PoseArray update (same space as T_wc_opt)
        public bool T_initSet = false;                 // [HYBRID] True after first PoseArray arrival
    }

    private Dictionary<uint, KeyframeBlock> keyframeBlocks = new Dictionary<uint, KeyframeBlock>();
    private LinkedList<uint> kfLruList = new LinkedList<uint>(); // Track eviction order
    private Dictionary<uint, int> kfToBufferIdx = new Dictionary<uint, int>();
    // [HYBRID] Cache for PoseArray that arrives before the point cloud batch
    private Dictionary<uint, Matrix4x4> pendingPoses = new Dictionary<uint, Matrix4x4>();
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

    // Used to provide a temporary pose for pointclouds arriving before their initial pose update
    private Matrix4x4 lastKnownPose = Matrix4x4.identity;

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

            // Extract Keyframe ID from header.frame_id
            // Format from pc2_to_map: "kf_42|tx:...|ty:...|tz:...|qx:...|qy:...|qz:...|qw:..."
            // We only need kf_id here. T_init is taken from the FIRST PoseArray update (same coord system).
            uint kfId = 0;
            string fid = msg.header.frame_id;
            string[] parts = fid.Split('|');
            if (parts.Length > 0 && parts[0].StartsWith("kf_"))
            {
                uint.TryParse(parts[0].Substring(3), out kfId);
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

                // -----------------------------------------------------------------------
                // Python: T_WC.act(X_canon) [world space] + Z=-Z [Unity Z-forward]
                // This matches the old system convention. After pc2_to_map's static TF,
                // the points are in a Unity-compatible world space.
                // OLD Unity (which worked) used (x, y, z) directly — do the same here.
                // -----------------------------------------------------------------------
                tmpPositions[valid] = new Vector3(x, y, z);
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

            // [HYBRID] Points are in global space → identity means "display as-is".
            // If a PoseArray already arrived for this kf (pendingPoses), set T_init now.
            if (pendingPoses.TryGetValue(b.kfId, out Matrix4x4 cachedPose))
            {
                block.T_init = cachedPose;
                block.T_initSet = true;
                pendingPoses.Remove(b.kfId);
                Debug.Log($"[PC-Sync] kf_{b.kfId}: T_init set from pending pose.");
            }
            keyframePosesArray[bIdx] = Matrix4x4.identity;
            posesDirty = true;
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
            block.globalPositions.Add(p); // [HYBRID] Storing global positions
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
            totalPointsCount -= block.globalPositions.Count;
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
            int count = block.globalPositions.Count;
            uint bIdx = (uint)block.bufferIndex;

            if (flatPositions.Count + count > maxGlobalPoints) break;

            flatPositions.AddRange(block.globalPositions);
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
        // [PGO GATE FIXED] Do NOT return early here! We must always record T_init
        // for new keyframes perfectly, even if PGO is toggled off, so that if the user
        // toggles PGO ON later, the baseline T_init is mathematically correct.

        // msg.header.frame_id: "kf_ids:0,1,5|scales:1.0,1.0,1.0"
        // Python (publish_keyframe_pose_updates) already applied ROS→Unity conversion:
        //   position: (tx, -ty, tz),  quaternion: (-qx, qy, -qz, qw)
        // Use values as-is. Do NOT re-convert here.
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
            if (!uint.TryParse(kfTokens[i], out uint kId)) continue;

            float s = 1.0f;
            float.TryParse(sTokens[i], System.Globalization.NumberStyles.Float,
                           System.Globalization.CultureInfo.InvariantCulture, out s);

            var rosPose = msg.poses[i];
            // -------------------------------------------------------------------------
            // -------------------------------------------------------------------------
            // 還原：使用者表示原來的旋轉（Python 提供的值）是正確的，
            // 之前的反轉補償反而導致世界顛倒。
            // 因此我們直接使用收到的數值。
            float final_qx = (float)rosPose.orientation.x;
            float final_qy = (float)rosPose.orientation.y;
            float final_qz = (float)rosPose.orientation.z;
            float final_qw = (float)rosPose.orientation.w;

            float px_orig = (float)rosPose.position.x;
            float py_orig = (float)rosPose.position.y;
            float pz_orig = (float)rosPose.position.z;

            Vector3 pos = new Vector3(px_orig, py_orig, pz_orig);
            Quaternion rot = new Quaternion(final_qx, final_qy, final_qz, final_qw);
            // -------------------------------------------------------------------------
            // -------------------------------------------------------------------------
            // [CRITICAL SCALE RESTORATION]
            // We previously removed 's' to prevent points from shrinking, but MASt3R uses Sim3
            // where Translation 't' is tied to Scale 's'. Removing 's' but using 't' warped the
            // pivot-center, tearing the map apart during PGO updates! We MUST preserve it in T_wc_opt.
            // (If scale shrinking point clouds is an issue later, we must fix it by scaling deltaT properly)
            Matrix4x4 T_wc_opt = Matrix4x4.TRS(pos, rot, new Vector3(s, s, s));

            if (!kfToBufferIdx.TryGetValue(kId, out int bIdx))
            {
                // Point cloud not yet received — cache pose so AppendBatch can pick it up.
                pendingPoses[kId] = T_wc_opt;
                continue;
            }

            if (!keyframeBlocks.TryGetValue(kId, out KeyframeBlock block)) continue;

            if (!block.T_initSet)
            {
                // [HYBRID] FIRST pose arrival for this keyframe → save as T_init baseline.
                // Both T_init and all future T_wc_opt come from MASt3R (same coordinate space).
                // ΔT will therefore be a pure PGO correction, geometrically meaningful.
                block.T_init = T_wc_opt;
                block.T_initSet = true;
                keyframePosesArray[bIdx] = Matrix4x4.identity; // no delta yet
                Debug.Log($"[PC-Sync] kf_{kId}: T_init captured from first PoseArray.");
            }
            else
            {
                // Subsequent updates → compute ΔT = T_opt · T_init⁻¹
                // Points are in global space; ΔT slides them to PGO-optimized positions.
                if (enablePGO)
                {
                    Matrix4x4 deltaT = T_wc_opt * block.T_init.inverse;
                    // [HYBRID FIX] 移除多餘的 * sensorXform
                    // 點雲本身已經是 global，我們只需要把「PGO誤差(ΔT)」加上去就好。
                    // 否則相機的初始偏轉角度 (sensorXform) 會被旋轉兩次！
                    keyframePosesArray[bIdx] = deltaT;
                }
                else
                {
                    keyframePosesArray[bIdx] = Matrix4x4.identity;
                }
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
