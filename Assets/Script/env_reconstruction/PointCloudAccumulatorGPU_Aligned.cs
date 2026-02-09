// =====================
//  PointCloudAccumulatorGPU with ICP Keyframe Alignment
//  - 每個 keyframe 收到後用 ICP 對齊到前一個 keyframe
//  - 累積變換矩陣，確保所有點雲在同一座標系
//  - 不增加傳輸量，對齊在 Unity 端完成
// =====================

using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using UnityEngine;
using UnityEngine.Rendering;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

[RequireComponent(typeof(RosConnector))]
public class PointCloudAccumulatorGPU_Aligned : MonoBehaviour
{
    public enum EvictionPolicy { FIFO, LRU }

    [Header("ROS Topic")]
    public string topicName = "/mast3r/pointcloud";

    [Header("Voxel & Memory")]
    [Tooltip("Voxel size (m). 1 point per voxel.")]
    [Min(0.001f)] public float voxelSize = 0.05f;

    [Tooltip("Total memory budget (MB). ~16B per point.")]
    public int memoryBudgetMB = 256;

    [Tooltip("Eviction when full")] public EvictionPolicy eviction = EvictionPolicy.FIFO;
    [Tooltip("Max points consumed per frame")] public int maxProcessPerFrame = 250_000;

    [Header("ICP Alignment")]
    [Tooltip("啟用 keyframe 間 ICP 對齊")]
    public bool enableICP = true;

    [Tooltip("ICP 最大迭代次數")]
    public int icpMaxIterations = 50;

    [Tooltip("ICP 最大對應點距離")]
    public float icpMaxDistance = 0.5f;

    [Tooltip("ICP 降採樣體素大小")]
    public float icpDownsampleVoxel = 0.05f;

    [Tooltip("對齊失敗時丟棄 keyframe")]
    public bool discardOnFailure = false;

    [Tooltip("顯示對齊統計")]
    public bool showAlignmentStats = true;

    [Header("Sensor -> Map (Anchor)")]
    public bool autoAnchor = true;
    public bool anchored = false;

    [Header("World Adjust (Draw-Time)")]
    public bool useWorldAdjust = true;
    public Transform worldAdjust;

    [Header("Optional Sensor Offsets")]
    public Vector3 positionOffset = Vector3.zero;
    public Vector3 rotationEuler = Vector3.zero;
    public Vector3 scale = Vector3.one;

    [Header("Rendering")]
    public Material pointMaterial;
    public float pointPixelSize = 3f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;

    [Header("Debug")]
    public bool showStats = true;
    public bool verbose = false;

    // ---- ROS ----
    private RosConnector rosConnector;
    private string subscriberId;

    // ---- Batches (keyframes) ----
    private struct Keyframe
    {
        public Vector3[] pos;
        public uint[] col;
        public int count;
        public int keyframeId;
    }
    private readonly ConcurrentQueue<Keyframe> keyframeQueue = new ConcurrentQueue<Keyframe>();
    private int keyframeCounter = 0;

    // ---- ICP 對齊 ----
    private ICPAlignerUnity icpAligner;
    private Vector3[] previousKeyframePoints;  // 上一個 keyframe 的對齊後點雲 (用於 ICP)
    private Matrix4x4 cumulativeTransform = Matrix4x4.identity;  // 累積變換
    private int alignedKeyframes = 0;
    private int failedKeyframes = 0;

    // ---- Voxel map ----
    private readonly Dictionary<Vector3Int, int> voxelToIndex = new Dictionary<Vector3Int, int>(1 << 20);
    private readonly Queue<Vector3Int> fifoQueue = new Queue<Vector3Int>(1 << 20);
    private readonly LinkedList<Vector3Int> lruList = new LinkedList<Vector3Int>();
    private readonly Dictionary<Vector3Int, LinkedListNode<Vector3Int>> lruNodeMap = new Dictionary<Vector3Int, LinkedListNode<Vector3Int>>(1 << 20);

    // ---- CPU storage ----
    private readonly List<Vector3> positions = new List<Vector3>(1024);
    private readonly List<uint> colors = new List<uint>(1024);

    // ---- GPU buffers ----
    private ComputeBuffer posBuffer;
    private ComputeBuffer colBuffer;
    private int bufferCapacity = 0;
    private bool gpuDirty = false;

    // ---- Derived / temps ----
    private int maxPointsCapacity;
    private Matrix4x4 sensorXform = Matrix4x4.identity;
    private Matrix4x4 T_anchor = Matrix4x4.identity;

    private Vector3[] tmpPositions = new Vector3[8192];
    private uint[] tmpColors = new uint[8192];

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
        maxPointsCapacity = Mathf.Max(1, (memoryBudgetMB * 1024 * 1024) / 16);

        // 初始化 ICP 對齊器
        icpAligner = new ICPAlignerUnity
        {
            MaxIterations = icpMaxIterations,
            Tolerance = 1e-6f,
            MaxCorrespondenceDistance = icpMaxDistance,
            DownsampleVoxelSize = icpDownsampleVoxel
        };
    }

    void OnValidate()
    {
        voxelSize = Mathf.Max(0.001f, voxelSize);
        pointPixelSize = Mathf.Max(1f, pointPixelSize);
        RebuildSensorXform();

        // 更新 ICP 參數
        if (icpAligner != null)
        {
            icpAligner.MaxIterations = icpMaxIterations;
            icpAligner.MaxCorrespondenceDistance = icpMaxDistance;
            icpAligner.DownsampleVoxelSize = icpDownsampleVoxel;
        }
    }

    void RebuildSensorXform()
    {
        sensorXform = Matrix4x4.TRS(positionOffset, Quaternion.Euler(rotationEuler), scale);
    }

    IEnumerator Start()
    {
        while (rosConnector.RosSocket == null) yield return null;
        while (!rosConnector.IsConnected.WaitOne(0)) yield return null;
        try { subscriberId = rosConnector.RosSocket.Subscribe<PointCloud2>(topicName, OnPointCloudReceived); }
        catch (Exception e) { Debug.LogError($"[PC-Aligned] Subscribe failed: {e.Message}"); }
    }

    void Update()
    {
        // 處理 keyframes
        int processed = 0;
        while (keyframeQueue.TryDequeue(out var kf))
        {
            ProcessKeyframe(kf);
            processed += kf.count;
            if (processed >= maxProcessPerFrame) break;
        }

        if (gpuDirty) UploadToGPU();

        if (showStats && Time.frameCount % 120 == 0)
        {
            string alignInfo = enableICP ? $" Aligned:{alignedKeyframes} Failed:{failedKeyframes}" : "";
            Debug.Log($"[PC-Aligned] Points:{positions.Count}/{maxPointsCapacity} Voxels:{voxelToIndex.Count}{alignInfo}");
        }
    }

    void OnDestroy()
    {
        if (!string.IsNullOrEmpty(subscriberId) && rosConnector?.RosSocket != null)
            rosConnector.RosSocket.Unsubscribe(subscriberId);
        ReleaseBuffers();
        voxelToIndex.Clear(); fifoQueue.Clear(); lruList.Clear(); lruNodeMap.Clear();
        positions.Clear(); colors.Clear();
    }

    // ---------------- ROS callback (background thread) ----------------
    void OnPointCloudReceived(PointCloud2 msg)
    {
        try
        {
            int num = (int)(msg.width * msg.height);
            if (num <= 0 || msg.data == null || msg.data.Length == 0) return;
            int step = (int)msg.point_step;

            if (tmpPositions.Length < num) { tmpPositions = new Vector3[num]; tmpColors = new uint[num]; }
            int valid = 0;

            for (int i = 0; i < num; i++)
            {
                int off = i * step; if (off + 15 >= msg.data.Length) break;
                float x = BitConverter.ToSingle(msg.data, off + 0);
                float y = BitConverter.ToSingle(msg.data, off + 4);
                float z = BitConverter.ToSingle(msg.data, off + 8);
                uint rgb = BitConverter.ToUInt32(msg.data, off + 12);

                // 暫時不套用 anchor，先保持原始座標
                // ICP 需要用原始座標來對齊
                Vector3 p = new Vector3(x, y, z);

                // 檢查有效性
                if (float.IsNaN(x) || float.IsInfinity(x)) continue;

                tmpPositions[valid] = p;
                tmpColors[valid] = (rgb & 0x00FFFFFFu);
                valid++;
            }

            if (valid <= 0) return;

            var pos = new Vector3[valid]; Array.Copy(tmpPositions, pos, valid);
            var col = new uint[valid]; Array.Copy(tmpColors, col, valid);

            keyframeQueue.Enqueue(new Keyframe
            {
                pos = pos,
                col = col,
                count = valid,
                keyframeId = System.Threading.Interlocked.Increment(ref keyframeCounter)
            });
        }
        catch { }
    }

    // ---------------- 處理 Keyframe (主執行緒) ----------------
    void ProcessKeyframe(Keyframe kf)
    {
        Vector3[] alignedPoints;

        if (enableICP && previousKeyframePoints != null && previousKeyframePoints.Length > 100)
        {
            // 用 ICP 對齊當前 keyframe 到前一個 keyframe
            var result = icpAligner.Align(kf.pos, previousKeyframePoints);

            if (result.Success)
            {
                // 累積變換
                cumulativeTransform = result.Transform * cumulativeTransform;
                alignedPoints = ICPAlignerUnity.TransformPoints(kf.pos, cumulativeTransform);
                alignedKeyframes++;

                if (showAlignmentStats && verbose)
                {
                    Debug.Log($"[ICP] KF{kf.keyframeId}: Success, error={result.FinalError:F4}m, " +
                              $"inliers={result.Inliers}, iters={result.Iterations}");
                }
            }
            else
            {
                failedKeyframes++;
                if (discardOnFailure)
                {
                    if (verbose) Debug.LogWarning($"[ICP] KF{kf.keyframeId}: Failed, discarding");
                    return;
                }

                // 對齊失敗，使用上一個累積變換
                alignedPoints = ICPAlignerUnity.TransformPoints(kf.pos, cumulativeTransform);
                if (verbose) Debug.LogWarning($"[ICP] KF{kf.keyframeId}: Failed, using previous transform");
            }
        }
        else
        {
            // 第一個 keyframe 或 ICP 禁用
            alignedPoints = kf.pos;

            // 設置 anchor
            if (!anchored && autoAnchor)
            {
                T_anchor = sensorXform;
                anchored = true;
            }
        }

        // 套用 anchor 變換
        Matrix4x4 A = anchored ? T_anchor : sensorXform;
        for (int i = 0; i < alignedPoints.Length; i++)
        {
            alignedPoints[i] = A.MultiplyPoint3x4(alignedPoints[i]);
        }

        // 更新 previousKeyframePoints (用對齊後的點雲)
        previousKeyframePoints = alignedPoints;

        // 添加到體素地圖
        AppendPoints(alignedPoints, kf.col);
    }

    void AppendPoints(Vector3[] pts, uint[] cols)
    {
        float inv = 1f / voxelSize;
        for (int i = 0; i < pts.Length; i++)
        {
            Vector3 p = pts[i];
            int vx = Mathf.FloorToInt(p.x * inv);
            int vy = Mathf.FloorToInt(p.y * inv);
            int vz = Mathf.FloorToInt(p.z * inv);
            var key = new Vector3Int(vx, vy, vz);

            if (voxelToIndex.TryGetValue(key, out int idxExisting))
            {
                if (eviction == EvictionPolicy.LRU && lruNodeMap.TryGetValue(key, out var node))
                { lruList.Remove(node); lruList.AddFirst(node); }
                continue;
            }

            if (positions.Count >= maxPointsCapacity) EvictOne();

            int newIdx = positions.Count;
            positions.Add(p);
            colors.Add(cols[i]);
            voxelToIndex[key] = newIdx;

            if (eviction == EvictionPolicy.FIFO) fifoQueue.Enqueue(key);
            else { var node = new LinkedListNode<Vector3Int>(key); lruList.AddFirst(node); lruNodeMap[key] = node; }
        }
        gpuDirty = true;
    }

    void EvictOne()
    {
        Vector3Int key;
        if (eviction == EvictionPolicy.FIFO) key = fifoQueue.Dequeue();
        else { var tail = lruList.Last; key = tail.Value; lruList.RemoveLast(); lruNodeMap.Remove(key); }

        if (!voxelToIndex.TryGetValue(key, out int idx)) return;
        int last = positions.Count - 1;
        if (idx != last)
        {
            positions[idx] = positions[last];
            colors[idx] = colors[last];
            Vector3 p = positions[idx];
            int vx = Mathf.FloorToInt(p.x / voxelSize);
            int vy = Mathf.FloorToInt(p.y / voxelSize);
            int vz = Mathf.FloorToInt(p.z / voxelSize);
            var movedKey = new Vector3Int(vx, vy, vz);
            voxelToIndex[movedKey] = idx;
        }
        positions.RemoveAt(last);
        colors.RemoveAt(last);
        voxelToIndex.Remove(key);
    }

    // ---------------- GPU upload & draw ----------------
    void EnsureBuffers(int needed)
    {
        if (needed <= bufferCapacity) return;
        int cap = Mathf.NextPowerOfTwo(Mathf.Max(needed, 1024));
        ReleaseBuffers();
        posBuffer = new ComputeBuffer(cap, sizeof(float) * 3);
        colBuffer = new ComputeBuffer(cap, sizeof(uint));
        bufferCapacity = cap;
    }

    void UploadToGPU()
    {
        int n = positions.Count; EnsureBuffers(n);
        if (n > 0) { posBuffer.SetData(positions); colBuffer.SetData(colors); }
        gpuDirty = false;
    }

    void ReleaseBuffers()
    {
        if (posBuffer != null) { posBuffer.Release(); posBuffer = null; }
        if (colBuffer != null) { colBuffer.Release(); colBuffer = null; }
        bufferCapacity = 0;
    }

    void OnRenderObject()
    {
        if (positions.Count == 0 || pointMaterial == null || posBuffer == null) return;

        Matrix4x4 adjMat = (useWorldAdjust && worldAdjust != null) ? worldAdjust.localToWorldMatrix : Matrix4x4.identity;

        pointMaterial.SetBuffer("_Positions", posBuffer);
        pointMaterial.SetBuffer("_Colors", colBuffer);
        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", colorTint);
        pointMaterial.SetMatrix("_WorldAdjust", adjMat);
        pointMaterial.SetPass(0);

        Graphics.DrawProceduralNow(MeshTopology.Points, positions.Count);
    }

    // ---------------- 公開方法 ----------------

    /// <summary>
    /// 重置對齊狀態 (保留現有點雲)
    /// </summary>
    public void ResetAlignment()
    {
        cumulativeTransform = Matrix4x4.identity;
        previousKeyframePoints = null;
        alignedKeyframes = 0;
        failedKeyframes = 0;
        Debug.Log("[PC-Aligned] Alignment reset");
    }

    /// <summary>
    /// 清空所有點雲和對齊狀態
    /// </summary>
    public void ClearAll()
    {
        ResetAlignment();
        positions.Clear();
        colors.Clear();
        voxelToIndex.Clear();
        fifoQueue.Clear();
        lruList.Clear();
        lruNodeMap.Clear();
        gpuDirty = true;
        anchored = false;
        Debug.Log("[PC-Aligned] All data cleared");
    }

    /// <summary>
    /// 獲取對齊統計
    /// </summary>
    public (int aligned, int failed, int total) GetAlignmentStats()
    {
        return (alignedKeyframes, failedKeyframes, alignedKeyframes + failedKeyframes + 1);
    }
}
