// =====================
// PointCloudKeyframeManager.cs
// 以 Keyframe 為單位管理點雲，支援姿態更新
// =====================

using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using UnityEngine;
using UnityEngine.Rendering;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

[RequireComponent(typeof(RosConnector))]
public class PointCloudKeyframeManager : MonoBehaviour
{
    // ============ 內部類：單個 Keyframe 資料 ============
    private class KeyframeData
    {
        public int keyframeId;
        public List<UnityEngine.Vector3> localPositions;  // 相對於 keyframe 原點的位置
        public List<uint> colors;
        public Matrix4x4 T_WC = Matrix4x4.identity;  // 世界座標變換
        public float scale = 1f;
        public bool gpuDirty = true;
        
        // GPU buffers
        public ComputeBuffer posBuffer;
        public ComputeBuffer colBuffer;
        public int bufferCapacity = 0;
        
        public KeyframeData(int id)
        {
            keyframeId = id;
            localPositions = new List<UnityEngine.Vector3>(4096);
            colors = new List<uint>(4096);
        }
        
        public void EnsureBuffers(int needed)
        {
            if (needed <= bufferCapacity) return;
            int cap = Mathf.NextPowerOfTwo(Mathf.Max(needed, 1024));
            ReleaseBuffers();
            posBuffer = new ComputeBuffer(cap, sizeof(float) * 3);
            colBuffer = new ComputeBuffer(cap, sizeof(uint));
            bufferCapacity = cap;
        }
        
        public void UploadToGPU()
        {
            int n = localPositions.Count;
            EnsureBuffers(n);
            if (n > 0)
            {
                posBuffer.SetData(localPositions);
                colBuffer.SetData(colors);
            }
            gpuDirty = false;
        }
        
        public void ReleaseBuffers()
        {
            if (posBuffer != null) { posBuffer.Release(); posBuffer = null; }
            if (colBuffer != null) { colBuffer.Release(); colBuffer = null; }
            bufferCapacity = 0;
        }
    }
    
    // ============ Inspector 設定 ============
    [Header("ROS Topics")]
    [Tooltip("點雲輸入 (需包含 keyframe ID)")]
    public string pointcloudTopic = "/mast3r/pointcloud_in_map";
    
    [Tooltip("姿態更新輸入")]
    public string poseUpdateTopic = "/mast3r/keyframe_pose_updates_in_map";
    
    [Header("Rendering")]
    public Material pointMaterial;
    public float pointPixelSize = 3f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;
    
    [Header("World Adjust (整體座標系調整)")]
    [Tooltip("用於修正 ROS->Unity 座標系差異，讓點雲方向正確。若為空則使用 Identity。")]
    public UnityEngine.Transform worldAdjust;
    
    [Header("Performance")]
    [Tooltip("每幀最多處理的點數")]
    public int maxProcessPerFrame = 100000;
    
    [Header("Debug")]
    public bool showStats = true;
    public bool verbose = false;
    
    // ============ 私有變數 ============
    private RosConnector rosConnector;
    private string pcSubscriberId;
    private string poseSubscriberId;
    
    // Keyframe 資料儲存
    private readonly Dictionary<int, KeyframeData> keyframes = new Dictionary<int, KeyframeData>();
    
    // 點雲接收隊列
    private struct PointCloudBatch
    {
        public int keyframeId;
        public UnityEngine.Vector3[] positions;
        public uint[] colors;
        public int count;
    }
    private readonly ConcurrentQueue<PointCloudBatch> pointcloudQueue = new ConcurrentQueue<PointCloudBatch>();
    
    private struct PoseUpdate
    {
        public int keyframeId;
        public UnityEngine.Vector3 position;
        public UnityEngine.Quaternion rotation;
        public float scale;
    }
    private readonly ConcurrentQueue<List<PoseUpdate>> poseUpdateQueue = new ConcurrentQueue<List<PoseUpdate>>();
    
    // ============ Unity 生命週期 ============
    void Awake()
    {
        rosConnector = GetComponent<RosConnector>();
        
        if (pointMaterial == null)
        {
            var sh = Shader.Find("Hidden/PointCloudKeyframe");
            if (sh == null) sh = Shader.Find("Hidden/PointCloudGPU");
            if (sh != null) pointMaterial = new Material(sh);
        }
    }
    
    IEnumerator Start()
    {
        while (rosConnector.RosSocket == null) yield return null;
        while (!rosConnector.IsConnected.WaitOne(0)) yield return null;
        
        try
        {
            pcSubscriberId = rosConnector.RosSocket.Subscribe<PointCloud2>(
                pointcloudTopic, OnPointCloudReceived);
            Debug.Log($"[KF-Manager] Subscribed to {pointcloudTopic}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[KF-Manager] PointCloud subscribe failed: {e.Message}");
        }
        
        try
        {
            poseSubscriberId = rosConnector.RosSocket.Subscribe<PoseArray>(
                poseUpdateTopic, OnPoseUpdateReceived);
            Debug.Log($"[KF-Manager] Subscribed to {poseUpdateTopic}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[KF-Manager] PoseUpdate subscribe failed: {e.Message}");
        }
    }
    
    void Update()
    {
        // 處理點雲
        int processed = 0;
        while (pointcloudQueue.TryDequeue(out var batch))
        {
            ProcessPointCloudBatch(batch);
            processed += batch.count;
            if (processed >= maxProcessPerFrame) break;
        }
        
        // 處理姿態更新
        while (poseUpdateQueue.TryDequeue(out var updates))
        {
            ProcessPoseUpdates(updates);
        }
        
        // 上傳 dirty keyframes 到 GPU
        foreach (var kv in keyframes)
        {
            if (kv.Value.gpuDirty)
            {
                kv.Value.UploadToGPU();
            }
        }
        
        // Debug stats
        if (showStats && Time.frameCount % 120 == 0)
        {
            int totalPoints = 0;
            foreach (var kv in keyframes)
                totalPoints += kv.Value.localPositions.Count;
            Debug.Log($"[KF-Manager] Keyframes: {keyframes.Count}, Total points: {totalPoints}");
        }
    }
    
    void OnDestroy()
    {
        if (!string.IsNullOrEmpty(pcSubscriberId) && rosConnector?.RosSocket != null)
            rosConnector.RosSocket.Unsubscribe(pcSubscriberId);
        if (!string.IsNullOrEmpty(poseSubscriberId) && rosConnector?.RosSocket != null)
            rosConnector.RosSocket.Unsubscribe(poseSubscriberId);
        
        foreach (var kv in keyframes)
            kv.Value.ReleaseBuffers();
        keyframes.Clear();
    }
    
    void OnRenderObject()
    {
        if (pointMaterial == null) return;
        
        foreach (var kv in keyframes)
        {
            var kf = kv.Value;
            if (kf.localPositions.Count == 0 || kf.posBuffer == null) continue;
            
            // 計算該 keyframe 的變換矩陣 (worldAdjust * T_WC * scale)
            Matrix4x4 scaleMatrix = Matrix4x4.Scale(UnityEngine.Vector3.one * kf.scale);
            Matrix4x4 W = (worldAdjust != null) ? worldAdjust.localToWorldMatrix : Matrix4x4.identity;
            Matrix4x4 finalTransform = W * kf.T_WC * scaleMatrix;
            
            pointMaterial.SetBuffer("_Positions", kf.posBuffer);
            pointMaterial.SetBuffer("_Colors", kf.colBuffer);
            pointMaterial.SetFloat("_PointSize", pointPixelSize);
            pointMaterial.SetColor("_Tint", colorTint);
            pointMaterial.SetMatrix("_KeyframeTransform", finalTransform);
            
            pointMaterial.SetPass(0);
            Graphics.DrawProceduralNow(MeshTopology.Points, kf.localPositions.Count);
        }
    }
    
    // ============ ROS 回調 (背景執行緒) ============
    void OnPointCloudReceived(PointCloud2 msg)
    {
        try
        {
            // 解析 keyframe ID from header.frame_id
            // 格式: "map|kf_id:5" 或 "kf_id:5"
            int keyframeId = ParseKeyframeId(msg.header.frame_id);
            if (keyframeId < 0)
            {
                if (verbose) Debug.LogWarning($"[KF-Manager] No keyframe ID in frame_id: {msg.header.frame_id}");
                return;
            }
            
            int num = (int)(msg.width * msg.height);
            if (num <= 0 || msg.data == null || msg.data.Length == 0) return;
            int step = (int)msg.point_step;
            
            var positions = new UnityEngine.Vector3[num];
            var colors = new uint[num];
            int valid = 0;
            
            for (int i = 0; i < num; i++)
            {
                int off = i * step;
                if (off + 15 >= msg.data.Length) break;
                
                float x = BitConverter.ToSingle(msg.data, off + 0);
                float y = BitConverter.ToSingle(msg.data, off + 4);
                float z = BitConverter.ToSingle(msg.data, off + 8);
                uint rgb = BitConverter.ToUInt32(msg.data, off + 12);
                
                positions[valid] = new UnityEngine.Vector3(x, y, z);
                colors[valid] = (rgb & 0x00FFFFFFu);
                valid++;
            }
            
            if (valid > 0)
            {
                pointcloudQueue.Enqueue(new PointCloudBatch
                {
                    keyframeId = keyframeId,
                    positions = positions,
                    colors = colors,
                    count = valid
                });
            }
        }
        catch (Exception e)
        {
            if (verbose) Debug.LogError($"[KF-Manager] PointCloud parse error: {e.Message}");
        }
    }
    
    void OnPoseUpdateReceived(PoseArray msg)
    {
        try
        {
            // 解析 header.frame_id: "kf_ids:0,1,5|scales:1.0,1.0,1.0"
            var (kfIds, scales) = ParsePoseUpdateHeader(msg.header.frame_id);
            if (kfIds == null || kfIds.Length == 0) return;
            
            var updates = new List<PoseUpdate>();
            for (int i = 0; i < Mathf.Min(kfIds.Length, msg.poses.Length); i++)
            {
                var pose = msg.poses[i];
                float scale = (i < scales.Length) ? scales[i] : 1f;
                
                updates.Add(new PoseUpdate
                {
                    keyframeId = kfIds[i],
                    position = new UnityEngine.Vector3(
                        (float)pose.position.x,
                        (float)pose.position.y,
                        (float)pose.position.z
                    ),
                    rotation = new UnityEngine.Quaternion(
                        (float)pose.orientation.x,
                        (float)pose.orientation.y,
                        (float)pose.orientation.z,
                        (float)pose.orientation.w
                    ),
                    scale = scale
                });
            }
            
            if (updates.Count > 0)
            {
                poseUpdateQueue.Enqueue(updates);
            }
        }
        catch (Exception e)
        {
            if (verbose) Debug.LogError($"[KF-Manager] PoseUpdate parse error: {e.Message}");
        }
    }
    
    // ============ 主執行緒處理 ============
    void ProcessPointCloudBatch(PointCloudBatch batch)
    {
        if (!keyframes.TryGetValue(batch.keyframeId, out var kf))
        {
            kf = new KeyframeData(batch.keyframeId);
            keyframes[batch.keyframeId] = kf;
            if (verbose) Debug.Log($"[KF-Manager] Created keyframe {batch.keyframeId}");
        }
        
        for (int i = 0; i < batch.count; i++)
        {
            kf.localPositions.Add(batch.positions[i]);
            kf.colors.Add(batch.colors[i]);
        }
        kf.gpuDirty = true;
    }
    
    void ProcessPoseUpdates(List<PoseUpdate> updates)
    {
        foreach (var update in updates)
        {
            if (keyframes.TryGetValue(update.keyframeId, out var kf))
            {
                // 更新變換矩陣
                kf.T_WC = Matrix4x4.TRS(update.position, update.rotation, UnityEngine.Vector3.one);
                kf.scale = update.scale;
                
                if (verbose)
                    Debug.Log($"[KF-Manager] Updated pose for keyframe {update.keyframeId}: pos={update.position}, scale={update.scale}");
            }
            else
            {
                // Keyframe 尚未收到點雲，先建立空的
                kf = new KeyframeData(update.keyframeId);
                kf.T_WC = Matrix4x4.TRS(update.position, update.rotation, UnityEngine.Vector3.one);
                kf.scale = update.scale;
                keyframes[update.keyframeId] = kf;
            }
        }
    }
    
    // ============ 解析輔助函數 ============
    int ParseKeyframeId(string frameId)
    {
        // 格式: "map|kf_id:5" 或 "kf_id:5"
        if (string.IsNullOrEmpty(frameId)) return -1;
        
        int idx = frameId.IndexOf("kf_id:");
        if (idx < 0) return -1;
        
        string numPart = frameId.Substring(idx + 6);
        int endIdx = numPart.IndexOfAny(new char[] { '|', ' ', '\0' });
        if (endIdx > 0) numPart = numPart.Substring(0, endIdx);
        
        if (int.TryParse(numPart, out int kfId))
            return kfId;
        return -1;
    }
    
    (int[], float[]) ParsePoseUpdateHeader(string frameId)
    {
        // 格式: "kf_ids:0,1,5|scales:1.0,1.0,1.0"
        if (string.IsNullOrEmpty(frameId)) return (null, null);
        
        int[] kfIds = null;
        float[] scales = null;
        
        // Parse kf_ids
        int kfIdx = frameId.IndexOf("kf_ids:");
        if (kfIdx >= 0)
        {
            int start = kfIdx + 7;
            int end = frameId.IndexOf('|', start);
            string kfStr = (end > 0) ? frameId.Substring(start, end - start) : frameId.Substring(start);
            var parts = kfStr.Split(',');
            kfIds = new int[parts.Length];
            for (int i = 0; i < parts.Length; i++)
            {
                if (int.TryParse(parts[i].Trim(), out int id))
                    kfIds[i] = id;
            }
        }
        
        // Parse scales
        int scaleIdx = frameId.IndexOf("scales:");
        if (scaleIdx >= 0)
        {
            int start = scaleIdx + 7;
            int end = frameId.IndexOf('|', start);
            string scaleStr = (end > 0) ? frameId.Substring(start, end - start) : frameId.Substring(start);
            var parts = scaleStr.Split(',');
            scales = new float[parts.Length];
            for (int i = 0; i < parts.Length; i++)
            {
                if (float.TryParse(parts[i].Trim(), out float s))
                    scales[i] = s;
                else
                    scales[i] = 1f;
            }
        }
        
        return (kfIds, scales ?? new float[0]);
    }
    
    // ============ Public API ============
    [ContextMenu("Clear All Keyframes")]
    public void ClearAll()
    {
        foreach (var kv in keyframes)
            kv.Value.ReleaseBuffers();
        keyframes.Clear();
        Debug.Log("[KF-Manager] Cleared all keyframes");
    }
    
    public int GetKeyframeCount() => keyframes.Count;
    
    public int GetTotalPointCount()
    {
        int total = 0;
        foreach (var kv in keyframes)
            total += kv.Value.localPositions.Count;
        return total;
    }
}
