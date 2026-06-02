using System;
using System.Collections;
using System.Collections.Concurrent;
using UnityEngine;
using UnityEngine.Rendering;
using Vector3 = UnityEngine.Vector3;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Matrix4x4 = UnityEngine.Matrix4x4;

using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

// ==============================================================================
//  PointCloudAccumulatorGPU_Anchored_IBGR
//  - Incremental Build, Global Redraw (IBGR) 架構
//  - 通道A (Incremental): 接收單一 Keyframe (pointcloud_in_map)，漸進式附加到畫面上 (滑順體驗)
//  - 通道B (Global Redraw): 接收完整地圖 (pointcloud_full_map)，並在背後準備好。
//                           一旦收到全量更新，瞬間 Switch (Double Buffering) 覆蓋畫面！
// ==============================================================================

[RequireComponent(typeof(RosConnector))]
public class cross_robot_map : MonoBehaviour
{
    [Header("ROS Topics (IBGR Architecture)")]
    [Tooltip("Optimized full-map topic. Every message on this topic is written to the back buffer and atomically swapped into view.")]
    public string optimizedFullMapTopic = "/multi_robot/optimized_map_points";

    [Tooltip("Incremental preview topics. These are appended to the currently visible active buffer until the next optimized full-map swap.")]
    public string[] incrementalPointcloudTopics = new string[]
    {
        // "/robot1/mast3r/frame_pointcloud",
        // "/robot2/mast3r/frame_pointcloud"
    };

    [Tooltip("Legacy single-topic compatibility path. Leave empty for the current multi-robot pipeline.")]
    public string legacyPointcloudTopic = "";

    [Tooltip("When using legacyPointcloudTopic, force every incoming cloud to be treated as full-map redraw.")]
    public bool legacyTreatIncomingAsFullMap = false;

    [Header("Memory (Full Map)")]
    [Tooltip("Maximum points across the ENTIRE map (safety limit).")]
    public int maxGlobalPoints = 5_000_000;

    [Header("World Adjust (Draw-Time)")]
    [Tooltip("Apply an additional overall transform matrix to the entire point cloud.")]
    public bool useWorldAdjust = true;
    public Transform worldAdjust;

    [Header("Rendering")]
    public Material pointMaterial;
    public float pointPixelSize = 3f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;

    [Header("Debug")]
    public bool showStats = true;

    [Header("Full Map Chunks")]
    [Tooltip("Swap a chunked full map after this fraction of chunks has arrived. Lower values behave more like the old fast-refresh script and tolerate heavy rosbridge loss.")]
    [Range(0.1f, 1.0f)]
    public float fullMapChunkSwapRatio = 0.8f;

    [Tooltip("Also swap once the back buffer has at least this many points, even if the chunk ratio was not reached. This keeps Unity responsive when rosbridge drops many chunks.")]
    public int minFullMapSwapPoints = 0;

    // ---- ROS ----
    private RosConnector rosConnector;
    private string subIdFullMap;
    private readonly System.Collections.Generic.List<string> subIdIncremental = new System.Collections.Generic.List<string>();
    private string subIdLegacyPointCloud;

    // ---- Batches (IPC Decoding to Main Thread) ----
    private struct Batch
    {
        public bool isFullMap;    // true = 舊式非分塊全域更新（相容用）
        public bool isChunk;      // true = 分塊全域地圖片段
        public bool isFirstChunk; // true = 本批次第一塊（應重置 Back Buffer）
        public bool isLastChunk;  // true = 本批次最後一塊（保留相容資訊；實際 swap 由比例閾值決定）
        public int revision;
        public int chunkN;
        public int totalChunks;
        public Vector3[] pos;
        public uint[] col;
        public int count;
    }
    private readonly ConcurrentQueue<Batch> batchQueue = new ConcurrentQueue<Batch>();

    // ---- Double Buffering GPU Memory ----
    // 0 = Buffer A, 1 = Buffer B
    private ComputeBuffer[] posBuffers = new ComputeBuffer[2];
    private ComputeBuffer[] colBuffers = new ComputeBuffer[2];
    private int[] bufferCounts = new int[2];

    private int activeBufferIdx = 0; // Displayed on screen
    private int backBufferIdx = 1;   // Background writing (for Full Map Redraw)

    // 為了相容原本的 PointCloudGPU.shader，準備兩個 Dummy Buffer
    private ComputeBuffer dummyIdxBuffer;
    private ComputeBuffer dummyPoseBuffer;

    void Awake()
    {
        rosConnector = GetComponent<RosConnector>();

        // Setup Shader
        if (pointMaterial == null)
        {
            var sh = Shader.Find("Hidden/PointCloudGPU");
            if (sh == null) sh = Shader.Find("Hidden/Point Cloud GPU");
            if (sh != null) pointMaterial = new Material(sh);
        }

        // Initialize Double Buffers
        for (int i = 0; i < 2; i++)
        {
            posBuffers[i] = new ComputeBuffer(maxGlobalPoints, 12);
            colBuffers[i] = new ComputeBuffer(maxGlobalPoints, 4);
            bufferCounts[i] = 0;
        }

        // Initialize Dummy Buffers to satisfy the existing shader
        dummyIdxBuffer = new ComputeBuffer(maxGlobalPoints, 4);
        uint[] zeroIndices = new uint[maxGlobalPoints]; // All set to 0
        dummyIdxBuffer.SetData(zeroIndices);

        dummyPoseBuffer = new ComputeBuffer(1, 64);
        Matrix4x4[] identities = new Matrix4x4[] { Matrix4x4.identity };
        dummyPoseBuffer.SetData(identities);
    }

    void Start()
    {
        if (!string.IsNullOrWhiteSpace(optimizedFullMapTopic))
        {
            subIdFullMap = rosConnector.RosSocket.Subscribe<PointCloud2>(optimizedFullMapTopic, ReceiveFullMapPointCloud);
            Debug.Log($"[IBGR] Subscribed full-map swap topic: {optimizedFullMapTopic}");
        }

        if (incrementalPointcloudTopics != null)
        {
            foreach (string topic in incrementalPointcloudTopics)
            {
                if (string.IsNullOrWhiteSpace(topic)) continue;
                string subId = rosConnector.RosSocket.Subscribe<PointCloud2>(topic, ReceiveIncrementalPointCloud);
                subIdIncremental.Add(subId);
                Debug.Log($"[IBGR] Subscribed incremental append topic: {topic}");
            }
        }

        if (!string.IsNullOrWhiteSpace(legacyPointcloudTopic))
        {
            subIdLegacyPointCloud = rosConnector.RosSocket.Subscribe<PointCloud2>(legacyPointcloudTopic, ReceiveLegacyPointCloud);
            Debug.Log($"[IBGR] Subscribed legacy pointcloud topic: {legacyPointcloudTopic}");
        }
    }

    // ---- Chunked Full Map State ----
    // 同一個 full-map revision 收到足夠比例的 chunk 後 swap，避免 rosbridge 掉包讓畫面卡死。
    private int _chunkExpectedTotal = 0;
    private int _chunkReceivedSoFar = 0;
    private int _chunkAssemblyRevision = 0;
    private int _chunkSourceRevision = -1;
    private int _lastSwappedSourceRevision = -1;
    private bool[] _chunkReceivedFlags = null;

    /// <summary>
    /// 解析 frame_id 中的分塊資訊。
    /// 支援 ROS 新版格式: kf_999999_REV_N_T
    /// 保留相容 ROS 舊版格式: kf_999999_N_T
    /// 也支援 IPC 模式格式: kf_<encodedId>|tx:... (encodedId = (chunkN << 16) | totalChunks)
    /// </summary>
    private (int revision, int chunkN, int totalChunks) ParseChunkFrameId(string frameId)
    {
        // Step 1: 先去掉 '|' 後面的 pose suffix
        string core = frameId;
        int pipeIdx = core.IndexOf('|');
        if (pipeIdx >= 0) core = core.Substring(0, pipeIdx); // e.g. "kf_65542" 或 "kf_999999_42_1_6"

        // 確認前綴
        if (!core.StartsWith("kf_")) return (0, 0, 0);

        string remainder = core.Substring(3); // 去掉 "kf_"

        // 1. 嘗試解析 ROS 新版格式 "999999_REV_N_T"，REV 讓 Unity 即使漏掉 chunk 1 也能開始新的 back-buffer assembly。
        if (remainder.StartsWith("999999_"))
        {
            var parts = remainder.Substring(7).Split('_');
            if (parts.Length == 3
                && int.TryParse(parts[0], out int rev)
                && int.TryParse(parts[1], out int n)
                && int.TryParse(parts[2], out int t))
            {
                if (rev > 0 && n >= 1 && t >= 2 && n <= t && t < 1024) return (rev, n, t);
            }

            // 舊版相容格式 "999999_N_T"
            if (parts.Length == 2 && int.TryParse(parts[0], out int oldN) && int.TryParse(parts[1], out int oldT))
            {
                if (oldN >= 1 && oldT >= 2 && oldN <= oldT && oldT < 1024) return (0, oldN, oldT);
            }
        }

        // 2. 嘗試解析 IPC 位元打包格式 (encodedId = (N << 16) | T)
        if (int.TryParse(remainder, out int encodedId))
        {
            if (encodedId > 65535)
            {
                int chunkN = encodedId >> 16;
                int totalChunks = encodedId & 0xFFFF;

                if (chunkN >= 1 && totalChunks >= 2 && chunkN <= totalChunks && totalChunks < 1024)
                {
                    return (0, chunkN, totalChunks);
                }
            }
        }

        return (0, 0, 0);
    }

    private void ReceiveFullMapPointCloud(PointCloud2 msg)
    {
        var frameId = msg.header.frame_id;
        var (revision, chunkN, totalChunks) = ParseChunkFrameId(frameId);
        if (chunkN > 0 && totalChunks > 0)
        {
            Debug.Log($"[IBGR] RX Full Map Chunk | rev={revision} chunk={chunkN}/{totalChunks} | raw={frameId}");
            DecodeChunk(msg, revision, chunkN, totalChunks);
            return;
        }

        DecodePointCloud(msg, isFullMap: true);
    }

    private void ReceiveIncrementalPointCloud(PointCloud2 msg)
    {
        DecodePointCloud(msg, isFullMap: false);
    }

    private void ReceiveLegacyPointCloud(PointCloud2 msg)
    {
        var frameId = msg.header.frame_id;

        if (legacyTreatIncomingAsFullMap)
        {
            DecodePointCloud(msg, isFullMap: true);
            return;
        }

        var (revision, chunkN, totalChunks) = ParseChunkFrameId(frameId);
        if (chunkN > 0 && totalChunks > 0)
        {
            Debug.Log($"[IBGR] RX Legacy Chunk | rev={revision} chunk={chunkN}/{totalChunks} | raw={frameId}");
            DecodeChunk(msg, revision, chunkN, totalChunks);
            return;
        }

        if (frameId.Contains("kf_999999"))
        {
            DecodePointCloud(msg, isFullMap: true);
            return;
        }

        DecodePointCloud(msg, isFullMap: false);
    }

    /// <summary>解碼一個分塊，累積到 Back Buffer；最後一塊觸發 Buffer Swap。</summary>
    private void DecodeChunk(PointCloud2 msg, int revision, int chunkN, int totalChunks)
    {
        int pointCount = (int)(msg.width * msg.height);
        if (pointCount == 0) return;

        Vector3[] pArr = new Vector3[pointCount];
        uint[] cArr = new uint[pointCount];

        int xOff = -1, yOff = -1, zOff = -1, rgbOff = -1;
        foreach (var field in msg.fields)
        {
            if (field.name == "x") xOff = (int)field.offset;
            else if (field.name == "y") yOff = (int)field.offset;
            else if (field.name == "z") zOff = (int)field.offset;
            else if (field.name == "rgb" || field.name == "rgba") rgbOff = (int)field.offset;
        }
        int step = (int)msg.point_step;
        byte[] data = msg.data;
        for (int i = 0; i < pointCount; i++)
        {
            int b = i * step;
            pArr[i] = new Vector3(
                BitConverter.ToSingle(data, b + xOff),
                BitConverter.ToSingle(data, b + yOff),
                BitConverter.ToSingle(data, b + zOff));
            cArr[i] = (rgbOff != -1) ? BitConverter.ToUInt32(data, b + rgbOff) : 0xFFFFFFFF;
        }

        batchQueue.Enqueue(new Batch
        {
            isFullMap = false,
            isChunk = true,
            isFirstChunk = (chunkN == 1),
            isLastChunk = (chunkN == totalChunks),
            revision = revision,
            chunkN = chunkN,
            totalChunks = totalChunks,
            pos = pArr,
            col = cArr,
            count = pointCount
        });
    }

    /// <summary>舊式增量或非分塊全域地圖的解碼（相容性保留）</summary>
    private void DecodePointCloud(PointCloud2 msg, bool isFullMap)
    {
        int pointCount = (int)(msg.width * msg.height);
        if (pointCount == 0) return;
        if (pointCount > maxGlobalPoints)
        {
            Debug.LogWarning($"[IBGR] Points ({pointCount}) > maxGlobalPoints. Truncating.");
            pointCount = maxGlobalPoints;
        }

        Vector3[] pArr = new Vector3[pointCount];
        uint[] cArr = new uint[pointCount];

        int xOff = -1, yOff = -1, zOff = -1, rgbOff = -1;
        foreach (var field in msg.fields)
        {
            if (field.name == "x") xOff = (int)field.offset;
            else if (field.name == "y") yOff = (int)field.offset;
            else if (field.name == "z") zOff = (int)field.offset;
            else if (field.name == "rgb" || field.name == "rgba") rgbOff = (int)field.offset;
        }
        int step = (int)msg.point_step;
        byte[] data = msg.data;
        for (int i = 0; i < pointCount; i++)
        {
            int b = i * step;
            pArr[i] = new Vector3(
                BitConverter.ToSingle(data, b + xOff),
                BitConverter.ToSingle(data, b + yOff),
                BitConverter.ToSingle(data, b + zOff));
            cArr[i] = (rgbOff != -1) ? BitConverter.ToUInt32(data, b + rgbOff) : 0xFFFFFFFF;
        }
        batchQueue.Enqueue(new Batch { isFullMap = isFullMap, isChunk = false, pos = pArr, col = cArr, count = pointCount });
    }

    void Update()
    {
        while (batchQueue.TryDequeue(out Batch b))
        {
            if (b.isChunk)
            {
                // =====================================================
                // PGO Chunked Full Map：同一組 chunk 收到足夠比例後 Atomic Swap。
                // =====================================================
                if (b.totalChunks <= 0 || b.chunkN <= 0 || b.chunkN > b.totalChunks)
                {
                    Debug.LogWarning($"[IBGR] Ignoring invalid full-map chunk {b.chunkN}/{b.totalChunks}.");
                    continue;
                }

                bool hasActiveAssembly = _chunkExpectedTotal > 0 && _chunkReceivedFlags != null;

                if (b.revision > 0 && b.revision <= _lastSwappedSourceRevision)
                {
                    Debug.Log($"[IBGR] Dropping old/tail chunk rev={b.revision} chunk={b.chunkN}/{b.totalChunks}; lastSwappedRev={_lastSwappedSourceRevision}.");
                    continue;
                }

                if (!hasActiveAssembly)
                {
                    if (!b.isFirstChunk)
                    {
                        Debug.LogWarning($"[IBGR] Dropping chunk {b.chunkN}/{b.totalChunks} for rev={b.revision}: waiting for chunk 1 to start a clean assembly.");
                        continue;
                    }
                    StartChunkAssembly(b);
                }
                else if (b.revision > 0 && b.revision != _chunkSourceRevision)
                {
                    if (b.revision < _chunkSourceRevision)
                    {
                        Debug.Log($"[IBGR] Dropping stale chunk rev={b.revision} chunk={b.chunkN}/{b.totalChunks}; activeRev={_chunkSourceRevision}.");
                        continue;
                    }

                    if (!b.isFirstChunk)
                    {
                        Debug.LogWarning($"[IBGR] Dropping future chunk rev={b.revision} chunk={b.chunkN}/{b.totalChunks}: activeRev={_chunkSourceRevision}, waiting for chunk 1 before switching revisions.");
                        continue;
                    }
                    StartChunkAssembly(b);
                }
                else if (b.isFirstChunk && _chunkReceivedSoFar > 0)
                {
                    Debug.LogWarning($"[IBGR] Duplicate chunk 1 for activeRev={_chunkSourceRevision}; ignoring restart attempt.");
                    continue;
                }

                if (b.totalChunks != _chunkExpectedTotal)
                {
                    Debug.LogWarning($"[IBGR] Dropping full-map chunk {b.chunkN}/{b.totalChunks}: expected total {_chunkExpectedTotal}.");
                    continue;
                }

                if (_chunkReceivedFlags[b.chunkN])
                {
                    Debug.LogWarning($"[IBGR] Duplicate full-map chunk {b.chunkN}/{b.totalChunks} ignored for rev={_chunkAssemblyRevision}.");
                    continue;
                }

                int backCount = bufferCounts[backBufferIdx];
                int insertCount = Mathf.Min(b.count, maxGlobalPoints - backCount);
                if (insertCount > 0)
                {
                    posBuffers[backBufferIdx].SetData(b.pos, 0, backCount, insertCount);
                    colBuffers[backBufferIdx].SetData(b.col, 0, backCount, insertCount);
                    bufferCounts[backBufferIdx] = backCount + insertCount;
                }

                _chunkReceivedFlags[b.chunkN] = true;
                _chunkReceivedSoFar++;

                int chunksRequiredForSwap = RequiredChunksForSwap(_chunkExpectedTotal);
                bool enoughChunks = _chunkReceivedSoFar >= chunksRequiredForSwap;
                bool enoughPoints = minFullMapSwapPoints > 0 && bufferCounts[backBufferIdx] >= minFullMapSwapPoints;
                if (!enoughChunks && !enoughPoints)
                {
                    Debug.Log($"[IBGR] PGO Chunk progress rev={_chunkAssemblyRevision}: {_chunkReceivedSoFar}/{_chunkExpectedTotal} chunks, need {chunksRequiredForSwap}, backBuffer={bufferCounts[backBufferIdx]:N0} pts, minSwapPts={minFullMapSwapPoints:N0}.");
                    continue;
                }

                // chunk 比例達標 → 原子性翻轉！畫面只閃一幀
                int temp = activeBufferIdx;
                activeBufferIdx = backBufferIdx;
                backBufferIdx = temp;

                // 清空舊的 Active（現在變成 Back），等下次 PGO 使用
                bufferCounts[backBufferIdx] = 0;
                Debug.Log($"[IBGR] ⚡ DB Swap localRev={_chunkAssemblyRevision} sourceRev={_chunkSourceRevision}! Displaying {bufferCounts[activeBufferIdx]:N0} PGO-aligned pts from {_chunkReceivedSoFar}/{_chunkExpectedTotal} chunks (threshold={chunksRequiredForSwap}, ratio={fullMapChunkSwapRatio:0.00}, minSwapPts={minFullMapSwapPoints:N0}).");

                _lastSwappedSourceRevision = _chunkSourceRevision;
                _chunkExpectedTotal = 0;
                _chunkReceivedSoFar = 0;
                _chunkSourceRevision = -1;
                _chunkReceivedFlags = null;
            }
            else if (b.isFullMap)
            {
                // =====================================================
                // 通道 B 舊式：一次性覆寫切換（相容用，理論上不再觸發）
                // =====================================================
                posBuffers[backBufferIdx].SetData(b.pos, 0, 0, b.count);
                colBuffers[backBufferIdx].SetData(b.col, 0, 0, b.count);
                bufferCounts[backBufferIdx] = b.count;

                int temp = activeBufferIdx;
                activeBufferIdx = backBufferIdx;
                backBufferIdx = temp;

                Debug.Log($"[IBGR] ⚡ Full Map DB Swap! Displaying {b.count} points perfectly aligned by PGO.");
            }
            else
            {
                // =====================================================
                // 通道 A：收到單幀點雲 (平時走路時漸進累加)
                // =====================================================
                int curCount = bufferCounts[activeBufferIdx];

                // 確認不會爆掉記憶體上限
                int insertCount = Mathf.Min(b.count, maxGlobalPoints - curCount);
                if (insertCount <= 0) continue;

                // 利用 ComputeBuffer.SetData 的偏移量功能，直接將新的點「附加 (Append)」在現有陣列的尾巴！
                posBuffers[activeBufferIdx].SetData(b.pos, 0, curCount, insertCount);
                colBuffers[activeBufferIdx].SetData(b.col, 0, curCount, insertCount);

                bufferCounts[activeBufferIdx] = curCount + insertCount;

                // Incremental preview only touches the active buffer. The next optimized full-map swap
                // replaces the visible scene and clears the previous active buffer, removing preview ghosts.
            }
        }

        DrawActiveBuffer();
    }

    private void StartChunkAssembly(Batch b)
    {
        _chunkAssemblyRevision++;
        _chunkSourceRevision = b.revision;
        _chunkExpectedTotal = b.totalChunks;
        _chunkReceivedSoFar = 0;
        _chunkReceivedFlags = new bool[b.totalChunks + 1];
        bufferCounts[backBufferIdx] = 0;
        int requiredChunks = RequiredChunksForSwap(_chunkExpectedTotal);
        Debug.Log($"[IBGR] ▶ PGO Chunk START localRev={_chunkAssemblyRevision} sourceRev={_chunkSourceRevision}: expecting {b.totalChunks} chunks, swap after {requiredChunks}; firstSeenChunk={b.chunkN}; active stays visible.");
    }

    private int RequiredChunksForSwap(int totalChunks)
    {
        if (totalChunks <= 0) return 0;
        float ratio = Mathf.Clamp(fullMapChunkSwapRatio, 0.1f, 1.0f);
        return Mathf.Clamp(Mathf.CeilToInt(totalChunks * ratio), 1, totalChunks);
    }

    private void DrawActiveBuffer()
    {
        if (pointMaterial == null) return;
        int activeCount = bufferCounts[activeBufferIdx];
        if (activeCount == 0) return;

        // 計算 World Adjust
        Matrix4x4 adjust = (useWorldAdjust && worldAdjust != null)
                            ? worldAdjust.localToWorldMatrix
                            : Matrix4x4.identity;

        pointMaterial.SetMatrix("_WorldAdjust", adjust);
        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", colorTint);

        // 綁定真實的前景資料 (對應 Shader 內的緩衝區變數名稱)
        pointMaterial.SetBuffer("_Positions", posBuffers[activeBufferIdx]);
        pointMaterial.SetBuffer("_Colors", colBuffers[activeBufferIdx]);

        // 綁定虛假的 Dummy 陣列 (騙過 Shader 讓它做偏移為 0 的計算)
        pointMaterial.SetBuffer("_KeyframeIndices", dummyIdxBuffer);
        pointMaterial.SetBuffer("_KeyframePoses", dummyPoseBuffer);

        // 繪製
        Graphics.DrawProcedural(pointMaterial,
                                new Bounds(Vector3.zero, Vector3.one * 10000f),
                                MeshTopology.Points,
                                activeCount, 1);
    }

    void OnGUI()
    {
        if (!showStats) return;
        GUI.color = Color.cyan;
        GUILayout.Label($"[IBGR Hybrid System] FPS: {1.0f / Time.smoothDeltaTime:0}");
        GUILayout.Label($"Displayed Points: {bufferCounts[activeBufferIdx]:N0} / {maxGlobalPoints}");
        GUILayout.Label($"Active GPU Buffer: {(activeBufferIdx == 0 ? "A" : "B")}");
    }

    void OnDestroy()
    {
        if (rosConnector != null && rosConnector.RosSocket != null)
        {
            if (subIdFullMap != null) rosConnector.RosSocket.Unsubscribe(subIdFullMap);
            foreach (string subId in subIdIncremental)
            {
                if (subId != null) rosConnector.RosSocket.Unsubscribe(subId);
            }
            if (subIdLegacyPointCloud != null) rosConnector.RosSocket.Unsubscribe(subIdLegacyPointCloud);
        }

        for (int i = 0; i < 2; i++)
        {
            posBuffers[i]?.Release();
            colBuffers[i]?.Release();
        }
        dummyIdxBuffer?.Release();
        dummyPoseBuffer?.Release();
    }
}
