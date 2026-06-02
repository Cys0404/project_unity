using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Collections;
using System.Threading;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

[RequireComponent(typeof(RosConnector))]
public class PointCloudFrameIdDiag : MonoBehaviour
{
    [Header("ROS Topic")]
    public string pointcloudTopic = "/mast3r/pointcloud_in_map";

    [Header("Connection")]
    public bool waitForRosSocket = true;
    public float subscribeRetryIntervalSec = 1.0f;
    public float subscribeTimeoutSec = 30.0f;
    public bool logSubscribeDiagnostics = true;

    [Header("Logging")]
    public bool debugMode = true;
    public bool logChunks = true;
    public bool logIncremental = false;
    public int incrementalLogEveryN = 30;
    public bool showOnGUI = true;

    [Header("Chunk Detection")]
    [Tooltip("避免把一般大數字誤判成 chunk")]
    public int maxReasonableTotalChunks = 1024;

    private RosConnector rosConnector;
    private string subscriptionId;
    private bool isSubscribed;
    private int receiveCallbackCount;

    private struct RxEvent
    {
        public string rawFrameId;
        public string coreFrameId;
        public bool isChunk;
        public int chunkN;
        public int totalChunks;
        public bool isIncremental;
        public int incrementalId;
        public double recvTime;
    }

    private readonly ConcurrentQueue<RxEvent> rxQueue = new ConcurrentQueue<RxEvent>();

    private int totalRx = 0;
    private int totalChunkRx = 0;
    private int totalIncrementalRx = 0;

    private int lastIncrementalId = -1;
    private double lastIncrementalTime = -1.0;

    private int currentChunkTotal = 0;
    private readonly HashSet<int> currentChunkReceived = new HashSet<int>();
    private string currentChunkSequencePreview = "";

    private readonly Queue<string> recentEvents = new Queue<string>();
    private const int MaxRecentEvents = 15;

    void Awake()
    {
        rosConnector = GetComponent<RosConnector>();

        if (rosConnector == null)
        {
            Debug.LogError("[Diag] RosConnector component not found on this GameObject.");
        }
    }

    IEnumerator Start()
    {
        if (rosConnector == null)
            yield break;

        if (string.IsNullOrEmpty(pointcloudTopic))
        {
            Debug.LogError("[Diag] pointcloudTopic is empty. Please set a valid ROS topic.");
            yield break;
        }

        float startTs = Time.realtimeSinceStartup;

        if (waitForRosSocket)
        {
            while (rosConnector.RosSocket == null)
            {
                float elapsed = Time.realtimeSinceStartup - startTs;

                if (logSubscribeDiagnostics)
                {
                    Debug.LogWarning($"[Diag] Waiting for RosSocket... elapsed={elapsed:F1}s");
                }

                if (subscribeTimeoutSec > 0f && elapsed >= subscribeTimeoutSec)
                {
                    Debug.LogError($"[Diag] Subscribe timeout after {elapsed:F1}s. RosSocket is still null.");
                    yield break;
                }

                yield return new WaitForSeconds(subscribeRetryIntervalSec);
            }
        }

        if (rosConnector.RosSocket == null)
        {
            Debug.LogError("[Diag] RosSocket is null. Cannot subscribe.");
            yield break;
        }

        try
        {
            subscriptionId = rosConnector.RosSocket.Subscribe<PointCloud2>(pointcloudTopic, ReceivePointCloud);
            isSubscribed = !string.IsNullOrEmpty(subscriptionId);

            if (isSubscribed)
                Debug.Log($"[Diag] Subscribed to {pointcloudTopic} | id={subscriptionId}");
            else
                Debug.LogWarning($"[Diag] Subscribe returned empty id on topic {pointcloudTopic}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[Diag] Subscribe exception on topic {pointcloudTopic}: {ex.Message}");
        }
    }

    private void ReceivePointCloud(PointCloud2 msg)
    {
        Interlocked.Increment(ref receiveCallbackCount);
        Debug.Log("[Diag] ReceivePointCloud triggered");

        // 這裡刻意保持超輕量
        string raw = msg.header.frame_id ?? "";
        string core = ExtractCoreFrameId(raw);

        bool isChunk = TryParsePackedChunk(core, out int chunkN, out int totalChunks);
        bool isIncremental = false;
        int incrementalId = -1;

        if (!isChunk)
        {
            isIncremental = TryParseIncremental(core, out incrementalId);
        }

        rxQueue.Enqueue(new RxEvent
        {
            rawFrameId = raw,
            coreFrameId = core,
            isChunk = isChunk,
            chunkN = chunkN,
            totalChunks = totalChunks,
            isIncremental = isIncremental,
            incrementalId = incrementalId,
            recvTime = Time.realtimeSinceStartupAsDouble
        });
    }

    void Update()
    {
        while (rxQueue.TryDequeue(out RxEvent ev))
        {
            totalRx++;

            if (ev.isChunk)
            {
                totalChunkRx++;
                HandleChunk(ev);
            }
            else if (ev.isIncremental)
            {
                totalIncrementalRx++;
                HandleIncremental(ev);
            }
            else
            {
                EnqueueRecent($"[Diag] Unknown frame_id: {ev.rawFrameId}");
                Debug.LogWarning($"[Diag] Unknown frame_id: {ev.rawFrameId}");
            }
        }
    }

    private void HandleChunk(RxEvent ev)
    {
        // 如果 total 改變，視為新一輪 chunk burst
        if (currentChunkTotal != ev.totalChunks)
        {
            currentChunkTotal = ev.totalChunks;
            currentChunkReceived.Clear();
            currentChunkSequencePreview = "";
            EnqueueRecent($"[Diag] New chunk burst detected: total={ev.totalChunks}");
            Debug.Log($"[Diag] New chunk burst detected: total={ev.totalChunks}");
        }

        bool isNew = currentChunkReceived.Add(ev.chunkN);

        if (isNew)
        {
            if (currentChunkSequencePreview.Length > 0)
                currentChunkSequencePreview += ", ";
            currentChunkSequencePreview += ev.chunkN.ToString();
        }

        if (debugMode || logChunks)
        {
            Debug.Log($"[Diag] RX Chunk | {ev.chunkN}/{ev.totalChunks} | frame_id={ev.rawFrameId}");
        }

        string seqLog = $"[Diag] Chunk {ev.chunkN}/{ev.totalChunks}";
        Debug.Log(seqLog);
        EnqueueRecent(seqLog);

        if (ev.chunkN == ev.totalChunks)
        {
            string missing = BuildMissingChunkString(ev.totalChunks, currentChunkReceived);
            string summary = $"[Diag] Chunk burst end | total={ev.totalChunks} | received={currentChunkReceived.Count}/{ev.totalChunks} | missing={missing}";
            Debug.Log(summary);
            EnqueueRecent(summary);
        }
    }

    private void HandleIncremental(RxEvent ev)
    {
        bool shouldLog = false;

        if (debugMode)
        {
            shouldLog = true;
        }
        else if (incrementalLogEveryN <= 1)
        {
            shouldLog = logIncremental;
        }
        else
        {
            shouldLog = logIncremental && (totalIncrementalRx % incrementalLogEveryN == 0);
        }

        if (shouldLog)
        {
            Debug.Log($"[Diag] RX Incremental | id={ev.incrementalId}");
        }

        lastIncrementalId = ev.incrementalId;
        lastIncrementalTime = ev.recvTime;
    }

    private string ExtractCoreFrameId(string rawFrameId)
    {
        if (string.IsNullOrEmpty(rawFrameId))
            return "";

        int pipeIdx = rawFrameId.IndexOf('|');
        if (pipeIdx >= 0)
            return rawFrameId.Substring(0, pipeIdx);

        return rawFrameId;
    }

    private bool TryParsePackedChunk(string coreFrameId, out int chunkN, out int totalChunks)
    {
        chunkN = 0;
        totalChunks = 0;

        if (string.IsNullOrEmpty(coreFrameId))
            return false;

        if (!coreFrameId.StartsWith("kf_"))
            return false;

        string idStr = coreFrameId.Substring(3);

        if (!int.TryParse(idStr, out int encodedId))
            return false;

        // 舊式小 id，通常是 incremental
        if (encodedId <= 65535)
            return false;

        int n = encodedId >> 16;
        int t = encodedId & 0xFFFF;

        if (n < 1) return false;
        if (t < 2) return false;
        if (n > t) return false;
        if (t >= maxReasonableTotalChunks) return false;

        chunkN = n;
        totalChunks = t;
        return true;
    }

    private bool TryParseIncremental(string coreFrameId, out int incrementalId)
    {
        incrementalId = -1;

        if (string.IsNullOrEmpty(coreFrameId))
            return false;

        if (!coreFrameId.StartsWith("kf_"))
            return false;

        string idStr = coreFrameId.Substring(3);

        if (!int.TryParse(idStr, out int parsed))
            return false;

        // 這裡只把非 packed chunk 的 kf_xxx 當 incremental
        if (parsed > 65535)
            return false;

        incrementalId = parsed;
        return true;
    }

    private string BuildMissingChunkString(int total, HashSet<int> received)
    {
        List<int> missing = new List<int>();
        for (int i = 1; i <= total; i++)
        {
            if (!received.Contains(i))
                missing.Add(i);
        }

        if (missing.Count == 0)
            return "none";

        return string.Join(",", missing);
    }

    private void EnqueueRecent(string text)
    {
        recentEvents.Enqueue(text);
        while (recentEvents.Count > MaxRecentEvents)
            recentEvents.Dequeue();
    }

    void OnGUI()
    {
        if (!showOnGUI) return;

        GUI.color = Color.green;
        GUILayout.BeginArea(new Rect(10, 10, 700, 400), GUI.skin.box);

        GUILayout.Label("[PointCloudFrameIdDiag]");
        GUILayout.Label($"Topic: {pointcloudTopic}");
        GUILayout.Label($"Subscribed: {isSubscribed} (id={subscriptionId})");
        GUILayout.Label($"Callback Triggered Count: {receiveCallbackCount}");
        GUILayout.Label($"Total RX: {totalRx}");
        GUILayout.Label($"Chunk RX: {totalChunkRx}");
        GUILayout.Label($"Incremental RX: {totalIncrementalRx}");
        GUILayout.Label($"Current Chunk Total: {currentChunkTotal}");
        GUILayout.Label($"Current Chunk Received Count: {currentChunkReceived.Count}");
        GUILayout.Label($"Chunk Sequence Preview: {currentChunkSequencePreview}");

        if (lastIncrementalId >= 0)
            GUILayout.Label($"Last Incremental ID: {lastIncrementalId}");

        GUILayout.Space(10);
        GUILayout.Label("Recent Events:");
        foreach (var line in recentEvents)
        {
            GUILayout.Label(line);
        }

        GUILayout.EndArea();
    }

    void OnDestroy()
    {
        if (rosConnector != null && rosConnector.RosSocket != null && !string.IsNullOrEmpty(subscriptionId))
        {
            rosConnector.RosSocket.Unsubscribe(subscriptionId);
            isSubscribed = false;
        }
    }
}