using System;
using System.Collections;
using System.Collections.Concurrent;
using System.IO;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using UnityEditor;
#endif
using Vector3 = UnityEngine.Vector3;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Transform = UnityEngine.Transform;

using RosSharp.RosBridgeClient;
using StringMsg = RosSharp.RosBridgeClient.MessageTypes.Std.String;

[RequireComponent(typeof(RosConnector))]
public class MultiRobotPlyMapViewer : MonoBehaviour
{
    [Header("ROS Notification")]
    public string plyUpdateTopic = "/multi_robot/optimized_map_ply_updates";

    [Header("Download")]
    public bool preferLatestUrl = false;
    public bool useDownloadHandlerFile = true;
    public int downloadTimeoutSec = 30;
    public float absoluteDownloadTimeoutSec = 30f;
    public float downloadStallTimeoutSec = 10f;
    public float downloadProgressLogIntervalSec = 1f;
    public bool skipOlderPendingUpdates = true;
    public bool loadManifestOnStart = true;
    public string manifestUrl = "http://192.168.10.10:8012/map_latest.json";
    public bool pollManifest = true;
    public float manifestPollIntervalSec = 2.0f;

    [Header("Memory")]
    public int maxGlobalPoints = 10_000_000;

    [Header("World Adjust")]
    public bool useWorldAdjust = true;
    public Transform worldAdjust;

    [Header("Rendering")]
    public Material pointMaterial;
    public float pointPixelSize = 3f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;

    [Header("Editor View")]
    public bool showInSceneView = true;

    [Header("Debug")]
    public bool showStats = true;
    public bool verboseLogs = true;

    private struct PlyUpdate
    {
        public int version;
        public string url;
        public string filename;
        public int pointCount;
        public long fileSizeBytes;
    }

    private struct ParsedPly
    {
        public int version;
        public string url;
        public Vector3[] positions;
        public uint[] colors;
        public int count;
        public int sourcePointCount;
    }

    private RosConnector rosConnector;
    private string subId;
    private readonly ConcurrentQueue<PlyUpdate> updateQueue = new ConcurrentQueue<PlyUpdate>();

    private ComputeBuffer[] posBuffers = new ComputeBuffer[2];
    private ComputeBuffer[] colBuffers = new ComputeBuffer[2];
    private int[] bufferCounts = new int[2];
    private int activeBufferIdx = 0;
    private int backBufferIdx = 1;

    private ComputeBuffer dummyIdxBuffer;
    private ComputeBuffer dummyPoseBuffer;

    private bool isLoading = false;
    private bool isPollingManifest = false;
    private PlyUpdate? pendingLatest = null;
    private int latestRequestedVersion = -1;
    private int displayedVersion = -1;
    private string displayedUrl = "";
    private float lastDownloadSec = 0f;
    private float lastParseSec = 0f;
    private int downloadedUpdates = 0;
    private int droppedUpdates = 0;
    private int failedUpdates = 0;

    void Awake()
    {
        rosConnector = GetComponent<RosConnector>();
        Debug.Log("[PlyMapViewer] Awake");

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

        dummyIdxBuffer = new ComputeBuffer(maxGlobalPoints, 4);
        uint[] zeroIndices = new uint[maxGlobalPoints];
        dummyIdxBuffer.SetData(zeroIndices);

        dummyPoseBuffer = new ComputeBuffer(1, 64);
        dummyPoseBuffer.SetData(new Matrix4x4[] { Matrix4x4.identity });
    }

#if UNITY_EDITOR
    void OnEnable()
    {
        SceneView.duringSceneGui += OnSceneViewGui;
    }

    void OnDisable()
    {
        SceneView.duringSceneGui -= OnSceneViewGui;
    }

    private void OnSceneViewGui(SceneView sceneView)
    {
        if (!showInSceneView || !Application.isPlaying) return;
        if (Event.current == null || Event.current.type != EventType.Repaint) return;
        if (bufferCounts[activeBufferIdx] <= 0) return;

        DrawActiveBuffer();
        sceneView.Repaint();
    }
#endif

    void Start()
    {
        StartCoroutine(Initialize());
    }

    private IEnumerator Initialize()
    {
        Debug.Log("[PlyMapViewer] Initialize");

        if (loadManifestOnStart && !string.IsNullOrWhiteSpace(manifestUrl))
        {
            yield return StartCoroutine(DownloadManifestAndQueue(manifestUrl));
        }

        if (pollManifest && !string.IsNullOrWhiteSpace(manifestUrl))
        {
            StartCoroutine(PollManifestLoop());
        }

        if (!string.IsNullOrWhiteSpace(plyUpdateTopic))
        {
            StartCoroutine(SubscribeWhenRosReady());
        }
    }

    private IEnumerator SubscribeWhenRosReady()
    {
        float start = Time.realtimeSinceStartup;
        while (rosConnector == null || rosConnector.RosSocket == null)
        {
            if (Time.realtimeSinceStartup - start > 5f)
            {
                Debug.LogWarning("[PlyMapViewer] Waiting for RosConnector.RosSocket; manifest polling will continue without rosbridge notifications");
                start = Time.realtimeSinceStartup;
            }
            yield return null;
        }
        subId = rosConnector.RosSocket.Subscribe<StringMsg>(plyUpdateTopic, ReceivePlyUpdate);
        Debug.Log($"[PlyMapViewer] Subscribed to {plyUpdateTopic}");
    }

    private IEnumerator PollManifestLoop()
    {
        while (true)
        {
            yield return new WaitForSeconds(Mathf.Max(0.5f, manifestPollIntervalSec));
            if (!isPollingManifest)
            {
                yield return StartCoroutine(DownloadManifestAndQueue(manifestUrl));
            }
        }
    }

    private IEnumerator DownloadManifestAndQueue(string url)
    {
        isPollingManifest = true;
        if (verboseLogs) Debug.Log($"[PlyMapViewer] Loading manifest {url}");
        using (UnityWebRequest req = UnityWebRequest.Get(url))
        {
            req.timeout = downloadTimeoutSec;
            yield return req.SendWebRequest();
            if (req.result != UnityWebRequest.Result.Success)
            {
                failedUpdates++;
                Debug.LogWarning($"[PlyMapViewer] Manifest download failed: {req.error} url={url}");
                isPollingManifest = false;
                yield break;
            }

            string json = req.downloadHandler.text;
            if (TryParseUpdate(json, out PlyUpdate update))
            {
                if (update.version > displayedVersion && update.version > latestRequestedVersion)
                {
                    updateQueue.Enqueue(update);
                    if (verboseLogs) Debug.Log($"[PlyMapViewer] Queued manifest version={update.version} url={update.url}");
                }
            }
            else
            {
                failedUpdates++;
                Debug.LogWarning($"[PlyMapViewer] Manifest parse failed: {json}");
            }
        }
        isPollingManifest = false;
    }

    private void ReceivePlyUpdate(StringMsg msg)
    {
        if (!TryParseUpdate(msg.data, out PlyUpdate update))
        {
            Debug.LogWarning($"[PlyMapViewer] Ignoring malformed update: {msg.data}");
            return;
        }

        if (update.version <= displayedVersion || update.version <= latestRequestedVersion)
        {
            droppedUpdates++;
            return;
        }

        updateQueue.Enqueue(update);
    }

    void Update()
    {
        PlyUpdate? newestQueued = null;
        while (updateQueue.TryDequeue(out PlyUpdate update))
        {
            if (update.version <= displayedVersion || update.version <= latestRequestedVersion)
            {
                droppedUpdates++;
                continue;
            }

            if (!newestQueued.HasValue || update.version > newestQueued.Value.version)
            {
                newestQueued = update;
            }
            else
            {
                droppedUpdates++;
            }
        }

        if (newestQueued.HasValue)
        {
            if (isLoading)
            {
                if (!pendingLatest.HasValue || newestQueued.Value.version > pendingLatest.Value.version)
                {
                    pendingLatest = newestQueued.Value;
                }
                else
                {
                    droppedUpdates++;
                }
            }
            else
            {
                StartCoroutine(DownloadParseAndSwap(newestQueued.Value));
            }
        }

        if (!isLoading && pendingLatest.HasValue)
        {
            PlyUpdate next = pendingLatest.Value;
            pendingLatest = null;
            if (next.version > displayedVersion && next.version > latestRequestedVersion)
            {
                StartCoroutine(DownloadParseAndSwap(next));
            }
        }
    }

    private IEnumerator DownloadParseAndSwap(PlyUpdate update)
    {
        isLoading = true;
        latestRequestedVersion = update.version;
        string url = update.url;
        float downloadStart = Time.realtimeSinceStartup;

        if (verboseLogs)
        {
            Debug.Log($"[PlyMapViewer] Downloading map version={update.version} url={url}");
        }

        byte[] bytes = null;
        string downloadedFilePath = null;
        using (UnityWebRequest req = UnityWebRequest.Get(url))
        {
            req.timeout = downloadTimeoutSec;
            if (useDownloadHandlerFile)
            {
                string safeName = $"multi_robot_map_{update.version}.ply";
                downloadedFilePath = Path.Combine(Application.temporaryCachePath, safeName);
                req.downloadHandler = new DownloadHandlerFile(downloadedFilePath)
                {
                    removeFileOnAbort = true
                };
            }

            UnityWebRequestAsyncOperation op = req.SendWebRequest();
            ulong lastBytes = 0;
            float lastProgressTime = Time.realtimeSinceStartup;
            float lastLogTime = Time.realtimeSinceStartup;

            while (!op.isDone)
            {
                ulong currentBytes = req.downloadedBytes;
                float now = Time.realtimeSinceStartup;

                if (currentBytes != lastBytes)
                {
                    lastBytes = currentBytes;
                    lastProgressTime = now;
                }

                if (verboseLogs && now - lastLogTime >= downloadProgressLogIntervalSec)
                {
                    float mb = currentBytes / (1024f * 1024f);
                    Debug.Log($"[PlyMapViewer] Download progress version={update.version} {mb:F1}MB progress={req.downloadProgress:P1}");
                    lastLogTime = now;
                }

                if (absoluteDownloadTimeoutSec > 0f && now - downloadStart > absoluteDownloadTimeoutSec)
                {
                    req.Abort();
                    failedUpdates++;
                    isLoading = false;
                    Debug.LogError($"[PlyMapViewer] Download absolute timeout version={update.version} result={req.result} error={req.error} code={req.responseCode} bytes={currentBytes} elapsed={now - downloadStart:F2}s url={url}");
                    yield break;
                }

                if (downloadStallTimeoutSec > 0f && currentBytes > 0 && now - lastProgressTime > downloadStallTimeoutSec)
                {
                    req.Abort();
                    failedUpdates++;
                    isLoading = false;
                    Debug.LogError($"[PlyMapViewer] Download stalled version={update.version} bytes={currentBytes} url={url}");
                    yield break;
                }

                yield return null;
            }

            lastDownloadSec = Time.realtimeSinceStartup - downloadStart;
            if (req.result != UnityWebRequest.Result.Success)
            {
                failedUpdates++;
                isLoading = false;
                Debug.LogError($"[PlyMapViewer] Download failed version={update.version}: {req.error} bytes={req.downloadedBytes} url={url}");
                yield break;
            }

            if (!useDownloadHandlerFile)
            {
                bytes = req.downloadHandler.data;
            }
        }

        long downloadedBytes = useDownloadHandlerFile && !string.IsNullOrEmpty(downloadedFilePath) && File.Exists(downloadedFilePath)
            ? new FileInfo(downloadedFilePath).Length
            : (bytes != null ? bytes.Length : 0);
        if (downloadedBytes <= 0)
        {
            failedUpdates++;
            isLoading = false;
            Debug.LogError($"[PlyMapViewer] Download returned empty data version={update.version} url={url}");
            yield break;
        }

        if (verboseLogs)
        {
            float mb = downloadedBytes / (1024f * 1024f);
            Debug.Log($"[PlyMapViewer] Downloaded version={update.version} size={mb:F1}MB in {lastDownloadSec:F3}s path={(downloadedFilePath ?? "memory")}");
        }

        float parseStart = Time.realtimeSinceStartup;
        Task<ParsedPly> parseTask = useDownloadHandlerFile
            ? Task.Run(() => ParseBinaryPlyFile(downloadedFilePath, update.version, url, maxGlobalPoints))
            : Task.Run(() => ParseBinaryPly(bytes, update.version, url, maxGlobalPoints));
        while (!parseTask.IsCompleted) yield return null;

        lastParseSec = Time.realtimeSinceStartup - parseStart;
        if (parseTask.IsFaulted)
        {
            failedUpdates++;
            isLoading = false;
            Exception ex = parseTask.Exception != null ? parseTask.Exception.GetBaseException() : null;
            Debug.LogError($"[PlyMapViewer] Parse failed version={update.version}: {ex?.Message}");
            yield break;
        }

        ParsedPly parsed = parseTask.Result;
        if (verboseLogs) Debug.Log($"[PlyMapViewer] Parsed version={parsed.version} points={parsed.count:N0}; uploading GPU buffer");
        int writeIdx = backBufferIdx;
        posBuffers[writeIdx].SetData(parsed.positions, 0, 0, parsed.count);
        colBuffers[writeIdx].SetData(parsed.colors, 0, 0, parsed.count);
        bufferCounts[writeIdx] = parsed.count;

        int oldActive = activeBufferIdx;
        activeBufferIdx = writeIdx;
        backBufferIdx = oldActive;
        bufferCounts[backBufferIdx] = 0;

        displayedVersion = parsed.version;
        displayedUrl = parsed.url;
        downloadedUpdates++;
        isLoading = false;

        if (verboseLogs)
        {
            float mb = bytes.Length / (1024f * 1024f);
            Debug.Log($"[PlyMapViewer] Displayed version={parsed.version} points={parsed.count:N0}/{parsed.sourcePointCount:N0} size={mb:F1}MB download={lastDownloadSec:F3}s parse={lastParseSec:F3}s");
        }
    }


    private static ParsedPly ParseBinaryPlyFile(string path, int version, string url, int maxPoints)
    {
        byte[] bytes = File.ReadAllBytes(path);
        return ParseBinaryPly(bytes, version, url, maxPoints);
    }

    private static ParsedPly ParseBinaryPly(byte[] bytes, int version, string url, int maxPoints)
    {
        int headerEnd = FindHeaderEnd(bytes);
        if (headerEnd < 0) throw new InvalidOperationException("PLY header end not found");

        string header = System.Text.Encoding.ASCII.GetString(bytes, 0, headerEnd);
        if (!header.Contains("format binary_little_endian 1.0"))
        {
            throw new InvalidOperationException("Only binary_little_endian PLY is supported");
        }

        int vertexCount = ParseVertexCount(header);
        if (vertexCount <= 0) throw new InvalidOperationException($"Invalid PLY vertex count: {vertexCount}");

        int dataStart = headerEnd + "end_header\n".Length;
        int stride = 15; // float x,y,z + uchar r,g,b
        int available = (bytes.Length - dataStart) / stride;
        int sourceCount = Math.Min(vertexCount, available);
        int outCount = sourceCount;
        int sampleStride = 1;
        if (maxPoints > 0 && outCount > maxPoints)
        {
            sampleStride = Math.Max(1, sourceCount / maxPoints);
            outCount = Math.Min(maxPoints, (sourceCount + sampleStride - 1) / sampleStride);
        }

        Vector3[] positions = new Vector3[outCount];
        uint[] colors = new uint[outCount];
        int written = 0;
        for (int i = 0; i < sourceCount && written < outCount; i += sampleStride)
        {
            int b = dataStart + i * stride;
            float x = BitConverter.ToSingle(bytes, b);
            float y = BitConverter.ToSingle(bytes, b + 4);
            float z = BitConverter.ToSingle(bytes, b + 8);
            byte r = bytes[b + 12];
            byte g = bytes[b + 13];
            byte bl = bytes[b + 14];
            positions[written] = new Vector3(x, y, z);
            colors[written] = ((uint)r << 16) | ((uint)g << 8) | bl;
            written++;
        }

        if (written != outCount)
        {
            Array.Resize(ref positions, written);
            Array.Resize(ref colors, written);
            outCount = written;
        }

        return new ParsedPly
        {
            version = version,
            url = url,
            positions = positions,
            colors = colors,
            count = outCount,
            sourcePointCount = sourceCount,
        };
    }

    private static int FindHeaderEnd(byte[] bytes)
    {
        byte[] marker = System.Text.Encoding.ASCII.GetBytes("end_header\n");
        for (int i = 0; i <= bytes.Length - marker.Length; i++)
        {
            bool match = true;
            for (int j = 0; j < marker.Length; j++)
            {
                if (bytes[i + j] != marker[j])
                {
                    match = false;
                    break;
                }
            }
            if (match) return i;
        }
        return -1;
    }

    private static int ParseVertexCount(string header)
    {
        Match m = Regex.Match(header, @"element\s+vertex\s+(\d+)");
        if (!m.Success) return 0;
        return int.Parse(m.Groups[1].Value);
    }

    private bool TryParseUpdate(string json, out PlyUpdate update)
    {
        update = new PlyUpdate();
        int version = ExtractInt(json, "version", ExtractInt(json, "revision", -1));
        string url = ExtractString(json, preferLatestUrl ? "latest_url" : "url");
        if (string.IsNullOrWhiteSpace(url) && preferLatestUrl)
        {
            url = ExtractString(json, "url");
        }
        if (version < 0 || string.IsNullOrWhiteSpace(url)) return false;

        update.version = version;
        update.url = url;
        update.filename = ExtractString(json, "filename");
        update.pointCount = ExtractInt(json, "point_count", 0);
        update.fileSizeBytes = ExtractLong(json, "file_size_bytes", 0);
        return true;
    }

    private static string ExtractString(string json, string key)
    {
        Match m = Regex.Match(json, "\\\"" + Regex.Escape(key) + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
        return m.Success ? m.Groups[1].Value : "";
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
        DrawActiveBuffer();
    }

    private void DrawActiveBuffer()
    {
        if (pointMaterial == null) return;
        int activeCount = bufferCounts[activeBufferIdx];
        if (activeCount == 0) return;

        Matrix4x4 adjust = (useWorldAdjust && worldAdjust != null)
            ? worldAdjust.localToWorldMatrix
            : Matrix4x4.identity;

        pointMaterial.SetMatrix("_WorldAdjust", adjust);
        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", colorTint);
        pointMaterial.SetBuffer("_Positions", posBuffers[activeBufferIdx]);
        pointMaterial.SetBuffer("_Colors", colBuffers[activeBufferIdx]);
        pointMaterial.SetBuffer("_KeyframeIndices", dummyIdxBuffer);
        pointMaterial.SetBuffer("_KeyframePoses", dummyPoseBuffer);

        Graphics.DrawProcedural(
            pointMaterial,
            new Bounds(Vector3.zero, Vector3.one * 10000f),
            MeshTopology.Points,
            activeCount,
            1);
    }

    void OnGUI()
    {
        if (!showStats) return;
        GUI.color = Color.cyan;
        GUILayout.Label($"[PLY Map Viewer] FPS: {1.0f / Time.smoothDeltaTime:0}");
        GUILayout.Label($"Displayed Version: {displayedVersion}");
        GUILayout.Label($"Displayed Points: {bufferCounts[activeBufferIdx]:N0} / {maxGlobalPoints:N0}");
        GUILayout.Label($"Download: {lastDownloadSec:F3}s  Parse: {lastParseSec:F3}s  Loading: {isLoading}");
        GUILayout.Label($"Updates: {downloadedUpdates} shown, {droppedUpdates} skipped, {failedUpdates} failed");
        if (!string.IsNullOrEmpty(displayedUrl)) GUILayout.Label(displayedUrl);
    }

    void OnDestroy()
    {
        if (rosConnector != null && rosConnector.RosSocket != null && subId != null)
        {
            rosConnector.RosSocket.Unsubscribe(subId);
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
