using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.Rendering;
#if UNITY_EDITOR
using UnityEditor;
#endif

using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

[RequireComponent(typeof(RosConnector))]
public class SingleRobotFramePointCloudViewer : MonoBehaviour
{
    [Header("ROS")]
    public string pointcloudTopic = "/robot1/mast3r/frame_pointcloud";

    [Header("Point Storage")]
    public bool accumulateFrames = true;
    public int maxPoints = 4_000_000;
    public int pointStride = 1;

    [Header("World Adjust")]
    public bool useWorldAdjust = false;
    public Transform worldAdjust;

    [Header("Rendering")]
    [HideInInspector] public Material pointMaterial;
    public float pointPixelSize = 3f;
    public float drawBoundsSize = 10000f;
    public List<Camera> extraRenderCameras = new List<Camera>();

    [Header("Debug")]
    public bool showStats = true;
    public bool showInSceneView = true;

    private struct CloudBatch
    {
        public Vector3[] positions;
        public uint[] colors;
        public int count;
        public string frameId;
    }

    private RosConnector rosConnector;
    private string subscriptionId;

    // RosSharp may invoke callbacks outside Unity's main thread. Keep only the
    // latest decoded cloud and upload it to the GPU from Update().
    private readonly object pendingLock = new object();
    private CloudBatch pendingBatch;
    private bool hasPendingBatch;

    private ComputeBuffer positionBuffer;
    private ComputeBuffer colorBuffer;
    private ComputeBuffer indexBuffer;
    private ComputeBuffer poseBuffer;
    private bool ownsMaterial;
    private int pointCount;
    private int receivedClouds;
    private int displayedClouds;
    private int replacedPendingClouds;
    private int rejectedClouds;
    private string lastFrameId = "";
    private bool capacityWarningLogged;
    private bool stopped;

    void Awake()
    {
        rosConnector = GetComponent<RosConnector>();
        maxPoints = Mathf.Max(1, maxPoints);

        if (pointMaterial == null)
        {
            Shader shader = Shader.Find("Hidden/PointCloudGPU");
            if (shader == null) shader = Shader.Find("Hidden/Point Cloud GPU");
            if (shader != null)
            {
                pointMaterial = new Material(shader);
                ownsMaterial = true;
            }
        }

        positionBuffer = new ComputeBuffer(maxPoints, 12);
        colorBuffer = new ComputeBuffer(maxPoints, 4);
        indexBuffer = new ComputeBuffer(maxPoints, 4);
        indexBuffer.SetData(new uint[maxPoints]);
        poseBuffer = new ComputeBuffer(1, 64);
        poseBuffer.SetData(new Matrix4x4[] { Matrix4x4.identity });
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

    void Start()
    {
        StartCoroutine(SubscribeWhenReady());
    }

    private IEnumerator SubscribeWhenReady()
    {
        while (!stopped && (rosConnector == null || rosConnector.RosSocket == null))
        {
            yield return null;
        }
        if (stopped) yield break;

        subscriptionId = rosConnector.RosSocket.Subscribe<PointCloud2>(pointcloudTopic, OnPointCloudReceived);
        Debug.Log($"[SingleFrameCloud] Subscribed to {pointcloudTopic}");
    }

    private void OnPointCloudReceived(PointCloud2 msg)
    {
        if (stopped) return;
        Interlocked.Increment(ref receivedClouds);

        if (!TryDecode(msg, out CloudBatch decoded))
        {
            int rejected = Interlocked.Increment(ref rejectedClouds);
            if (rejected <= 3)
            {
                Debug.LogWarning("[SingleFrameCloud] Rejected PointCloud2: empty data, missing xyz fields, or unsupported layout.");
            }
            return;
        }

        lock (pendingLock)
        {
            if (hasPendingBatch) replacedPendingClouds++;
            pendingBatch = decoded;
            hasPendingBatch = true;
        }
    }

    private bool TryDecode(PointCloud2 msg, out CloudBatch batch)
    {
        batch = new CloudBatch();
        if (msg == null || msg.data == null || msg.data.Length == 0) return false;
        if (msg.fields == null || msg.fields.Length == 0 || msg.is_bigendian) return false;

        int totalPoints = (int)(msg.width * msg.height);
        int pointStep = (int)msg.point_step;
        if (totalPoints <= 0 || pointStep <= 0) return false;

        int xOffset = -1;
        int yOffset = -1;
        int zOffset = -1;
        int rgbOffset = -1;
        foreach (PointField field in msg.fields)
        {
            if (field.name == "x") xOffset = (int)field.offset;
            else if (field.name == "y") yOffset = (int)field.offset;
            else if (field.name == "z") zOffset = (int)field.offset;
            else if (field.name == "rgb" || field.name == "rgba") rgbOffset = (int)field.offset;
        }
        if (xOffset < 0 || yOffset < 0 || zOffset < 0) return false;

        int stride = Math.Max(1, pointStride);
        int capacity = Math.Min((totalPoints + stride - 1) / stride, maxPoints);
        Vector3[] positions = new Vector3[capacity];
        uint[] colors = new uint[capacity];
        int written = 0;

        for (int pointIndex = 0; pointIndex < totalPoints && written < capacity; pointIndex += stride)
        {
            int offset = pointIndex * pointStep;
            if (offset < 0 || offset + pointStep > msg.data.Length) break;

            float x = BitConverter.ToSingle(msg.data, offset + xOffset);
            float y = BitConverter.ToSingle(msg.data, offset + yOffset);
            float z = BitConverter.ToSingle(msg.data, offset + zOffset);
            if (!IsFinite(x) || !IsFinite(y) || !IsFinite(z)) continue;

            positions[written] = new Vector3(x, y, z);
            colors[written] = rgbOffset >= 0 && offset + rgbOffset + 4 <= msg.data.Length
                ? BitConverter.ToUInt32(msg.data, offset + rgbOffset) & 0x00FFFFFFu
                : 0x00FFFFFFu;
            written++;
        }

        if (written <= 0) return false;
        if (written != capacity)
        {
            Array.Resize(ref positions, written);
            Array.Resize(ref colors, written);
        }

        batch = new CloudBatch
        {
            positions = positions,
            colors = colors,
            count = written,
            frameId = msg.header != null ? msg.header.frame_id : "",
        };
        return true;
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    void Update()
    {
        CloudBatch batch;
        lock (pendingLock)
        {
            if (!hasPendingBatch) return;
            batch = pendingBatch;
            hasPendingBatch = false;
        }

        int destinationOffset = accumulateFrames ? pointCount : 0;
        int uploadCount = Mathf.Min(batch.count, maxPoints - destinationOffset);
        if (uploadCount <= 0)
        {
            if (!capacityWarningLogged)
            {
                capacityWarningLogged = true;
                Debug.LogWarning("[SingleFrameCloud] GPU buffer is full. Increase Max Points, increase Point Stride, or disable Accumulate Frames.");
            }
            return;
        }

        positionBuffer.SetData(batch.positions, 0, destinationOffset, uploadCount);
        colorBuffer.SetData(batch.colors, 0, destinationOffset, uploadCount);
        pointCount = accumulateFrames ? destinationOffset + uploadCount : uploadCount;
        displayedClouds++;
        lastFrameId = batch.frameId;
    }

    void OnRenderObject()
    {
        Camera camera = Camera.current;
        if (IsExtraRenderCamera(camera)) return;
        DrawPointCloud();
    }

    private void OnCameraPostRender(Camera camera)
    {
        if (IsExtraRenderCamera(camera)) DrawPointCloud();
    }

    private bool IsExtraRenderCamera(Camera camera)
    {
        if (camera == null || extraRenderCameras == null) return false;
        for (int i = 0; i < extraRenderCameras.Count; i++)
        {
            if (extraRenderCameras[i] == camera) return true;
        }
        return false;
    }

#if UNITY_EDITOR
    private void OnSceneViewGui(SceneView sceneView)
    {
        if (!showInSceneView || !Application.isPlaying) return;
        if (Event.current == null || Event.current.type != EventType.Repaint) return;
        DrawPointCloud();
        sceneView.Repaint();
    }
#endif

    private void DrawPointCloud()
    {
        if (pointMaterial == null || pointCount <= 0) return;

        Matrix4x4 adjust = useWorldAdjust && worldAdjust != null
            ? worldAdjust.localToWorldMatrix
            : Matrix4x4.identity;
        pointMaterial.SetMatrix("_WorldAdjust", adjust);
        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", Color.white);
        pointMaterial.SetBuffer("_Positions", positionBuffer);
        pointMaterial.SetBuffer("_Colors", colorBuffer);
        pointMaterial.SetBuffer("_KeyframeIndices", indexBuffer);
        pointMaterial.SetBuffer("_KeyframePoses", poseBuffer);

        Graphics.DrawProcedural(
            pointMaterial,
            new Bounds(Vector3.zero, Vector3.one * Mathf.Max(1f, drawBoundsSize)),
            MeshTopology.Points,
            pointCount,
            1);
    }

    void OnGUI()
    {
        if (!showStats) return;
        GUI.color = Color.cyan;
        GUILayout.Label("[Single Robot Frame PointCloud]");
        GUILayout.Label($"received={receivedClouds} displayed={displayedClouds} rejected={rejectedClouds}");
        GUILayout.Label($"points={pointCount:N0}/{maxPoints:N0} replaced_pending={replacedPendingClouds}");
        GUILayout.Label($"frame_id={lastFrameId}");
    }

    public void ClearAndStop()
    {
        if (stopped) return;
        stopped = true;
        StopAllCoroutines();
        Unsubscribe();
        lock (pendingLock)
        {
            hasPendingBatch = false;
            pendingBatch = default(CloudBatch);
        }
        pointCount = 0;
        ReleaseBuffers();
        enabled = false;
        Debug.Log($"[SingleFrameCloud] Cleared and stopped {pointcloudTopic}");
    }

    private void Unsubscribe()
    {
        if (rosConnector != null && rosConnector.RosSocket != null && !string.IsNullOrEmpty(subscriptionId))
        {
            rosConnector.RosSocket.Unsubscribe(subscriptionId);
            subscriptionId = null;
        }
    }

    private void ReleaseBuffers()
    {
        positionBuffer?.Release();
        colorBuffer?.Release();
        indexBuffer?.Release();
        poseBuffer?.Release();
        positionBuffer = null;
        colorBuffer = null;
        indexBuffer = null;
        poseBuffer = null;
        if (ownsMaterial && pointMaterial != null)
        {
            Destroy(pointMaterial);
            pointMaterial = null;
            ownsMaterial = false;
        }
    }

    void OnDestroy()
    {
        stopped = true;
        Unsubscribe();
        ReleaseBuffers();
    }
}
