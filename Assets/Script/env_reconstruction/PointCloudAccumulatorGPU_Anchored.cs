// =====================
//  PointCloudAccumulatorGPU (Anchored + WorldAdjust)
//  - AutoAnchor: �Ĥ@���Ʀ۰���w���I (T_anchor)
//  - ReAnchor(): �H�ɭ������I (�U���Ʈɥͮ�)
//  - UseWorldAdjust: ø�s�����W�i�կx�} (������ Transform)�A�����I+�s�I�@�_��
//  - ComputeBuffer + DrawProcedural�]�ʸU�I�^ + ��� Hash Map�]FIFO / LRU �^�O�^
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
public class PointCloudAccumulatorGPU_Anchored : MonoBehaviour
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

    [Header("Sensor -> Map (Anchor)"), Tooltip("�Y�}�ҡA�Ĥ@���Ʒ|����U�� sensorXform �������I T_anchor �éT�w�C")]
    public bool autoAnchor = true;

    [Tooltip("���I�O�_�w�إߡ]��Ū�^")]
    public bool anchored = false; // inspector �i��

    [Header("World Adjust (Draw-Time)")]
    [Tooltip("�ҥξ���i�ա]ø�s�����W�x�}�^�A�����I+�s�I�@�_��")]
    public bool useWorldAdjust = true;

    [Tooltip("�Ψӷ��@�@�ɷL�ժ�������F�Y�šA�� Identity�C")]
    public Transform worldAdjust; // ��ĳ���V���󱾦b�Y�ӪŪ��󩳤U�A�������w��

    [Header("Optional Sensor Offsets (pre-anchor)")]
    public Vector3 positionOffset = Vector3.zero;
    public Vector3 rotationEuler = Vector3.zero;
    public Vector3 scale = Vector3.one;

    [Header("Rendering")]
    public Material pointMaterial; // Hidden/PointCloudGPU �ΧA������ Shader
    public float pointPixelSize = 3f;
    [ColorUsage(false, true)] public Color colorTint = Color.white;

    [Header("Debug")]
    public bool showStats = true;
    public bool verbose = false;

    // ---- ROS ----
    private RosConnector rosConnector;
    private string subscriberId;

    // ---- Batches ----
    private struct Batch { public Vector3[] pos; public uint[] col; public int count; }
    private readonly ConcurrentQueue<Batch> batchQueue = new ConcurrentQueue<Batch>();

    // ---- Voxel map ----
    private readonly Dictionary<Vector3Int, int> voxelToIndex = new Dictionary<Vector3Int, int>(1 << 20);
    private readonly Queue<Vector3Int> fifoQueue = new Queue<Vector3Int>(1 << 20);
    private readonly LinkedList<Vector3Int> lruList = new LinkedList<Vector3Int>();
    private readonly Dictionary<Vector3Int, LinkedListNode<Vector3Int>> lruNodeMap = new Dictionary<Vector3Int, LinkedListNode<Vector3Int>>(1 << 20);

    // ---- CPU storage ----
    private readonly List<Vector3> positions = new List<Vector3>(1024);   // "map space" = �w�M T_anchor ���y��
    private readonly List<uint> colors = new List<uint>(1024);

    // ---- GPU buffers ----
    private ComputeBuffer posBuffer;   // float3
    private ComputeBuffer colBuffer;   // uint (0xRRGGBB)
    private int bufferCapacity = 0;
    private bool gpuDirty = false;

    // ---- Derived / temps ----
    private int maxPointsCapacity;
    private Matrix4x4 sensorXform = Matrix4x4.identity;   // �� positionOffset/rotationEuler/scale �զ�
    private Matrix4x4 T_anchor = Matrix4x4.identity;      // �Ĥ@����Ʈ���w

    private Vector3[] tmpPositions = new Vector3[8192];
    private uint[] tmpColors = new uint[8192];

    void Awake()
    {
        rosConnector = GetComponent<RosConnector>();

        // ���է� Shader�]��ةR�W���ա^
        if (pointMaterial == null)
        {
            var sh = Shader.Find("Hidden/PointCloudGPU");
            if (sh == null) sh = Shader.Find("Hidden/Point Cloud GPU");
            if (sh != null) pointMaterial = new Material(sh);
        }

        RebuildSensorXform();
        //maxPointsCapacity = Mathf.Max(1, (memoryBudgetMB * 1024 * 1024) / 16);
        maxPointsCapacity = Mathf.Max(1, (int)((long)memoryBudgetMB * 1024 * 1024 / 16));
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
        try { subscriberId = rosConnector.RosSocket.Subscribe<PointCloud2>(topicName, OnPointCloudReceived); }
        catch (Exception e) { Debug.LogError($"[PC-Anchor] Subscribe failed: {e.Message}"); }
    }

    void Update()
    {
        // consume batches (cap)
        int processed = 0;
        while (batchQueue.TryDequeue(out var b))
        {
            AppendBatch(b);
            processed += b.count;
            if (processed >= maxProcessPerFrame) break;
        }

        if (gpuDirty) UploadToGPU();

        if (showStats && Time.frameCount % 120 == 0)
        {
            Debug.Log($"[PC-Anchor] Points:{positions.Count}/{maxPointsCapacity} Voxels:{voxelToIndex.Count} Anchored:{anchored} UseWorldAdjust:{useWorldAdjust}");
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

    // ---------------- ROS callback (background) ----------------
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

                // ����P���I��� map �Ŷ��G
                // �Ĥ@�������ƥB autoAnchor�G��w T_anchor = sensorXform
                var A = (anchored || !autoAnchor) ? T_anchor : sensorXform;
                if (!anchored && autoAnchor) { T_anchor = sensorXform; anchored = true; A = T_anchor; }

                Vector3 p_map = A.MultiplyPoint3x4(new Vector3(x, y, z));
                tmpPositions[valid] = p_map;
                tmpColors[valid] = (rgb & 0x00FFFFFFu); // �� RRGGBB
                valid++;
            }
            if (valid <= 0) return;

            var pos = new Vector3[valid]; Array.Copy(tmpPositions, pos, valid);
            var col = new uint[valid]; Array.Copy(tmpColors, col, valid);
            batchQueue.Enqueue(new Batch { pos = pos, col = col, count = valid });
        }
        catch { }
    }

    // ---------------- Main-thread accumulation ----------------
    void AppendBatch(Batch b)
    {
        float inv = 1f / voxelSize;
        for (int i = 0; i < b.count; i++)
        {
            Vector3 p = b.pos[i];
            int vx = Mathf.FloorToInt(p.x * inv);
            int vy = Mathf.FloorToInt(p.y * inv);
            int vz = Mathf.FloorToInt(p.z * inv);
            var key = new Vector3Int(vx, vy, vz);

            if (voxelToIndex.TryGetValue(key, out int idxExisting))
            {
                // LRU touch
                if (eviction == EvictionPolicy.LRU && lruNodeMap.TryGetValue(key, out var node))
                { lruList.Remove(node); lruList.AddFirst(node); }
                continue;
            }

            if (positions.Count >= maxPointsCapacity) EvictOne();

            int newIdx = positions.Count;
            positions.Add(p);
            colors.Add(b.col[i]);
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
            // LRU �M�g����
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
        if (positions.Count == 0 || pointMaterial == null) return;

        // �]�w�@�ɷL�կx�}�]�w�] Identity�^
        Matrix4x4 W = Matrix4x4.identity;
        if (useWorldAdjust && worldAdjust != null) W = worldAdjust.localToWorldMatrix;

        pointMaterial.SetBuffer("_Positions", posBuffer);
        pointMaterial.SetBuffer("_Colors", colBuffer);
        pointMaterial.SetFloat("_PointSize", pointPixelSize);
        pointMaterial.SetColor("_Tint", colorTint);
        pointMaterial.SetMatrix("_WorldAdjust", W);

        pointMaterial.SetPass(0);
        Graphics.DrawProceduralNow(MeshTopology.Points, positions.Count);
    }

    // ---------------- Utilities ----------------
    [ContextMenu("Clear All")]
    public void ClearAll()
    {
        positions.Clear(); colors.Clear();
        voxelToIndex.Clear(); fifoQueue.Clear(); lruList.Clear(); lruNodeMap.Clear();
        gpuDirty = true;
    }

    [ContextMenu("Re-Anchor (next batch)")]
    public void ReAnchor()
    {
        anchored = false; // �U�@���Ʒ|����U sensorXform ��i T_anchor
    }
}

// ===================== Shader: PointCloudGPU (�s�W _WorldAdjust) =====================
// �N�A�{���� PointCloudGPU.shader ���e�H�����򩳡]�Υu�[�J _WorldAdjust �P���k�^
/*
Shader "Hidden/PointCloudGPU"
{
    Properties { _PointSize ("Point Size (px)", Float) = 3.0  _Tint ("Tint", Color) = (1,1,1,1) }
    SubShader {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        Pass {
            ZWrite On  Cull Off
            CGPROGRAM
            #pragma target 4.0
            #pragma vertex   vert
            #pragma geometry geom
            #pragma fragment frag
            #include "UnityCG.cginc"

            StructuredBuffer<float3> _Positions;
            StructuredBuffer<uint>   _Colors;   // 0xRRGGBB
            float4x4 _WorldAdjust;             // << �s�W�G�@�ɷL�կx�}
            float    _PointSize; fixed4 _Tint;

            struct v2g { float4 clipPos : SV_POSITION; fixed4 col : COLOR; };
            struct g2f { float4 clipPos : SV_POSITION; fixed4 col : COLOR; };

            fixed4 DecodeColor(uint rgb){
                float r = ((rgb >> 16) & 255) / 255.0;
                float g = ((rgb >> 8)  & 255) / 255.0;
                float b = ((rgb >> 0)  & 255) / 255.0;
                return fixed4(r,g,b,1);
            }

            v2g vert(uint id : SV_VertexID)
            {
                v2g o;
                float3 p_map = _Positions[id];
                // map -> world (�i��) -> clip
                float4 p_world = mul(_WorldAdjust, float4(p_map,1));
                o.clipPos = mul(UNITY_MATRIX_VP, p_world);
                o.col = DecodeColor(_Colors[id]) * _Tint;
                return o;
            }

            [maxvertexcount(4)]
            void geom(point v2g IN[1], inout TriangleStream<g2f> triStream)
            {
                float4 pos = IN[0].clipPos;
                float2 ndcPerPixel = float2(2.0/_ScreenParams.x, 2.0/_ScreenParams.y);
                float2 dx = float2(_PointSize,0) * ndcPerPixel;
                float2 dy = float2(0,_PointSize) * ndcPerPixel;
                g2f o; o.col = IN[0].col;
                o.clipPos = pos + float4(-dx - dy,0,0); triStream.Append(o);
                o.clipPos = pos + float4( dx - dy,0,0); triStream.Append(o);
                o.clipPos = pos + float4( dx + dy,0,0); triStream.Append(o);
                o.clipPos = pos + float4(-dx + dy,0,0); triStream.Append(o);
                triStream.RestartStrip();
            }
            fixed4 frag(g2f i):SV_Target{ return i.col; }
            ENDCG
        }
    }
}
*/