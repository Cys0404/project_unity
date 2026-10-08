using System;
using OpenCvSharp;
using UnityEngine;
using UnityEngine.Rendering;

namespace VRRGBD
{
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public sealed class RgbdPointCloudRenderer : MonoBehaviour
    {
        [Header("Point Cloud")]
        [Range(1, 16)] public int downsample = 4;
        [Min(0.05f)] public float maxDepthMeters = 3.0f;
        [Min(0.001f)] public float pointSizeMeters = 0.008f;
        public PointCloudProjectionMode projectionMode = PointCloudProjectionMode.UrdfOpticalYForward;
        public bool visible = true;

        [Header("Runtime")]
        public string cameraId = "cam0";
        public int pointCount;
        public float latencyMs;

        private Mesh mesh;
        private MeshRenderer meshRenderer;
        private Material material;
        private Texture2D colorTexture;
        private Texture2D depthTexture;
        private int meshWidth;
        private int meshHeight;
        private int meshStep;

        private static readonly int ColorTexId = Shader.PropertyToID("_ColorTex");
        private static readonly int DepthTexId = Shader.PropertyToID("_DepthTex");
        private static readonly int IntrinsicsId = Shader.PropertyToID("_Intrinsics");
        private static readonly int ImageSizeId = Shader.PropertyToID("_ImageSize");
        private static readonly int DepthUnitId = Shader.PropertyToID("_DepthUnitMeters");
        private static readonly int MaxDepthId = Shader.PropertyToID("_MaxDepthMeters");
        private static readonly int PointSizeId = Shader.PropertyToID("_PointSizeMeters");
        private static readonly int ProjectionModeId = Shader.PropertyToID("_ProjectionMode");

        private void Awake()
        {
            meshRenderer = GetComponent<MeshRenderer>();
            var filter = GetComponent<MeshFilter>();
            mesh = new Mesh { name = "RGB-D Point Grid", indexFormat = IndexFormat.UInt32 };
            filter.sharedMesh = mesh;

            Shader shader = Shader.Find("VRRGBD/RGBD Point Cloud");
            material = new Material(shader);
            meshRenderer.sharedMaterial = material;
            meshRenderer.enabled = visible;
        }

        public void ApplyFrame(RgbdFrame frame)
        {
            if (frame == null || frame.header == null) return;
            if (!ValidateFrame(frame)) return;

            cameraId = frame.CameraId;
            EnsureTextures(frame.Width, frame.Height);

            colorTexture.LoadImage(frame.rgbJpeg, false);
            LoadDepthPng16(frame.depthPng, frame.Width, frame.Height);
            colorTexture.Apply(false, false);
            depthTexture.Apply(false, false);

            EnsurePointMesh(frame.Width, frame.Height, downsample);

            var k = frame.header.camera;
            material.SetTexture(ColorTexId, colorTexture);
            material.SetTexture(DepthTexId, depthTexture);
            material.SetVector(IntrinsicsId, new Vector4(k.fx, k.fy, k.cx, k.cy));
            material.SetVector(ImageSizeId, new Vector4(frame.Width, frame.Height, 1f / frame.Width, 1f / frame.Height));
            material.SetFloat(DepthUnitId, frame.DepthUnitMeters);
            material.SetFloat(MaxDepthId, maxDepthMeters);
            material.SetFloat(PointSizeId, pointSizeMeters);
            material.SetFloat(ProjectionModeId, projectionMode == PointCloudProjectionMode.UrdfOpticalYForward ? 1f : 0f);

            latencyMs = frame.header.send_time_unix > 0.0
                ? (float)((frame.receivedTimeUnix - frame.header.send_time_unix) * 1000.0)
                : 0f;
        }

        public void SetVisible(bool isVisible)
        {
            visible = isVisible;
            if (meshRenderer != null) meshRenderer.enabled = isVisible;
        }

        private bool ValidateFrame(RgbdFrame frame)
        {
            bool ok = frame.rgbJpeg != null && frame.rgbJpeg.Length > 0 &&
                      frame.depthPng != null && frame.depthPng.Length > 0 &&
                      frame.Width > 0 && frame.Height > 0 &&
                      frame.header.camera.fx > 0f && frame.header.camera.fy > 0f;
            if (!ok) Debug.LogWarning($"[RgbdPointCloudRenderer] Incomplete RGB-D frame for {frame.CameraId}");
            return ok;
        }

        private void EnsureTextures(int width, int height)
        {
            if (colorTexture == null || colorTexture.width != width || colorTexture.height != height)
            {
                colorTexture = new Texture2D(width, height, TextureFormat.RGB24, false, false)
                {
                    name = $"{cameraId}_rgb",
                    wrapMode = TextureWrapMode.Clamp,
                    filterMode = FilterMode.Bilinear
                };
            }

            if (depthTexture == null || depthTexture.width != width || depthTexture.height != height)
            {
                depthTexture = new Texture2D(width, height, TextureFormat.R16, false, true)
                {
                    name = $"{cameraId}_depth16",
                    wrapMode = TextureWrapMode.Clamp,
                    filterMode = FilterMode.Point
                };
            }
        }

        private void EnsurePointMesh(int width, int height, int step)
        {
            step = Mathf.Max(1, step);
            if (meshWidth == width && meshHeight == height && meshStep == step && mesh.vertexCount > 0) return;

            int cols = Mathf.CeilToInt(width / (float)step);
            int rows = Mathf.CeilToInt(height / (float)step);
            pointCount = cols * rows;
            if (pointCount > 1_000_000)
            {
                Debug.LogWarning($"[RgbdPointCloudRenderer] {cameraId} point grid is very large: {pointCount}");
            }

            var vertices = new Vector3[pointCount];
            var uvs = new Vector2[pointCount];
            var indices = new int[pointCount];
            int idx = 0;
            for (int y = 0; y < height; y += step)
            {
                for (int x = 0; x < width; x += step)
                {
                    vertices[idx] = Vector3.zero;
                    uvs[idx] = new Vector2((x + 0.5f) / width, 1f - ((y + 0.5f) / height));
                    indices[idx] = idx;
                    idx++;
                }
            }

            mesh.Clear();
            mesh.vertices = vertices;
            mesh.uv = uvs;
            mesh.SetIndices(indices, MeshTopology.Points, 0, false);
            mesh.bounds = new Bounds(Vector3.zero, Vector3.one * (maxDepthMeters * 2f));

            meshWidth = width;
            meshHeight = height;
            meshStep = step;
        }

        private void LoadDepthPng16(byte[] pngBytes, int width, int height)
        {
            using (Mat decoded = Cv2.ImDecode(pngBytes, ImreadModes.Unchanged))
            {
                if (decoded.Empty())
                {
                    Debug.LogWarning($"[RgbdPointCloudRenderer] Empty depth PNG for {cameraId}");
                    return;
                }

                using (Mat depth16 = new Mat())
                {
                    if (decoded.Type() == MatType.CV_16UC1)
                    {
                        decoded.CopyTo(depth16);
                    }
                    else
                    {
                        decoded.ConvertTo(depth16, MatType.CV_16UC1);
                    }

                    if (depth16.Width != width || depth16.Height != height)
                    {
                        Debug.LogWarning($"[RgbdPointCloudRenderer] Depth size mismatch for {cameraId}: {depth16.Width}x{depth16.Height}, expected {width}x{height}");
                        return;
                    }

                    int rowBytes = width * sizeof(ushort);
                    int byteCount = rowBytes * height;
                    byte[] topDownRaw = new byte[byteCount];
                    byte[] unityRaw = new byte[byteCount];
                    System.Runtime.InteropServices.Marshal.Copy(depth16.Data, topDownRaw, 0, byteCount);

                    for (int y = 0; y < height; y++)
                    {
                        Buffer.BlockCopy(topDownRaw, y * rowBytes, unityRaw, (height - 1 - y) * rowBytes, rowBytes);
                    }

                    depthTexture.LoadRawTextureData(unityRaw);
                }
            }
        }

        private void OnValidate()
        {
            downsample = Mathf.Clamp(downsample, 1, 16);
            maxDepthMeters = Mathf.Max(0.05f, maxDepthMeters);
            pointSizeMeters = Mathf.Max(0.001f, pointSizeMeters);
            if (meshRenderer != null) meshRenderer.enabled = visible;
        }

        private void OnDestroy()
        {
            if (mesh != null) Destroy(mesh);
            if (material != null) Destroy(material);
            if (colorTexture != null) Destroy(colorTexture);
            if (depthTexture != null) Destroy(depthTexture);
        }
    }
}
