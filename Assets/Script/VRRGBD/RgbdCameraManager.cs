using System.Collections.Generic;
using UnityEngine;

namespace VRRGBD
{
    public sealed class RgbdCameraManager : MonoBehaviour
    {
        [Header("References")]
        public RgbdTcpReceiver receiver;
        [HideInInspector]
        public Transform referenceOpticalFrame;
        [HideInInspector]
        public Transform cameraRoot;

        [HideInInspector]
        public CameraOpticalFrameBinding[] opticalFrameBindings =
        {
            new CameraOpticalFrameBinding { cameraId = "cam0" },
            new CameraOpticalFrameBinding { cameraId = "cam1" },
            new CameraOpticalFrameBinding { cameraId = "cam2" }
        };

        [HideInInspector]
        public string referenceCameraId = "cam0";
        [HideInInspector]
        public string[] cameraIds = { "cam0", "cam1", "cam2" };

        [HideInInspector]
        [Range(1, 16)] public int defaultDownsample = 4;
        [HideInInspector]
        public float defaultMaxDepthMeters = 3f;
        [HideInInspector]
        public float defaultPointSizeMeters = 0.008f;
        [HideInInspector]
        public PointCloudProjectionMode defaultProjectionMode = PointCloudProjectionMode.UrdfOpticalYForward;

        [HideInInspector]
        public ExtrinsicsApplyMode extrinsicsApplyMode = ExtrinsicsApplyMode.Manual;

        [Header("Runtime Debug")]
        public string pointCloudFps;

        private readonly Dictionary<string, RgbdPointCloudRenderer> renderers = new Dictionary<string, RgbdPointCloudRenderer>();
        private readonly Dictionary<string, Transform> frames = new Dictionary<string, Transform>();
        private readonly Dictionary<string, Transform> alignmentRoots = new Dictionary<string, Transform>();
        private readonly Dictionary<string, Matrix4x4> manualExtrinsics = new Dictionary<string, Matrix4x4>();
        private readonly Dictionary<string, CameraRuntimeStats> stats = new Dictionary<string, CameraRuntimeStats>();
        private readonly Dictionary<string, CameraOpticalFrameBinding> bindingsById = new Dictionary<string, CameraOpticalFrameBinding>();

        public IReadOnlyDictionary<string, CameraRuntimeStats> Stats => stats;
        public int PendingManualExtrinsicsCount => manualExtrinsics.Count;
        public int PendingManualExtrinsicsCountForCameras(params string[] cameraIds)
        {
            int count = 0;
            if (cameraIds == null) return count;
            foreach (string cameraId in cameraIds)
                if (!string.IsNullOrWhiteSpace(cameraId) && manualExtrinsics.ContainsKey(cameraId)) count++;
            return count;
        }

        public void Configure(
            RgbdTcpReceiver newReceiver,
            string newReferenceCameraId,
            string[] newCameraIds,
            Transform newReferenceOpticalFrame,
            CameraOpticalFrameBinding[] newOpticalFrameBindings,
            int newDownsample,
            float newMaxDepthMeters,
            float newPointSizeMeters,
            PointCloudProjectionMode newProjectionMode,
            ExtrinsicsApplyMode newExtrinsicsApplyMode)
        {
            receiver = newReceiver;
            referenceCameraId = string.IsNullOrWhiteSpace(newReferenceCameraId) ? "cam0" : newReferenceCameraId.Trim();
            cameraIds = newCameraIds;
            referenceOpticalFrame = newReferenceOpticalFrame;
            if (referenceOpticalFrame != null) cameraRoot = referenceOpticalFrame;
            opticalFrameBindings = newOpticalFrameBindings;
            defaultDownsample = Mathf.Clamp(newDownsample, 1, 16);
            defaultMaxDepthMeters = Mathf.Max(0.05f, newMaxDepthMeters);
            defaultPointSizeMeters = Mathf.Max(0.001f, newPointSizeMeters);
            defaultProjectionMode = newProjectionMode;
            extrinsicsApplyMode = newExtrinsicsApplyMode;

            RebuildBindingLookup();

            if (Application.isPlaying)
            {
                EnsureConfiguredCameras();
                ReapplyOpticalFrameBindings();
                ApplyRenderingDefaultsToAll();
            }
        }

        private void Awake()
        {
            if (receiver == null) receiver = FindObjectOfType<RgbdTcpReceiver>();
            RebuildBindingLookup();

            if (referenceOpticalFrame == null)
            {
                CameraOpticalFrameBinding referenceBinding = GetBinding(referenceCameraId);
                if (referenceBinding != null && referenceBinding.opticalFrame != null)
                    referenceOpticalFrame = referenceBinding.opticalFrame;
            }

            if (referenceOpticalFrame != null && (cameraRoot == null || cameraRoot.name == "RGBD Camera Frames"))
            {
                cameraRoot = referenceOpticalFrame;
            }

            if (cameraRoot == null)
            {
                var root = new GameObject("RGBD Camera Frames");
                root.transform.SetParent(transform, false);
                cameraRoot = root.transform;
            }

            EnsureConfiguredCameras();
            SetCameraTransform(referenceCameraId, Matrix4x4.identity, true);
        }

        private void Update()
        {
            if (receiver != null)
            {
                foreach (string id in new List<string>(receiver.LatestCameraIds))
                {
                    if (receiver.TryConsumeLatest(id, out RgbdFrame frame))
                    {
                        ApplyFrame(frame);
                    }
                }
            }
        }

        public RgbdPointCloudRenderer EnsureCamera(string cameraId)
        {
            if (string.IsNullOrWhiteSpace(cameraId)) cameraId = referenceCameraId;
            if (renderers.TryGetValue(cameraId, out var existing)) return existing;

            Transform frame = GetOpticalFrameForCamera(cameraId);
            if (frame == null)
            {
                var frameObject = new GameObject($"{cameraId}_CameraFrame");
                frameObject.transform.SetParent(GetParentForCamera(cameraId), false);
                frameObject.transform.localPosition = Vector3.zero;
                frameObject.transform.localRotation = Quaternion.identity;
                frameObject.transform.localScale = Vector3.one;
                frame = frameObject.transform;
            }

            frames[cameraId] = frame;
            alignmentRoots[cameraId] = GetAlignmentRootForCamera(cameraId, frame);

            var cloudObject = new GameObject($"{cameraId}_PointCloud");
            cloudObject.transform.SetParent(frame, false);
            cloudObject.transform.localPosition = Vector3.zero;
            cloudObject.transform.localRotation = Quaternion.identity;
            cloudObject.transform.localScale = Vector3.one;
            var renderer = cloudObject.AddComponent<RgbdPointCloudRenderer>();
            renderer.cameraId = cameraId;
            renderer.downsample = defaultDownsample;
            renderer.maxDepthMeters = defaultMaxDepthMeters;
            renderer.pointSizeMeters = defaultPointSizeMeters;
            renderer.projectionMode = defaultProjectionMode;
            renderers[cameraId] = renderer;

            stats[cameraId] = new CameraRuntimeStats { cameraId = cameraId, aligned = cameraId == referenceCameraId };
            UpdatePointCloudFpsSummary();
            return renderer;
        }

        public void ApplyFrame(RgbdFrame frame)
        {
            var renderer = EnsureCamera(frame.CameraId);
            try
            {
                renderer.ApplyFrame(frame);
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[RgbdCameraManager] Failed to apply {frame.CameraId} frame: {e.Message}");
                return;
            }

            var s = stats[frame.CameraId];
            double now = Time.realtimeSinceStartupAsDouble;
            if (s.lastFrameUnix > 0.0)
            {
                double dt = frame.receivedTimeUnix - s.lastFrameUnix;
                if (dt > 0.0001) s.streamFps = Mathf.Lerp(s.streamFps, (float)(1.0 / dt), 0.15f);
            }
            s.lastFrameUnix = frame.receivedTimeUnix;
            s.lastSendUnix = frame.header.send_time_unix;
            s.receivedFrames++;
            s.connected = receiver != null && receiver.IsConnected;
            s.pointCount = renderer.pointCount;
            s.latencyMs = renderer.latencyMs;
            UpdatePointCloudFpsSummary();
        }

        public void SetCameraTransform(string cameraId, Matrix4x4 cameraToReferenceRos, bool alreadyUnityMatrix = false)
        {
            EnsureCamera(cameraId);

            Matrix4x4 unityMatrix = alreadyUnityMatrix
                ? cameraToReferenceRos
                : RosUnityCoordinate.OpticalTransformToUnity(cameraToReferenceRos, defaultProjectionMode);

            if (cameraId == referenceCameraId)
            {
                if (IsGeneratedFrame(cameraId))
                {
                    frames[cameraId].SetParent(GetReferenceParent(), false);
                    frames[cameraId].localPosition = Vector3.zero;
                    frames[cameraId].localRotation = Quaternion.identity;
                    frames[cameraId].localScale = Vector3.one;
                }
            }
            else
            {
                if (extrinsicsApplyMode == ExtrinsicsApplyMode.Manual)
                {
                    manualExtrinsics[cameraId] = unityMatrix;
                    stats[cameraId].aligned = false;
                    return;
                }

                ApplyCameraTransform(cameraId, unityMatrix);
            }

            stats[cameraId].aligned = true;
        }

        [ContextMenu("Apply Pending Manual Extrinsics")]
        public void ApplyPendingManualExtrinsics()
        {
            foreach (var kvp in new Dictionary<string, Matrix4x4>(manualExtrinsics))
            {
                EnsureCamera(kvp.Key);
                ApplyCameraTransform(kvp.Key, kvp.Value);
                if (stats.TryGetValue(kvp.Key, out var s)) s.aligned = true;
            }

            manualExtrinsics.Clear();
        }

        public int ApplyPendingManualExtrinsicsForCameras(params string[] cameraIds)
        {
            int applied = 0;
            if (cameraIds == null) return applied;
            foreach (string cameraId in cameraIds)
            {
                if (string.IsNullOrWhiteSpace(cameraId) || cameraId == referenceCameraId ||
                    !manualExtrinsics.TryGetValue(cameraId, out Matrix4x4 matrix)) continue;
                EnsureCamera(cameraId);
                ApplyCameraTransform(cameraId, matrix);
                if (stats.TryGetValue(cameraId, out var cameraStats)) cameraStats.aligned = true;
                manualExtrinsics.Remove(cameraId);
                applied++;
            }
            return applied;
        }

        private void UpdatePointCloudFpsSummary()
        {
            pointCloudFps = string.Empty;
            foreach (var kvp in stats)
            {
                if (pointCloudFps.Length > 0) pointCloudFps += "  ";
                pointCloudFps += $"{kvp.Key}:{kvp.Value.streamFps:F1}Hz";
            }
        }

        public void SetCameraVisible(string cameraId, bool visible)
        {
            EnsureCamera(cameraId).SetVisible(visible);
        }

        public void SetDownsample(string cameraId, int downsample)
        {
            EnsureCamera(cameraId).downsample = Mathf.Clamp(downsample, 1, 16);
        }

        public void EnsureConfiguredCameras()
        {
            RebuildBindingLookup();
            foreach (string id in GetConfiguredCameraIds())
            {
                EnsureCamera(id);
            }
        }

        public void ApplyRenderingDefaultsToAll()
        {
            defaultDownsample = Mathf.Clamp(defaultDownsample, 1, 16);
            defaultMaxDepthMeters = Mathf.Max(0.05f, defaultMaxDepthMeters);
            defaultPointSizeMeters = Mathf.Max(0.001f, defaultPointSizeMeters);

            foreach (var kvp in renderers)
            {
                kvp.Value.downsample = defaultDownsample;
                kvp.Value.maxDepthMeters = defaultMaxDepthMeters;
                kvp.Value.pointSizeMeters = defaultPointSizeMeters;
                kvp.Value.projectionMode = defaultProjectionMode;
            }
        }

        private void OnValidate()
        {
            if (!Application.isPlaying) return;
            ReapplyOpticalFrameBindings();
            ApplyRenderingDefaultsToAll();
        }

        public void ReapplyOpticalFrameBindings()
        {
            RebuildBindingLookup();
            foreach (string id in new List<string>(frames.Keys))
            {
                Transform opticalFrame = GetOpticalFrameForCamera(id);
                if (opticalFrame != null)
                {
                    frames[id] = opticalFrame;
                    alignmentRoots[id] = GetAlignmentRootForCamera(id, opticalFrame);
                    ReparentPointCloudToFrame(id, opticalFrame);
                    continue;
                }

                if (id == referenceCameraId)
                {
                    frames[id].SetParent(GetReferenceParent(), false);
                    frames[id].localPosition = Vector3.zero;
                    frames[id].localRotation = Quaternion.identity;
                    frames[id].localScale = Vector3.one;
                }
                else
                {
                    frames[id].SetParent(GetInitialParentForCamera(id), false);
                    frames[id].localPosition = Vector3.zero;
                    frames[id].localRotation = Quaternion.identity;
                    frames[id].localScale = Vector3.one;
                }
            }
        }

        private void RebuildBindingLookup()
        {
            bindingsById.Clear();
            if (opticalFrameBindings == null) return;
            foreach (var binding in opticalFrameBindings)
            {
                if (binding == null || string.IsNullOrWhiteSpace(binding.cameraId)) continue;
                bindingsById[binding.cameraId.Trim()] = binding;
            }
        }

        private IEnumerable<string> GetConfiguredCameraIds()
        {
            var ids = new HashSet<string>();

            if (!string.IsNullOrWhiteSpace(referenceCameraId))
                ids.Add(referenceCameraId.Trim());

            if (cameraIds != null)
            {
                foreach (string id in cameraIds)
                {
                    if (!string.IsNullOrWhiteSpace(id))
                        ids.Add(id.Trim());
                }
            }

            if (opticalFrameBindings != null)
            {
                foreach (var binding in opticalFrameBindings)
                {
                    if (binding != null && !string.IsNullOrWhiteSpace(binding.cameraId))
                        ids.Add(binding.cameraId.Trim());
                }
            }

            return ids;
        }

        private CameraOpticalFrameBinding GetBinding(string cameraId)
        {
            if (bindingsById.Count == 0) RebuildBindingLookup();
            bindingsById.TryGetValue(cameraId, out var binding);
            return binding;
        }

        private Transform GetParentForCamera(string cameraId)
        {
            return cameraId == referenceCameraId ? GetReferenceParent() : GetInitialParentForCamera(cameraId);
        }

        private Transform GetOpticalFrameForCamera(string cameraId)
        {
            CameraOpticalFrameBinding binding = GetBinding(cameraId);
            if (binding != null && binding.lockPointCloudToOpticalFrame && binding.opticalFrame != null)
                return binding.opticalFrame;

            return null;
        }

        private Transform GetAlignmentRootForCamera(string cameraId, Transform fallback)
        {
            CameraOpticalFrameBinding binding = GetBinding(cameraId);
            if (binding != null && binding.alignmentRoot != null)
                return binding.alignmentRoot;

            return fallback;
        }

        private bool IsGeneratedFrame(string cameraId)
        {
            CameraOpticalFrameBinding binding = GetBinding(cameraId);
            return binding == null || binding.opticalFrame == null || frames[cameraId] != binding.opticalFrame;
        }

        private void ApplyCameraTransform(string cameraId, Matrix4x4 cameraToReferenceUnity)
        {
            Transform opticalFrame = frames[cameraId];
            Transform root = alignmentRoots.TryGetValue(cameraId, out var assignedRoot) && assignedRoot != null
                ? assignedRoot
                : opticalFrame;

            Transform reference = GetReferenceParent();
            Matrix4x4 desiredOpticalWorld = reference.localToWorldMatrix * cameraToReferenceUnity;

            if (root == opticalFrame)
            {
                if (root.parent != reference)
                    root.SetParent(reference, true);

                Matrix4x4 desiredLocal = reference.worldToLocalMatrix * desiredOpticalWorld;
                RosUnityCoordinate.ApplyLocalMatrix(root, desiredLocal);
                return;
            }

            Matrix4x4 opticalInRoot = root.worldToLocalMatrix * opticalFrame.localToWorldMatrix;
            Matrix4x4 desiredRootWorld = desiredOpticalWorld * opticalInRoot.inverse;
            Matrix4x4 desiredLocalRoot = GetLocalMatrix(root, desiredRootWorld);
            RosUnityCoordinate.ApplyLocalMatrix(root, desiredLocalRoot);
        }

        private static Matrix4x4 GetLocalMatrix(Transform target, Matrix4x4 worldMatrix)
        {
            Transform parent = target.parent;
            return parent != null ? parent.worldToLocalMatrix * worldMatrix : worldMatrix;
        }

        private void ReparentPointCloudToFrame(string cameraId, Transform frame)
        {
            if (!renderers.TryGetValue(cameraId, out var renderer) || renderer == null) return;

            Transform cloud = renderer.transform;
            if (cloud.parent == frame) return;

            cloud.SetParent(frame, false);
            cloud.localPosition = Vector3.zero;
            cloud.localRotation = Quaternion.identity;
            cloud.localScale = Vector3.one;
        }

        private Transform GetInitialParentForCamera(string cameraId)
        {
            CameraOpticalFrameBinding binding = GetBinding(cameraId);
            if (binding != null && binding.lockPointCloudToOpticalFrame && binding.opticalFrame != null)
                return binding.opticalFrame;

            if (referenceOpticalFrame != null) return referenceOpticalFrame;
            return cameraRoot != null ? cameraRoot : transform;
        }

        private Transform GetReferenceParent()
        {
            if (referenceOpticalFrame != null) return referenceOpticalFrame;

            CameraOpticalFrameBinding referenceBinding = GetBinding(referenceCameraId);
            if (referenceBinding != null && referenceBinding.opticalFrame != null) return referenceBinding.opticalFrame;

            return cameraRoot != null ? cameraRoot : transform;
        }
    }
}
