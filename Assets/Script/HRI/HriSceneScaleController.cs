using System;
using UnityEngine;

namespace HRI
{
    /// <summary>Display calibration only. Never publishes ROS commands.</summary>
    public sealed class HriSceneScaleController : MonoBehaviour
    {
        public Mast3rKeyframeDeltaViewer viewer;
        public Transform mapAdjustment;
        [Min(0.001f)] public float minimumScale = 0.01f;
        [Min(0.001f)] public float maximumScale = 100f;
        [SerializeField] private float scale = 1f;
        public float Scale => scale;
        [Min(0.001f)] public float requestedScale = 1f;
        public event Action Changed;
        private Vector3 baseScale;
        private bool initialized;

        [Header("Optional measured distance (same landmark in the SLAM map)")]
        public float knownDistanceMeters;
        public float measuredRawMapDistance;
        [Header("Optional three corresponding landmarks")]
        public Vector3[] rawMapLandmarks = new Vector3[3];
        public Vector3[] metricWorldLandmarks = new Vector3[3];
        [Min(0.001f)] public float maximumAlignmentErrorMeters = 0.05f;
        [TextArea] public string calibrationStatus = "Manual scale; extrinsics must be calibrated.";

        private void Awake() { Initialize(); }
        private bool Initialize()
        {
            if (initialized) return true;
            if (mapAdjustment == null && viewer != null) mapAdjustment = viewer.worldAdjust;
            if (mapAdjustment == null) return false;
            baseScale = mapAdjustment.localScale / Mathf.Max(scale, 0.001f);
            initialized = true;
            return true;
        }

        public void SetScale(float value)
        {
            if (!Initialize() || float.IsNaN(value) || float.IsInfinity(value)) return;
            scale = Mathf.Clamp(value, minimumScale, maximumScale);
            requestedScale = scale;
            mapAdjustment.localScale = baseScale * scale;
            if (viewer != null) viewer.RecaptureWorldAdjust();
            Changed?.Invoke();
        }
        public void ResetScale() { SetScale(1f); }
        [ContextMenu("Apply Inspector Scale")]
        public void ApplyInspectorScale() { SetScale(requestedScale); }

        [ContextMenu("Apply Known Distance Scale")]
        public void ApplyKnownDistance()
        {
            if (!(knownDistanceMeters > 0f) || !(measuredRawMapDistance > 0.00001f))
            { calibrationStatus = "Need a positive known distance and the matching raw SLAM distance."; return; }
            SetScale(knownDistanceMeters / measuredRawMapDistance);
            calibrationStatus = "Known-distance scale applied; rotation and translation still require alignment.";
        }

        [ContextMenu("Align Three Corresponding Landmarks")]
        public void AlignLandmarks()
        {
            if (!Initialize() || rawMapLandmarks.Length != 3 || metricWorldLandmarks.Length != 3) return;
            var source = new Vector3[3];
            for (int i = 0; i < 3; i++) source[i] = mapAdjustment.TransformPoint(rawMapLandmarks[i]);
            if (!TrySolveSimilarity(source, metricWorldLandmarks, maximumAlignmentErrorMeters,
                out float ratio, out Quaternion rotation, out Vector3 translation, out float error))
            { calibrationStatus = "Rejected: landmarks must be finite, non-collinear and correspond within the error limit."; return; }
            float next = scale * ratio;
            if (next < minimumScale || next > maximumScale)
            { calibrationStatus = "Rejected: scale outside configured bounds."; return; }
            // A uniformly scaled parent is required; no shear is introduced.
            if (mapAdjustment.parent != null)
            {
                Vector3 p = mapAdjustment.parent.lossyScale;
                if ((p - Vector3.one).sqrMagnitude > 0.00001f)
                { calibrationStatus = "Rejected: mapAdjustment parent must have unit scale."; return; }
            }
            Vector3 position = rotation * (ratio * mapAdjustment.position) + translation;
            mapAdjustment.SetPositionAndRotation(position, rotation * mapAdjustment.rotation);
            SetScale(next);
            calibrationStatus = $"Three-landmark similarity applied; RMS error {error:F3} m. Validate independent landmarks.";
        }

        public static bool TrySolveSimilarity(Vector3[] source, Vector3[] target, float maxError,
            out float ratio, out Quaternion rotation, out Vector3 translation, out float error)
        {
            ratio = 1; rotation = Quaternion.identity; translation = Vector3.zero; error = float.PositiveInfinity;
            if (source == null || target == null || source.Length != 3 || target.Length != 3) return false;
            for (int i = 0; i < 3; i++) if (!Finite(source[i]) || !Finite(target[i])) return false;
            Vector3 a = source[1] - source[0], b = source[2] - source[0];
            Vector3 c = target[1] - target[0], d = target[2] - target[0];
            if (a.sqrMagnitude < 1e-8f || c.sqrMagnitude < 1e-8f ||
                Vector3.Cross(a, b).sqrMagnitude < 1e-8f || Vector3.Cross(c, d).sqrMagnitude < 1e-8f) return false;
            Quaternion from = Quaternion.LookRotation(a, Vector3.Cross(a, b));
            Quaternion to = Quaternion.LookRotation(c, Vector3.Cross(c, d));
            rotation = to * Quaternion.Inverse(from);
            Vector3 meanS = (source[0] + source[1] + source[2]) / 3f;
            Vector3 meanT = (target[0] + target[1] + target[2]) / 3f;
            float numerator = 0, denominator = 0;
            for (int i = 0; i < 3; i++)
            {
                Vector3 v = source[i] - meanS;
                numerator += Vector3.Dot(rotation * v, target[i] - meanT);
                denominator += v.sqrMagnitude;
            }
            ratio = numerator / denominator;
            if (!(ratio > 0f) || float.IsInfinity(ratio)) return false;
            translation = meanT - rotation * (ratio * meanS);
            float sum = 0;
            for (int i = 0; i < 3; i++) sum += (rotation * (ratio * source[i]) + translation - target[i]).sqrMagnitude;
            error = Mathf.Sqrt(sum / 3f);
            return error <= maxError;
        }
        private static bool Finite(Vector3 p) => !float.IsNaN(p.sqrMagnitude) && !float.IsInfinity(p.sqrMagnitude);
    }
}
