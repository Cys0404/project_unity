using UnityEngine;

namespace HRI
{
    /// <summary>Re-expresses a tracked hand in the stationary command rig's calibration space.</summary>
    [DefaultExecutionOrder(-250)]
    public sealed class HriTrackedPoseProxy : MonoBehaviour
    {
        public Transform source;
        public Transform trackingSpace;
        public Transform commandTrackingSpace;
        private void Update()
        {
            if (source == null || trackingSpace == null || commandTrackingSpace == null) return;
            transform.SetPositionAndRotation(
                commandTrackingSpace.TransformPoint(trackingSpace.InverseTransformPoint(source.position)),
                commandTrackingSpace.rotation * Quaternion.Inverse(trackingSpace.rotation) * source.rotation);
        }
    }
}
