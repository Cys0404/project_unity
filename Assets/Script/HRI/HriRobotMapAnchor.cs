using UnityEngine;

namespace HRI
{
    /// <summary>Places a metric robot from the SLAM camera pose and its URDF extrinsic.</summary>
    [DefaultExecutionOrder(100)]
    public sealed class HriRobotMapAnchor : MonoBehaviour
    {
        public Mast3rKeyframeDeltaViewer viewer;
        public Transform robotRoot;
        public Transform slamCameraOpticalFrame;
        public bool urdfOpticalYForward = true;
        [Min(0.1f)] public float staleAfterSeconds = 2f;
        public string status = "Waiting for SLAM trajectory";
        public bool IsFresh => viewer != null && viewer.LastCameraPoseReceivedAt >= 0 &&
            Time.realtimeSinceStartup - viewer.LastCameraPoseReceivedAt < staleAfterSeconds;

        private void LateUpdate()
        {
            if (viewer == null || robotRoot == null || slamCameraOpticalFrame == null) return;
            if (!slamCameraOpticalFrame.IsChildOf(robotRoot))
            { status = "The calibrated SLAM optical frame must be a child of robotRoot."; return; }
            if ((robotRoot.lossyScale - Vector3.one).sqrMagnitude > 0.00001f)
            { status = "robotRoot must be metric (unit world scale), outside scaled mapAdjustment."; return; }
            if (!viewer.TryGetLatestRawCameraPose(out Pose raw)) return;
            Matrix4x4 map = viewer.MapToWorldMatrix;
            Vector3 forward = map.MultiplyVector(raw.rotation * Vector3.forward).normalized;
            Vector3 up = map.MultiplyVector(raw.rotation * Vector3.down).normalized;
            if (Vector3.Cross(forward, up).sqrMagnitude < 0.00001f) return;
            Quaternion desiredView = Quaternion.LookRotation(forward, up);
            Quaternion currentView = urdfOpticalYForward
                ? Quaternion.LookRotation(slamCameraOpticalFrame.up, -slamCameraOpticalFrame.forward)
                : slamCameraOpticalFrame.rotation;
            Quaternion relativeView = Quaternion.Inverse(robotRoot.rotation) * currentView;
            Vector3 relativePosition = robotRoot.InverseTransformPoint(slamCameraOpticalFrame.position);
            Quaternion rootRotation = desiredView * Quaternion.Inverse(relativeView);
            Vector3 rootPosition = map.MultiplyPoint3x4(raw.position) - rootRotation * relativePosition;
            robotRoot.SetPositionAndRotation(rootPosition, rootRotation);
            status = IsFresh ? "SLAM camera pose + metric URDF extrinsic" : "Trajectory stale: displaying last pose";
        }
    }
}
