using UnityEngine;

namespace VRRGBD
{
    public static class RosUnityCoordinate
    {
        private static readonly Matrix4x4 OpticalRosToUnityZForward = Matrix4x4.Scale(new Vector3(1f, -1f, 1f));
        private static readonly Matrix4x4 OpticalRosToUnityYForward = new Matrix4x4(
            new Vector4(1f, 0f, 0f, 0f),
            new Vector4(0f, 0f, 1f, 0f),
            new Vector4(0f, 1f, 0f, 0f),
            new Vector4(0f, 0f, 0f, 1f));

        public static Vector3 OpticalPointToUnity(Vector3 rosOptical)
        {
            return new Vector3(rosOptical.x, -rosOptical.y, rosOptical.z);
        }

        public static Matrix4x4 OpticalTransformToUnity(Matrix4x4 cameraToReferenceRos)
        {
            return OpticalTransformToUnity(cameraToReferenceRos, PointCloudProjectionMode.UnityZForward);
        }

        public static Matrix4x4 OpticalTransformToUnity(Matrix4x4 cameraToReferenceRos, PointCloudProjectionMode projectionMode)
        {
            Matrix4x4 basis = projectionMode == PointCloudProjectionMode.UrdfOpticalYForward
                ? OpticalRosToUnityYForward
                : OpticalRosToUnityZForward;

            return basis * cameraToReferenceRos * basis.inverse;
        }

        public static void ApplyLocalMatrix(Transform target, Matrix4x4 matrix)
        {
            Vector3 right = matrix.GetColumn(0);
            Vector3 up = matrix.GetColumn(1);
            Vector3 forward = matrix.GetColumn(2);
            Vector3 position = matrix.GetColumn(3);

            target.localPosition = position;
            target.localRotation = forward.sqrMagnitude > 0.000001f && up.sqrMagnitude > 0.000001f
                ? Quaternion.LookRotation(forward, up)
                : Quaternion.identity;
            target.localScale = new Vector3(right.magnitude, up.magnitude, forward.magnitude);
        }
    }
}
