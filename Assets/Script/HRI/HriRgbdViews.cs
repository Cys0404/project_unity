using UnityEngine;
using UnityEngine.Serialization;
using VRRGBD;

namespace HRI
{
    /// <summary>One RGBD receiver/consumer, four calibrated optical frames, two isolated views.</summary>
    [DefaultExecutionOrder(-150)]
    public sealed class HriRgbdViews : MonoBehaviour
    {
        public RgbdTcpReceiver receiver;
        public RgbdCameraManager manager;
        public HriFloatingMenu menu;
        public Transform headOpticalFrame, d455OpticalFrame, leftOpticalFrame, rightOpticalFrame;
        [Tooltip("Optional camera model roots to move when fixed-rig extrinsics are applied. Leave D455/head empty; assign the left/right D415 model roots if their visible camera models must move with their optical frames.")]
        public Transform leftAlignmentRoot, rightAlignmentRoot;
        public Transform surroundViewAnchor;
        public bool urdfOpticalYForward = true;
        [Range(8, 31)] public int headLayer = 28;
        [Range(8, 31)] public int surroundLayer = 29;
        [Header("Point Cloud Quality (live adjustable)")]
        [Tooltip("Pixel sampling step. 1 = every depth pixel (highest GPU/CPU cost); 2 = quarter as many points; 4 = 1/16 points.")]
        [Range(1, 16)] public int pointCloudDownsample = 4;
        [Tooltip("Discard points farther than this many metres. Lower values reduce clutter and overdraw.")]
        [Min(0.05f)] public float maxDepthMeters = 8f;
        [Tooltip("Rendered point/splat size in metres. This changes appearance, not geometric accuracy.")]
        [Min(0.001f)] public float pointSizeMeters = 0.008f;
        [FormerlySerializedAs("pointCloudLocalEulerCorrection")]
        [Tooltip("Visual-axis correction for the fixed D455 + left/right D415 point clouds. Use Z = 90 or -90 if applying extrinsics rolls only the three-camera rig. This does not change ROS extrinsics or URDF transforms.")]
        public Vector3 fixedRigPointCloudEulerCorrection = Vector3.zero;
        [Tooltip("Independent visual-axis correction for the movable D435i/head_camera point cloud.")]
        public Vector3 headPointCloudEulerCorrection = Vector3.zero;
        [Header("Menu Preview Cameras (restart Play after changing)")]
        [Range(160, 1920)] public int previewWidth = 640;
        [Range(120, 1080)] public int previewHeight = 480;
        [Range(30f, 120f)] public float previewFieldOfView = 70f;
        [Min(0.1f)] public float previewFarClipMeters = 15f;
        private Camera headCamera, surroundCamera;
        private RenderTexture headTexture, surroundTexture;
        private void Awake()
        {
            if (receiver == null || manager == null || menu == null || headOpticalFrame == null ||
                d455OpticalFrame == null || leftOpticalFrame == null || rightOpticalFrame == null || headLayer == surroundLayer)
            { Debug.LogError("HriRgbdViews requires all four calibrated optical frames and two distinct reserved layers."); enabled = false; return; }
            string[] ids = { "mast3r_camera", "camera_left", "camera_right", "head_camera" };
            Transform[] frames = { d455OpticalFrame, leftOpticalFrame, rightOpticalFrame, headOpticalFrame };
            Transform validatedLeftRoot = ValidateAlignmentRoot(leftAlignmentRoot, leftOpticalFrame, rightOpticalFrame, "left");
            Transform validatedRightRoot = ValidateAlignmentRoot(rightAlignmentRoot, rightOpticalFrame, leftOpticalFrame, "right");
            if (validatedLeftRoot != null && validatedLeftRoot == validatedRightRoot)
            {
                Debug.LogError("HriRgbdViews left/right Alignment Root cannot be the same object; falling back to both optical frames.");
                validatedLeftRoot = validatedRightRoot = null;
            }
            Transform[] alignmentRoots = { null, validatedLeftRoot, validatedRightRoot, null };
            var bindings = new CameraOpticalFrameBinding[4];
            for (int i = 0; i < 4; i++) bindings[i] = new CameraOpticalFrameBinding
                { cameraId = ids[i], opticalFrame = frames[i], alignmentRoot = alignmentRoots[i], lockPointCloudToOpticalFrame = true };
            manager.Configure(receiver, ids[0], ids, d455OpticalFrame, bindings,
                pointCloudDownsample, maxDepthMeters, pointSizeMeters,
                urdfOpticalYForward ? PointCloudProjectionMode.UrdfOpticalYForward : PointCloudProjectionMode.UnityZForward, ExtrinsicsApplyMode.Manual);
            for (int i = 0; i < 4; i++)
            {
                RgbdPointCloudRenderer cloud = manager.EnsureCamera(ids[i]);
                cloud.gameObject.layer = i == 3 ? headLayer : surroundLayer;
                cloud.transform.localRotation = Quaternion.Euler(
                    ids[i] == "head_camera" ? headPointCloudEulerCorrection : fixedRigPointCloudEulerCorrection);
            }
            headCamera = CreateCamera("Head cloud view", headLayer, out headTexture);
            surroundCamera = CreateCamera("Surround cloud view", surroundLayer, out surroundTexture);
            menu.headView = headTexture; menu.surroundView = surroundTexture;
        }
        private Camera CreateCamera(string name, int layer, out RenderTexture texture)
        {
            texture = new RenderTexture(previewWidth, previewHeight, 24) { name = name }; texture.Create();
            var go = new GameObject(name); go.transform.SetParent(transform, false);
            Camera camera = go.AddComponent<Camera>(); camera.targetTexture = texture;
            camera.cullingMask = 1 << layer; camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = Color.black; camera.nearClipPlane = 0.03f; camera.farClipPlane = previewFarClipMeters;
            camera.stereoTargetEye = StereoTargetEyeMask.None; camera.fieldOfView = previewFieldOfView;
            return camera;
        }
        private void LateUpdate()
        {
            if (headCamera == null) return;
            headCamera.transform.SetPositionAndRotation(headOpticalFrame.position, OpticalRotation(headOpticalFrame));
            Transform anchor = surroundViewAnchor != null ? surroundViewAnchor : d455OpticalFrame;
            surroundCamera.transform.SetPositionAndRotation(anchor.position,
                surroundViewAnchor != null ? anchor.rotation : OpticalRotation(anchor));
        }
        private Quaternion OpticalRotation(Transform frame) => urdfOpticalYForward
            ? Quaternion.LookRotation(frame.up, -frame.forward) : frame.rotation;
        private Transform ValidateAlignmentRoot(Transform candidate, Transform optical, Transform otherOptical, string label)
        {
            if (candidate == null) return null;
            bool ownsOptical = candidate == optical || optical.IsChildOf(candidate);
            bool containsOtherCamera = d455OpticalFrame.IsChildOf(candidate) || headOpticalFrame.IsChildOf(candidate) ||
                otherOptical.IsChildOf(candidate);
            if (!ownsOptical || containsOtherCamera || candidate.gameObject.scene != gameObject.scene)
            {
                Debug.LogError($"HriRgbdViews rejected {label} Alignment Root '{candidate.name}'. It must be a same-scene root for only that camera, not a shared robot/camera-rig ancestor. Falling back to its Optical Frame.");
                return null;
            }
            return candidate;
        }
        private void OnValidate()
        {
            pointCloudDownsample = Mathf.Clamp(pointCloudDownsample, 1, 16);
            maxDepthMeters = Mathf.Max(0.05f, maxDepthMeters);
            pointSizeMeters = Mathf.Max(0.001f, pointSizeMeters);
            previewWidth = Mathf.Clamp(previewWidth, 160, 1920);
            previewHeight = Mathf.Clamp(previewHeight, 120, 1080);
            previewFarClipMeters = Mathf.Max(0.1f, previewFarClipMeters);
            if (!Application.isPlaying || manager == null) return;
            manager.defaultDownsample = pointCloudDownsample;
            manager.defaultMaxDepthMeters = maxDepthMeters;
            manager.defaultPointSizeMeters = pointSizeMeters;
            manager.defaultProjectionMode = urdfOpticalYForward
                ? PointCloudProjectionMode.UrdfOpticalYForward : PointCloudProjectionMode.UnityZForward;
            manager.ApplyRenderingDefaultsToAll();
            string[] ids = { "mast3r_camera", "camera_left", "camera_right", "head_camera" };
            foreach (string id in ids)
                manager.EnsureCamera(id).transform.localRotation = Quaternion.Euler(
                    id == "head_camera" ? headPointCloudEulerCorrection : fixedRigPointCloudEulerCorrection);
        }
        private void OnDestroy()
        {
            if (headCamera != null) Destroy(headCamera.gameObject);
            if (surroundCamera != null) Destroy(surroundCamera.gameObject);
            if (headTexture != null) { headTexture.Release(); Destroy(headTexture); }
            if (surroundTexture != null) { surroundTexture.Release(); Destroy(surroundTexture); }
        }
    }
}
