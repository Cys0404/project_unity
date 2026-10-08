using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace VRRGBD
{
    [DisallowMultipleComponent]
    public sealed class RgbdBridgeBootstrap : MonoBehaviour
    {
        [Header("Ubuntu RGB-D TCP")]
        public string robotHost = "192.168.1.10";
        public int rgbdPort = 50010;

        [Header("Ubuntu ROS-TCP-Endpoint")]
        [Tooltip("IP address of the Ubuntu machine running ros_tcp_endpoint. This is used for ROS topics such as /unity/camera_extrinsics.")]
        public string rosHost = "192.168.1.10";
        [Tooltip("Port used by ros_tcp_endpoint. The Unity Robotics default is 10000.")]
        public int rosPort = 10000;
        public bool configureRosConnection = true;

        [Header("Extrinsics ROS Topic")]
        [Tooltip("Relative camera poses are received through Unity Robotics ROS TCP Connector using ROS Host/ROS Port above, not the RGB-D TCP socket.")]
        public string extrinsicsTopic = "/unity/camera_extrinsics";
        public string referenceCameraId = "cam0";
        public string[] cameraIds = { "cam0", "cam1", "cam2" };

        [Header("URDF Optical Frames")]
        [Tooltip("Drag cam0 camera_color_optical_frame here. Point clouds will be emitted in this frame.")]
        public Transform referenceOpticalFrame;
        public CameraOpticalFrameBinding[] opticalFrameBindings =
        {
            new CameraOpticalFrameBinding { cameraId = "cam0" },
            new CameraOpticalFrameBinding { cameraId = "cam1" },
            new CameraOpticalFrameBinding { cameraId = "cam2" }
        };

        [Header("Rendering Defaults")]
        [Range(1, 16)] public int downsample = 4;
        public float maxDepthMeters = 3f;
        public float pointSizeMeters = 0.008f;
        public PointCloudProjectionMode projectionMode = PointCloudProjectionMode.UrdfOpticalYForward;

        [Header("Extrinsics Alignment")]
        [Tooltip("Immediate applies every incoming /unity/camera_extrinsics pose. Manual stores the latest pose and applies it only when you click Apply Pending Manual Extrinsics.")]
        public ExtrinsicsApplyMode extrinsicsApplyMode = ExtrinsicsApplyMode.Manual;

        public RgbdTcpReceiver receiver;
        public RgbdCameraManager cameraManager;
        public CameraExtrinsicsSubscriber extrinsicsSubscriber;

        private void Awake()
        {
            ConfigureRosConnection();

            receiver = GetOrAdd<RgbdTcpReceiver>();
            receiver.host = robotHost;
            receiver.port = rgbdPort;
            receiver.connectOnStart = true;

            cameraManager = GetOrAdd<RgbdCameraManager>();
            ConfigureCameraManager();

            extrinsicsSubscriber = GetOrAdd<CameraExtrinsicsSubscriber>();
            extrinsicsSubscriber.cameraManager = cameraManager;
            extrinsicsSubscriber.extrinsicsTopic = extrinsicsTopic;
            extrinsicsSubscriber.subscribeOnStart = true;
        }

        private void Start()
        {
            if (cameraManager == null) return;
            ConfigureCameraManager();
        }

        private void OnValidate()
        {
            downsample = Mathf.Clamp(downsample, 1, 16);
            maxDepthMeters = Mathf.Max(0.05f, maxDepthMeters);
            pointSizeMeters = Mathf.Max(0.001f, pointSizeMeters);

            if (!Application.isPlaying || cameraManager == null) return;
            ConfigureCameraManager();
        }

        private T GetOrAdd<T>() where T : Component
        {
            if (TryGetComponent(out T component)) return component;
            return gameObject.AddComponent<T>();
        }

        private void ConfigureCameraManager()
        {
            cameraManager.Configure(
                receiver,
                referenceCameraId,
                cameraIds,
                referenceOpticalFrame,
                opticalFrameBindings,
                downsample,
                maxDepthMeters,
                pointSizeMeters,
                projectionMode,
                extrinsicsApplyMode);
        }

        private void ConfigureRosConnection()
        {
            if (!configureRosConnection) return;

            string host = string.IsNullOrWhiteSpace(rosHost) ? robotHost : rosHost.Trim();
            var ros = ROSConnection.GetOrCreateInstance();
            ros.RosIPAddress = host;
            ros.RosPort = rosPort;
            ros.ConnectOnStart = true;
        }
    }
}
