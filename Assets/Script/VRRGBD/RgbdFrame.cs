using System;
using UnityEngine;

namespace VRRGBD
{
    public enum ExtrinsicsApplyMode
    {
        Manual,
        Immediate
    }

    public enum PointCloudProjectionMode
    {
        UnityZForward,
        UrdfOpticalYForward
    }

    [Serializable]
    public sealed class RgbdFrameHeader
    {
        public string camera_id;
        public string frame_id;
        public double ros_time;
        public double send_time_unix;
        public ImageInfo color = new ImageInfo();
        public ImageInfo depth = new ImageInfo();
        public CameraIntrinsics camera = new CameraIntrinsics();
    }

    [Serializable]
    public sealed class ImageInfo
    {
        public int width;
        public int height;
        public string format;
        public float unit_m = 0.001f;
    }

    [Serializable]
    public sealed class CameraIntrinsics
    {
        public float fx;
        public float fy;
        public float cx;
        public float cy;
        public float[] k = Array.Empty<float>();
        public float[] d = Array.Empty<float>();
    }

    public sealed class RgbdFrame
    {
        public RgbdFrameHeader header;
        public byte[] rgbJpeg;
        public byte[] depthPng;
        public double receivedTimeUnix;
        public uint sequence;

        public string CameraId => string.IsNullOrEmpty(header?.camera_id) ? "cam0" : header.camera_id;
        public int Width => header?.color?.width > 0 ? header.color.width : header?.depth?.width ?? 0;
        public int Height => header?.color?.height > 0 ? header.color.height : header?.depth?.height ?? 0;
        public float DepthUnitMeters => header?.depth?.unit_m > 0f ? header.depth.unit_m : 0.001f;
    }

    public sealed class CameraRuntimeStats
    {
        public string cameraId;
        public bool connected;
        public bool aligned;
        public double lastFrameUnix;
        public double lastSendUnix;
        public uint receivedFrames;
        public uint droppedFrames;
        public int pointCount;
        public float latencyMs;
        public float streamFps;
    }

    [Serializable]
    public sealed class CameraOpticalFrameBinding
    {
        public string cameraId = "cam0";
        public Transform opticalFrame;
        [Tooltip("Optional root to move when extrinsics are applied. Use the camera URDF/model root if you want the visible URDF to move with the point cloud. If empty, Optical Frame is moved.")]
        public Transform alignmentRoot;
        [Tooltip("Attach the point cloud directly to the URDF optical frame so points are emitted from the real camera frame.")]
        public bool lockPointCloudToOpticalFrame = true;
    }

    [Serializable]
    public sealed class RgbdReceiverCameraDebug
    {
        public string cameraId;
        public uint receivedFrames;
        public uint overwrittenFrames;
        public int width;
        public int height;
        public float lastLatencyMs;
        public string lastFrameId;
    }
}
