# VR RGB-D Point Cloud Bridge

Unity side for `vr_realsense_bridge`.

## Quick Start

1. In the scene, create an empty GameObject named `VR RGBD Bridge`.
2. Add `VRRGBD.RgbdBridgeBootstrap`.
3. Set `Robot Host` to the Ubuntu robot IP and keep `RGBD Port` at `50010`.
4. Set `ROS Host` to the same Ubuntu robot IP and keep `ROS Port` at the ROS-TCP-Endpoint port, usually `10000`.
5. Drag the URDF `camera_color_optical_frame` for `cam0` into `Reference Optical Frame`.
6. In `Optical Frame Bindings`, set `cam0` and optionally `cam1`:
   - `Optical Frame`: drag that camera's URDF `camera_color_optical_frame`.
   - `Alignment Root`: optional, drag the camera URDF/model root that should move when extrinsics arrive. If empty, Unity moves the optical frame itself.
   - Keep `Lock Point Cloud To Optical Frame` enabled so the point cloud is emitted from the URDF optical frame.
7. Start the ROS side `vr_realsense_bridge`.

## Connection Ports

There are two separate Unity connections:

- RGB-D frames: `RgbdBridgeBootstrap.Robot Host = Ubuntu_IP`, `RGBD Port = 50010`. This is the custom raw TCP stream carrying `camera_id`, RGB JPEG, depth PNG16, and intrinsics.
- Camera relative poses: `RgbdBridgeBootstrap.ROS Host = Ubuntu_IP`, `ROS Port = ROS-TCP-Endpoint port`, usually `10000`, and `CameraExtrinsicsSubscriber.Extrinsics Topic = /unity/camera_extrinsics`. This configures the Unity Robotics ROS TCP Connector automatically.

Do not set the extrinsics/ROS connector to `50010`; that port is only for the RGB-D packet stream.

The first RGB-D packet for `cam0` creates:

- `RGBD Camera Frames/cam0_CameraFrame`
- `cam0_PointCloud`

Extra cameras are created automatically when their `camera_id` appears in the TCP stream or `/unity/camera_extrinsics`.

`RgbdTcpReceiver > Runtime Debug > Camera Debug` shows per-camera received frame counts. If `cam1` does not increase there, Unity is not parsing `camera_id=cam1` from the TCP stream.
Configured cameras from `Camera Ids` and `Optical Frame Bindings` are also created on startup. If `cam2_PointCloud` exists but has `pts 0` or no received frames, the RGB-D TCP sender is not sending `camera_id=cam2`.

## Data Contract

RGB-D uses raw TCP, not rosbridge:

- magic: `RGBD`
- version: `1`
- `uint32 header_json_len`
- `uint32 rgb_len`
- `uint32 depth_len`
- UTF-8 JSON header
- JPEG RGB bytes
- 16-bit PNG depth bytes

Unity keeps only the newest frame for each camera. Old queued frames are overwritten.

Extrinsics come from `/unity/camera_extrinsics` as JSON string. `to_reference_row_major` is interpreted as camera optical frame to `cam0` optical frame. `cam0` remains identity; other Camera Frames move.
`visible_tags.cam2` does not contain a camera pose by itself. For `cam2` alignment, the JSON must include `extrinsics.cam2.to_reference_row_major`.

When `Optical Frame` is assigned, each `camX_PointCloud` is parented directly under that URDF optical frame, matching the older `PointCloudStreaming.RGBDRender.offset` behavior. For non-reference cameras, extrinsics move `Alignment Root` if assigned; this lets the visible URDF camera body move together with its point cloud.

## Performance Knobs

- `downsample`: lower values increase point-cloud density and quality. Use `1` for full resolution, `2` for a good quality/performance balance, and `4` for lighter 640x480 streaming.
- `maxDepthMeters`: clips far points in the shader. Set it close to the useful working distance to remove noisy far depth.
- `pointSizeMeters`: visual point size. Smaller values look sharper when dense; larger values hide holes when downsampled.
- `projectionMode`: use `UrdfOpticalYForward` when point clouds are parented to URDF `camera_color_optical_frame`; this matches the older `PointCloudStreaming.RGBDRender.offset` convention. Use `UnityZForward` for generated Unity-only frames.
- Disable individual camera point clouds through `RgbdCameraManager.SetCameraVisible`.

Changing `RgbdBridgeBootstrap` rendering defaults during Play mode is applied to all existing camera renderers.

## Alignment Modes

Set this on `RgbdBridgeBootstrap > Extrinsics Alignment > Extrinsics Apply Mode`. `RgbdCameraManager` receives this value at runtime and keeps its copy hidden to avoid duplicate inspector settings.

- `Immediate`: apply every `/unity/camera_extrinsics` update directly.
- `Manual`: store incoming extrinsics without moving. Click `Apply Pending Manual Extrinsics` in the `CameraExtrinsicsSubscriber` inspector, or use the component context menu, when you want to lock in the latest pose.

In the Unity Editor, `Alignment Root` transforms are restored back into Edit mode when Play mode stops, so the calibrated URDF camera pose is kept in the scene.

The renderer uses one point mesh per camera and a shader that reconstructs point positions from RGB/depth textures and camera intrinsics. It does not instantiate per-point GameObjects.
