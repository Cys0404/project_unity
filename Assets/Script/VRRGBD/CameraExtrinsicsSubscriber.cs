using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Collections.Generic;
using System.Text;
using System.Text.RegularExpressions;
using UnityEngine.SceneManagement;

namespace VRRGBD
{
    public sealed class CameraExtrinsicsSubscriber : MonoBehaviour
    {
        [Tooltip("Unity Robotics ROS-TCP topic. Active HRI profile: Robot publishes in domain 0, Thor bridges it to domain 95, and Unity's single ROSConnection uses Thor 192.168.0.30:10003. Robot :10000 is legacy/direct mode and cannot simultaneously carry the Thor map through the same singleton. This is never rosbridge :9090 or raw RGB-D :50010.")]
        public string extrinsicsTopic = "/unity/camera_extrinsics";
        public RgbdCameraManager cameraManager;
        public bool subscribeOnStart = true;
        public bool verbose;

        [Header("Manual Calibration Persistence")]
        [Tooltip("Save the left/right transforms that were manually applied, then load them as pending on the next Play/build launch. A manual Apply is still required before anything moves.")]
        public bool persistLastManualCalibration = true;
        [Tooltip("PlayerPrefs key. Change it only when separate scenes/robots must keep independent calibrations.")]
        public string calibrationPersistenceKey = "VRRGBD.HRI.FixedRigExtrinsics.v1";

        [Header("Manual Test")]
        [TextArea(3, 10)] public string manualTestJson;

        [Header("Runtime Debug")]
        public int receivedMessages;
        public int appliedExtrinsics;
        public string lastCameraId;
        [TextArea(2, 6)] public string lastJson;
        public string lastStatus;

        private ROSConnection ros;
        private readonly Dictionary<string, string> latestFixedRigObjects = new Dictionary<string, string>();

        [ContextMenu("Apply Manual Test Json")]
        public void ApplyManualTestJson()
        {
            if (cameraManager == null) cameraManager = FindObjectOfType<RgbdCameraManager>();
            if (string.IsNullOrWhiteSpace(manualTestJson))
            {
                lastStatus = "Manual Test Json is empty.";
                return;
            }

            lastJson = manualTestJson;
            ApplyExtrinsicsJson(manualTestJson);
        }

        [ContextMenu("Apply Pending Manual Extrinsics")]
        public void ApplyPendingManualExtrinsics()
        {
            ApplyPendingFixedRigExtrinsics();
        }

        public int ApplyPendingFixedRigExtrinsics()
        {
            if (cameraManager == null) cameraManager = FindObjectOfType<RgbdCameraManager>();
            if (cameraManager == null) return 0;
            string persistenceJson = BuildPendingFixedRigJson();
            int applied = cameraManager.ApplyPendingManualExtrinsicsForCameras("camera_left", "camera_right");
            if (applied > 0)
            {
                string key = EffectivePersistenceKey();
                if (persistLastManualCalibration && !string.IsNullOrWhiteSpace(key) &&
                    !string.IsNullOrWhiteSpace(persistenceJson))
                {
                    PlayerPrefs.SetString(key, persistenceJson);
                    PlayerPrefs.Save();
                }
                lastStatus = $"Applied {applied} pending fixed-rig transform(s)" +
                    (persistLastManualCalibration ? " and saved calibration." : ".");
            }
            return applied;
        }

        [ContextMenu("Clear Saved Manual Calibration")]
        public void ClearSavedManualCalibration()
        {
            string key = EffectivePersistenceKey();
            if (!string.IsNullOrWhiteSpace(key))
            {
                PlayerPrefs.DeleteKey(key);
                PlayerPrefs.Save();
            }
            lastStatus = "Cleared saved fixed-rig calibration.";
        }

        private void Start()
        {
            if (cameraManager == null) cameraManager = FindObjectOfType<RgbdCameraManager>();
            RestoreSavedManualCalibration();
            if (!subscribeOnStart) return;

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<StringMsg>(extrinsicsTopic, OnExtrinsics);
        }

        private void RestoreSavedManualCalibration()
        {
            string key = EffectivePersistenceKey();
            if (!persistLastManualCalibration || cameraManager == null ||
                string.IsNullOrWhiteSpace(key) || !PlayerPrefs.HasKey(key)) return;

            string savedJson = PlayerPrefs.GetString(key, string.Empty);
            if (string.IsNullOrWhiteSpace(savedJson)) return;
            string savedReference = RgbdHeaderParser.ReadString(savedJson, "reference_camera", string.Empty);
            if (savedReference != cameraManager.referenceCameraId)
            {
                lastStatus = $"Ignored saved calibration: reference '{savedReference}' does not match '{cameraManager.referenceCameraId}'.";
                return;
            }
            lastJson = savedJson;
            ApplyExtrinsicsJson(savedJson);
            int restored = cameraManager.PendingManualExtrinsicsCountForCameras("camera_left", "camera_right");
            lastStatus = restored > 0
                ? $"Loaded {restored} saved fixed-rig transform(s) as pending; press Apply to move cameras."
                : "Saved fixed-rig calibration was found but no left/right transform could be queued.";
        }

        private string EffectivePersistenceKey()
        {
            if (string.IsNullOrWhiteSpace(calibrationPersistenceKey) || cameraManager == null) return string.Empty;
            return calibrationPersistenceKey + "." + gameObject.scene.name + "." + cameraManager.referenceCameraId;
        }

        private string BuildPendingFixedRigJson()
        {
            if (cameraManager == null) return string.Empty;
            var builder = new StringBuilder();
            builder.Append("{\"reference_camera\":\"").Append(cameraManager.referenceCameraId)
                .Append("\",\"extrinsics\":{");
            bool wrote = false;
            foreach (string cameraId in new[] { "camera_left", "camera_right" })
            {
                if (cameraManager.PendingManualExtrinsicsCountForCameras(cameraId) <= 0 ||
                    !latestFixedRigObjects.TryGetValue(cameraId, out string objectJson)) continue;
                if (wrote) builder.Append(',');
                builder.Append('\"').Append(cameraId).Append("\":").Append(objectJson);
                wrote = true;
            }
            builder.Append("}}");
            return wrote ? builder.ToString() : string.Empty;
        }

        private void OnExtrinsics(StringMsg msg)
        {
            receivedMessages++;
            if (cameraManager == null)
            {
                lastStatus = "No RgbdCameraManager assigned.";
                return;
            }
            if (msg == null || string.IsNullOrEmpty(msg.data))
            {
                lastStatus = "Empty extrinsics message.";
                return;
            }

            lastJson = msg.data;
            ApplyExtrinsicsJson(msg.data);
        }

        public void ApplyExtrinsicsJson(string json)
        {
            lastStatus = string.Empty;
            bool anyApplied = false;
            var processedObjects = new HashSet<string>();
            foreach (Match match in Regex.Matches(json, "\"(?<id>[^\"]+)\"\\s*:\\s*(?<obj>\\{[^\\{\\}]*\"to_reference_row_major\"\\s*:\\s*\\[[^\\]]*\\][^\\{\\}]*\\})"))
            {
                string cameraId = match.Groups["id"].Value;
                if (cameraId == "extrinsics") continue;
                string objectJson = match.Groups["obj"].Value;
                processedObjects.Add(objectJson);
                anyApplied |= TryApplySingleExtrinsic(objectJson, cameraId);
            }

            foreach (Match match in Regex.Matches(json, "\\{[^\\{\\}]*\"to_reference_row_major\"\\s*:\\s*\\[[^\\]]*\\][^\\{\\}]*\\}"))
            {
                if (processedObjects.Add(match.Value)) anyApplied |= TryApplySingleExtrinsic(match.Value);
            }

            if (!anyApplied && string.IsNullOrEmpty(lastStatus))
                lastStatus = "No camera extrinsic object found in JSON.";
        }

        private bool TryApplySingleExtrinsic(string json)
        {
            return TryApplySingleExtrinsic(json, string.Empty);
        }

        private bool TryApplySingleExtrinsic(string json, string fallbackCameraId)
        {
            string cameraId = RgbdHeaderParser.ReadString(json, "camera_id", string.Empty);
            if (string.IsNullOrEmpty(cameraId)) cameraId = RgbdHeaderParser.ReadString(json, "child_frame_id", string.Empty);
            if (string.IsNullOrEmpty(cameraId)) cameraId = RgbdHeaderParser.ReadString(json, "camera", string.Empty);
            if (string.IsNullOrEmpty(cameraId)) cameraId = fallbackCameraId;
            if (string.IsNullOrEmpty(cameraId))
            {
                lastStatus = "Missing camera_id / child_frame_id / camera.";
                if (verbose) Debug.LogWarning("[CameraExtrinsicsSubscriber] Missing camera_id.");
                return false;
            }

            lastCameraId = cameraId;
            bool isReference = cameraId == cameraManager.referenceCameraId;
            bool isMovableFixedCamera = cameraId == "camera_left" || cameraId == "camera_right";
            if (!isReference && !isMovableFixedCamera)
            {
                lastStatus = $"Ignored {cameraId}; fixed-rig alignment accepts only the reference D455 and left/right cameras.";
                return true;
            }
            if (isReference)
            {
                cameraManager.SetCameraTransform(cameraId, Matrix4x4.identity, true);
                appliedExtrinsics++;
                lastStatus = $"Applied identity for reference {cameraId}.";
                return true;
            }

            float[] values = RgbdHeaderParser.ReadFloatArray(json, "to_reference_row_major");
            if (values.Length != 16)
            {
                lastStatus = $"Missing to_reference_row_major 4x4 matrix for {cameraId}. Parsed {values.Length} values.";
                if (verbose) Debug.LogWarning($"[CameraExtrinsicsSubscriber] Missing 4x4 matrix for {cameraId}.");
                return false;
            }

            var m = new Matrix4x4
            {
                m00 = values[0],  m01 = values[1],  m02 = values[2],  m03 = values[3],
                m10 = values[4],  m11 = values[5],  m12 = values[6],  m13 = values[7],
                m20 = values[8],  m21 = values[9],  m22 = values[10], m23 = values[11],
                m30 = values[12], m31 = values[13], m32 = values[14], m33 = values[15]
            };

            latestFixedRigObjects[cameraId] = json;
            cameraManager.SetCameraTransform(cameraId, m, false);
            appliedExtrinsics++;
            lastStatus = cameraManager.extrinsicsApplyMode == ExtrinsicsApplyMode.Manual
                ? $"Received and queued manual extrinsics for {cameraId}."
                : $"Applied extrinsics for {cameraId}.";
            return true;
        }
    }
}
