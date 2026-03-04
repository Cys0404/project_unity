using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class HandToGripper : UnityPublisher<MessageTypes.Std.Float64>
    {
        [Header("Gripper Finger Transforms (drag from Hierarchy)")]
        [Tooltip("Transform for the right gripper finger (e.g., link_gripper_finger_right)")]
        public Transform gripperFingerRight;
        [Tooltip("Transform for the left gripper finger (e.g., link_gripper_finger_left)")]
        public Transform gripperFingerLeft;

        [Header("Hand Joint Transforms (drag from Hierarchy)")]
        [Tooltip("Transform representing the Index Tip joint")]
        public Transform indexTip;
        [Tooltip("Transform representing the Thumb Tip joint")]
        public Transform thumbTip;

        [Header("Gripper Rotation Settings")]
        [Tooltip("Rotation angle (in degrees) when the gripper is fully open (right finger rotates +, left finger rotates -)")]
        private float maxOpenAngle = 29f;
        [Tooltip("Rotation angle (in degrees) when the gripper is closed")]
        private float closedAngle = 0f;

        [Header("Distance Thresholds (in meters)")]
        [Tooltip("Distance below which the gripper is considered closed")]
        public float minDistance = 0.02f;  // e.g., 2 cm
        [Tooltip("Distance above which the gripper is considered fully open")]
        public float maxDistance = 0.1f;   // e.g., 10 cm

        [Header("Smoothing Settings")]
        [Tooltip("Smoothing speed for the measured distance (higher values = less smoothing)")]
        public float distanceSmoothing = 10f;
        [Tooltip("Speed of rotation interpolation (set to 0 for immediate rotation)")]
        private float rotationSpeed = 0f;

        // -------------------- Added: ROS publish control & connector --------------------
        [Header("ROS Publish Control")]
        [Tooltip("Enable/disable publishing to ROS. Turn OFF for offline testing to keep Console quiet.")]
        public bool enableRosPublish = true;

        [Tooltip("If true, publish only when RosConnector has a valid RosSocket (connected).")]
        public bool publishOnlyWhenConnected = true;

        [Header("ROS Connector Reference (optional)")]
        [Tooltip("Assign RosConnector here (recommended). If not assigned, script will try FindObjectOfType at Start().")]
        public RosConnector rosConnector;
        // -------------------------------------------------------------------------------

        // Internal variable for smoothed distance
        private float smoothedDistance = 0f;

        // ROS message that will be published
        private MessageTypes.Std.Float64 message;

        protected override void Start()
        {
            // -------------------- Added: initialize message BEFORE base.Start() --------------------
            // Why: base.Start() depends on ROS connection setup; if ROS bridge isn't running,
            // base.Start() may fail/throw and prevent InitializeMessage() from being called.
            // We want message to ALWAYS exist so Unity local control never crashes.
            InitializeMessage();
            // --------------------------------------------------------------------------------------

            base.Start();

            // -------------------- Added: get RosConnector (optional) --------------------
            // If you didn't drag RosConnector in Inspector, we try to find it once at Start().
            // (We intentionally do NOT call FindObjectOfType every frame in Update.)
            if (rosConnector == null)
                rosConnector = FindObjectOfType<RosConnector>();
            // ---------------------------------------------------------------------------

            // Initialize the smoothed distance to the first measurement if available
            if (indexTip != null && thumbTip != null)
            {
                smoothedDistance = Vector3.Distance(indexTip.position, thumbTip.position);
            }
        }

        void Update()
        {
            // Check if hand joint references are assigned
            if (indexTip == null || thumbTip == null)
            {
                Debug.LogWarning("IndexTip or ThumbTip transform is not assigned.");
                return;
            }

            // -------------------- Added: check gripper finger references too --------------------
            // Your original code assumes these exist. If not assigned, it would NullRef earlier.
            if (gripperFingerRight == null || gripperFingerLeft == null)
            {
                Debug.LogWarning("Gripper finger transforms are not assigned.");
                return;
            }
            // ---------------------------------------------------------------------------

            // Measure the current distance between the index tip and thumb tip
            float measuredDistance = Vector3.Distance(indexTip.position, thumbTip.position);

            // Smooth the measured distance using a low-pass filter to reduce jitter
            smoothedDistance = Mathf.Lerp(smoothedDistance, measuredDistance, Time.deltaTime * distanceSmoothing);

            // Map the smoothed distance to a normalized value (0 to 1) based on minDistance and maxDistance
            float t = Mathf.InverseLerp(minDistance, maxDistance, smoothedDistance);

            // Compute the target rotation angle (in degrees)
            // When t=0 (fingers very close) -> angle = closedAngle.
            // When t=1 (fingers far apart) -> angle = maxOpenAngle.
            float targetAngle = Mathf.Lerp(closedAngle, maxOpenAngle, t);

            // Update the gripper finger rotations in the scene
            if (rotationSpeed > 0f)
            {
                // If using smooth interpolation, create target rotations and lerp toward them
                Quaternion targetRight = Quaternion.Euler(
                    gripperFingerRight.localRotation.eulerAngles.x,
                    targetAngle,
                    gripperFingerRight.localRotation.eulerAngles.z);
                Quaternion targetLeft = Quaternion.Euler(
                    gripperFingerLeft.localRotation.eulerAngles.x,
                    -targetAngle,
                    gripperFingerLeft.localRotation.eulerAngles.z);

                gripperFingerRight.localRotation = Quaternion.Lerp(gripperFingerRight.localRotation, targetRight, Time.deltaTime * rotationSpeed);
                gripperFingerLeft.localRotation = Quaternion.Lerp(gripperFingerLeft.localRotation, targetLeft, Time.deltaTime * rotationSpeed);
            }
            else
            {
                // Immediate rotation update with an offset (adjust offset if necessary)
                Vector3 rightEuler = gripperFingerRight.localRotation.eulerAngles;
                Vector3 leftEuler = gripperFingerLeft.localRotation.eulerAngles;
                gripperFingerRight.localRotation = Quaternion.Euler(targetAngle + 60, rightEuler.y, rightEuler.z);
                gripperFingerLeft.localRotation = Quaternion.Euler(-targetAngle - 60, rightEuler.y, leftEuler.z);
            }

            // -------------------- Added: ROS publish gating (quiet when offline) --------------------
            // If you are testing without rosbridge/bridge running, Publish() may spam errors.
            // These flags let you keep local Unity control working while skipping ROS publish.
            if (!enableRosPublish)
                return;

            if (publishOnlyWhenConnected)
            {
                // If no connector or no socket, we assume "not connected" and skip silently.
                if (rosConnector == null || rosConnector.RosSocket == null)
                    return;
            }
            // --------------------------------------------------------------------------------------

            // Set the ROS message data to the target angle and publish it.
            // (Make sure the receiving ROS node interprets this value correctly.)
            // -------------------- Added: safety re-init in case lifecycle got weird --------------------
            if (message == null)
                InitializeMessage();
            // --------------------------------------------------------------------------------------

            message.data = targetAngle;

            // -------------------- Added: extra safety before Publish --------------------
            // If publishOnlyWhenConnected is false, Publish might still throw if socket is null,
            // so we guard it anyway.
            if (!publishOnlyWhenConnected || (rosConnector != null && rosConnector.RosSocket != null))
                Publish(message);
            // --------------------------------------------------------------------------
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Std.Float64();
        }
    }
}