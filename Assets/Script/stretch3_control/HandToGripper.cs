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

        // Internal variable for smoothed distance
        private float smoothedDistance = 0f;

        // ROS message that will be published
        private MessageTypes.Std.Float64 message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();

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

            // Set the ROS message data to the target angle and publish it.
            // (Make sure the receiving ROS node interprets this value correctly.)
            message.data = targetAngle;
            Publish(message);
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Std.Float64();
        }
    }
}
