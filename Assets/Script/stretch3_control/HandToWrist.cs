using UnityEngine;
using System.Collections.Generic;
using RosSharp;
using RosSharp.RosBridgeClient;
using UnityEngine.InputSystem;

public class HandToWrist : MonoBehaviour
{
    public Transform rollJoint;
    public JointStateWriter rollJointWriter;
    public Transform pitchJoint;
    public Transform yawJoint;
    public Transform R_Wrist; // R_Wrist rotation as a Transform
    public bool debug_message = false;

    public float smoothSpeed = 5f;

    [System.Serializable]
    public struct AngleLimits
    {
        public float Min;
        public float Max;

        public AngleLimits(float min, float max)
        {
            Min = min;
            Max = max;
        }
    }

    public AngleLimits rollLimits = new AngleLimits(-160f, 160f);
    public AngleLimits pitchLimits = new AngleLimits(-32f, 90f);
    public AngleLimits yawLimits = new AngleLimits(-100f, 100f);

    private Vector3 initialRollRotation;
    private Vector3 initialPitchRotation;
    private Vector3 initialYawRotation;

    void Start()
    {
        initialRollRotation = rollJoint.localRotation.eulerAngles;
        initialPitchRotation = pitchJoint.localRotation.eulerAngles;
        initialYawRotation = yawJoint.localRotation.eulerAngles;
    }

    void Update()
    {
        if (R_Wrist != null && rollJoint != null && pitchJoint != null && yawJoint != null)
        {
            // Get the local rotation of the joystick
            Vector3 R_WristLocalEulerAngles = R_Wrist.localRotation.eulerAngles;

            // Extract angles and normalize them
            float rotateRoll = NormalizeAngle(R_WristLocalEulerAngles.z);
            float rotatePitch = NormalizeAngle(R_WristLocalEulerAngles.x);
            float rotateYaw = NormalizeAngle(R_WristLocalEulerAngles.y);

            // Clamp the angles based on the limits
            float clampedRoll = Mathf.Clamp(rotateRoll, rollLimits.Min, rollLimits.Max);
            float clampedPitch = Mathf.Clamp(rotatePitch, pitchLimits.Min, pitchLimits.Max);
            float clampedYaw = Mathf.Clamp(rotateYaw, yawLimits.Min, yawLimits.Max);

            if (debug_message)
            {
                Debug.Log($"Roll: {rotateRoll} Clamped: {clampedRoll}");
                Debug.Log($"Pitch: {rotatePitch} Clamped: {clampedPitch}");
                Debug.Log($"Yaw: {rotateYaw} Clamped: {clampedYaw}");
            }

            // Adjust yawJoint based on the local rotation
            Quaternion targetYawRotation = Quaternion.Euler(initialYawRotation.x + clampedYaw, 180.0f, 90.0f);
            yawJoint.localRotation = Quaternion.Slerp(yawJoint.localRotation, targetYawRotation, Time.deltaTime * smoothSpeed);

            // Writing rollJointWriter in radians
            rollJointWriter.Write(-clampedRoll * Mathf.Deg2Rad);

            // Adjust pitchJoint based on the clamped pitch
            Quaternion targetPitchRotation = Quaternion.Euler(initialPitchRotation.x - clampedPitch, initialPitchRotation.y, initialPitchRotation.z);
            pitchJoint.localRotation = Quaternion.Slerp(pitchJoint.localRotation, targetPitchRotation, Time.deltaTime * smoothSpeed);
        }
    }

    private float NormalizeAngle(float angle)
    {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        return angle;
    }
}
