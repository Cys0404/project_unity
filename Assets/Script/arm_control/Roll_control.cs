using UnityEngine;

public class GripperRollController : MonoBehaviour
{
    // The input source from which the roll value is taken.
    public Transform inputSource;

    // The gripper transform whose roll will be controlled.
    public Transform gripper;

    // Multiplier to scale the input roll.
    [Tooltip("Multiplier to map the input source's roll to the gripper's roll.")]
    public float rollMultiplier = 1.0f;

    // Safe limits for the gripper's roll angle (in degrees).
    [Tooltip("Minimum allowed gripper roll angle.")]
    public float minRollAngle = -45.0f;
    [Tooltip("Maximum allowed gripper roll angle.")]
    public float maxRollAngle = 45.0f;

    // Smoothing factor to gradually approach the target roll angle.
    [Tooltip("Speed at which the gripper approaches the target roll angle.")]
    public float smoothSpeed = 5.0f;

    void Update()
    {
        // Ensure that both the input source and gripper are assigned.
        if (inputSource == null || gripper == null)
            return;

        // Capture the roll value from the input source.
        // Note: Unity's Euler angles are in the range [0, 360], so we normalize it.
        float inputRoll = NormalizeAngle(inputSource.localEulerAngles.z);

        // Compute the target roll by applying the multiplier.
        float targetRoll = inputRoll * rollMultiplier;

        // Clamp the target roll to the safe range.
        targetRoll = Mathf.Clamp(targetRoll, minRollAngle, maxRollAngle);

        // Get the current roll of the gripper, normalized to [-180, 180].
        float currentRoll = NormalizeAngle(gripper.localEulerAngles.z);

        // Smoothly interpolate from the current roll to the target roll.
        float newRoll = Mathf.LerpAngle(currentRoll, targetRoll, smoothSpeed * Time.deltaTime);

        // Update the gripper's rotation while preserving its other rotational axes.
        Vector3 currentEuler = gripper.localEulerAngles;
        gripper.localRotation = Quaternion.Euler(currentEuler.x, currentEuler.y, newRoll);
    }

    // Helper function to normalize an angle to the range [-180, 180].
    float NormalizeAngle(float angle)
    {
        angle = (angle + 180f) % 360f;
        if (angle < 0)
            angle += 360f;
        return angle - 180f;
    }
}
