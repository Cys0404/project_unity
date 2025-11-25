using UnityEngine;

public class HandToShoulder : MonoBehaviour
{
    [Header("Transforms")]
    [Tooltip("Drag the robot's shoulder transform here.")]
    public Transform robotShoulder;

    [Tooltip("Drag the R_Wrist transform here.")]
    public Transform rWrist;

    [Header("Options")]
    [Tooltip("An optional offset for the y-axis.")]
    public float yOffset = 0f;

    [Header("Protection Settings")]
    [Tooltip("Maximum change in y-position per second (units/sec) to protect the robot motor.")]
    public float maxYSpeed = 1.0f;

    [Header("Deceleration Settings")]
    [Tooltip("Distance threshold within which the arm will decelerate.")]
    public float decelerationDistance = 0.2f;

    [Tooltip("A minimum speed factor to ensure there is always some movement (value between 0 and 1).")]
    [Range(0f, 1f)]
    public float minSpeedFactor = 0.1f;

    [Header("Position Constraints")]
    [Tooltip("Minimum y-value for the shoulder.")]
    public float minYPosition = -1.0f;

    [Tooltip("Maximum y-value for the shoulder.")]
    public float maxYPosition = 1.0f;

    [Header("Debug Settings")]
    [Tooltip("Enable or disable debug logs.")]
    public bool enableDebugLogs = true;

    void Update()
    {
        // Check that both transforms are assigned.
        if (robotShoulder == null || rWrist == null)
        {
            Debug.LogWarning("Please assign both the robot's shoulder and R_Wrist transforms in the Inspector.");
            return;
        }

        // Use the R_Wrist's world position to determine the target y-value.
        float targetY = rWrist.position.y + yOffset;
        float currentY = robotShoulder.position.y;

        // Log the target and current positions if debugging is enabled.
        if (enableDebugLogs)
        {
            Debug.Log("Current Y Position of Shoulder: " + currentY);
            Debug.Log("Target Y Position: " + targetY);
        }

        // Calculate the distance to the target.
        float distance = Mathf.Abs(targetY - currentY);

        // Determine a speed factor based on the remaining distance.
        // If the distance is smaller than decelerationDistance, reduce speed proportionally.
        // Otherwise, use full speed.
        float speedFactor = distance < decelerationDistance ? Mathf.Clamp(distance / decelerationDistance, minSpeedFactor, 1f) : 1f;
        float allowedSpeed = maxYSpeed * speedFactor;

        // Smoothly move toward the target y position.
        float newY = Mathf.MoveTowards(currentY, targetY, allowedSpeed * Time.deltaTime);

        // Clamp the new y position to ensure it stays within the min and max bounds.
        newY = Mathf.Clamp(newY, minYPosition, maxYPosition);

        // Log the new clamped y-position if debugging is enabled.
        if (enableDebugLogs)
        {
            Debug.Log("New Y Position (Clamped): " + newY);
        }

        // Apply the new y position (preserving x and z).
        Vector3 shoulderPos = robotShoulder.position;
        shoulderPos.y = newY;
        robotShoulder.position = shoulderPos;

        if (enableDebugLogs)
        {
            Debug.Log("Current Y Position of Shoulder: " + robotShoulder.position.y);
        }
    }
}
