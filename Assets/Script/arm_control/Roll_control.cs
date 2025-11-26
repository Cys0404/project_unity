using UnityEngine;

public class Roll_Control : MonoBehaviour
{
    // Reference to the XR Hands' R_IndexProximal transform.
    public Transform handWrist;

    // Reference to the gripper (could be a Transform or a custom gripper controller).
    public Transform gripperStatic;

    // Multiplier to scale the finger rotation value before applying it to the gripper.
    [Tooltip("Multiplier to map the finger's rotation to the gripper's rotation.")]
    public float rotationMultiplier = 1.0f;

    void Update()
    {
        // Ensure both references are set.
        if (handWrist == null || gripperStatic == null)
            return;

        // Capture the local rotation of the index finger's proximal bone.
        // Here, we assume the rotation around the local X axis is the one to use.
        float HandRotation = handWrist.localEulerAngles.z;

        // Map or scale the finger rotation as needed.
        float RollRotation = HandRotation * rotationMultiplier; // add some offset to make the rotatio correct

        // For this example, we apply the computed rotation to the gripper's Y axis.
        // Adjust the axis or use a more sophisticated mapping if necessary.
        Vector3 currentRollEuler = gripperStatic.localEulerAngles;
        gripperStatic.localRotation = Quaternion.Euler(RollRotation, currentRollEuler.y, currentRollEuler.z);
    }
}
