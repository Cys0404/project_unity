using UnityEngine;
using RosSharp.RosBridgeClient;  // Include the correct namespace
public class RobotJointController : MonoBehaviour
{
    [System.Serializable]
    public class RobotJoint
    {
        public GameObject jointObject;  // Reference to the joint GameObject
        public Vector3 rotationAxis;   // Rotation axis (1,0,0) for X, (0,1,0) for Y, (0,0,1) for Z
        public float rotationStep = 5f;  // Rotation increment value
        public float minRotation = -180f;  // Minimum rotation angle
        public float maxRotation = 180f;   // Maximum rotation angle
    }

    public RobotJoint[] robotJoints;  // Array of robot joints
    public JointStatePublisher jointState_Pub;  // Direct reference to the script

    void Update()
    {
        // Joint control mappings
        if (Input.GetKey(KeyCode.A)) RotateJoint(0, 1);
        if (Input.GetKey(KeyCode.Z)) RotateJoint(0, -1);

        if (Input.GetKey(KeyCode.S)) RotateJoint(1, 1);
        if (Input.GetKey(KeyCode.X)) RotateJoint(1, -1);

        if (Input.GetKey(KeyCode.D)) RotateJoint(2, 1);
        if (Input.GetKey(KeyCode.C)) RotateJoint(2, -1);

        if (Input.GetKey(KeyCode.F)) RotateJoint(3, 1);
        if (Input.GetKey(KeyCode.V)) RotateJoint(3, -1);

        if (Input.GetKey(KeyCode.G)) RotateJoint(4, 1);
        if (Input.GetKey(KeyCode.B)) RotateJoint(4, -1);

        if (Input.GetKey(KeyCode.H)) RotateJoint(5, 1);
        if (Input.GetKey(KeyCode.N)) RotateJoint(5, -1);

        // Enable/disable the JointStatePublisher script
        if (Input.GetKeyDown(KeyCode.Space))
        {
            SetJointStatePublisherActive(true);
        }
        if (Input.GetKeyUp(KeyCode.Space))
        {
            SetJointStatePublisherActive(false);
        }
    }

    private void RotateJoint(int jointIndex, int direction)
    {
        if (jointIndex < 0 || jointIndex >= robotJoints.Length) return;

        RobotJoint joint = robotJoints[jointIndex];
        if (joint.jointObject != null)
        {
            // Calculate new rotation value
            float currentRotation = Vector3.Dot(joint.jointObject.transform.localEulerAngles, joint.rotationAxis);
            currentRotation = NormalizeAngle(currentRotation); // normalize to -180 to 180
            float newRotation = currentRotation + joint.rotationStep * direction;
            

            // Apply limits
            if (newRotation >= joint.minRotation && newRotation <= joint.maxRotation)
            {
                Vector3 rotation = joint.rotationAxis * joint.rotationStep * direction;
                joint.jointObject.transform.Rotate(rotation, Space.Self);
                Debug.Log($"Rotated Joint {jointIndex + 1}: {newRotation}");
            }
            else
            {
                Debug.Log($"Joint {jointIndex + 1} rotation limit reached!");
            }
        }
    }
    private void SetJointStatePublisherActive(bool isActive)
    {
        if (jointState_Pub != null)
        {
            jointState_Pub.enabled = isActive;
        }
        else
        {
            Debug.LogError("JointStatePublisher reference is missing!");
        }
    }
    private float NormalizeAngle(float angle)
    {
        angle = Mathf.Repeat(angle + 180f, 360f) - 180f;
        return angle;
    }
}
