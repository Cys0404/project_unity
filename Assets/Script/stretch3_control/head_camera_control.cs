using UnityEngine;

public class HeadCameraControl : MonoBehaviour
{
    public Transform cameraObject;
    public Transform panObject;
    public Transform tiltObject;
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

    public AngleLimits panAngleLimits = new AngleLimits(-90f, 90f);
    public AngleLimits tiltAngleLimits = new AngleLimits(-25f, 115f);

    private Vector3 initialPanRotation;
    private Vector3 initialTiltRotation;

    void Start()
    {
        // Store the initial local rotation of the pan and tilt objects
        initialPanRotation = panObject.localRotation.eulerAngles;
        initialTiltRotation = tiltObject.localRotation.eulerAngles;
    }

    void Update()
    {
        if (cameraObject != null && panObject != null && tiltObject != null)
        {
            Quaternion cameraRotation = cameraObject.rotation;
            Vector3 cameraEulerAngles = cameraRotation.eulerAngles;

            // Normalize camera angles
            float rotate_pan = NormalizeAngle(cameraEulerAngles.y);
            float rotate_tilt = NormalizeAngle(cameraEulerAngles.x);

            float clampedPan = Mathf.Clamp(rotate_pan, panAngleLimits.Min, panAngleLimits.Max);
            float clampedTilt = Mathf.Clamp(rotate_tilt, tiltAngleLimits.Min, tiltAngleLimits.Max);
            if (debug_message)
            {
                Debug.Log("Current pan:" + rotate_pan + " Clamp pan:" + clampedPan);
                Debug.Log("Current tilt:" + rotate_tilt + " Clamp tilt:" + clampedTilt);

            }
            // Add camera rotation to the initial rotations
            float targetPanAngle = NormalizeAngle(initialPanRotation.y + clampedPan);
            float targetTiltAngle = NormalizeAngle(initialTiltRotation.x + clampedTilt);

            Quaternion targetPanRotation = Quaternion.Euler(initialPanRotation.x, targetPanAngle, initialPanRotation.z);
            panObject.localRotation = Quaternion.Slerp(panObject.localRotation, targetPanRotation, Time.deltaTime * smoothSpeed);

            Quaternion targetTiltRotation = Quaternion.Euler(targetTiltAngle, initialTiltRotation.y, initialTiltRotation.z);
            tiltObject.localRotation = Quaternion.Slerp(tiltObject.localRotation, targetTiltRotation, Time.deltaTime * smoothSpeed);
        }
    }

    private float NormalizeAngle(float angle)
    {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        return angle;
    }
}
