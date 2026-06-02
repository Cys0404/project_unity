using UnityEngine;
using UnityEngine.Serialization;

namespace RosSharp.RosBridgeClient
{
    public class keyboardtobase : UnityPublisher<MessageTypes.Geometry.Twist>
    {
        [Header("Movement Settings")]
        [FormerlySerializedAs("max_speed")]
        [Tooltip("Forward/backward speed (m/s)")]
        public float linearSpeed = 0.25f;

        [Tooltip("Turning speed (rad/s)")]
        public float angularSpeed = 0.60f;

        [Header("Key Bindings")]
        public KeyCode forwardKey = KeyCode.W;
        public KeyCode backwardKey = KeyCode.S;
        public KeyCode leftKey = KeyCode.A;
        public KeyCode rightKey = KeyCode.D;

        [Header("Debug")]
        public bool debug_message = false;

        private MessageTypes.Geometry.Twist message;

        protected override void Start()
        {
            base.Start();
            message = new MessageTypes.Geometry.Twist
            {
                linear = new MessageTypes.Geometry.Vector3(),
                angular = new MessageTypes.Geometry.Vector3()
            };
        }

        private void Update()
        {
            message.linear.x = 0f;
            message.angular.z = 0f;

            bool forward = Input.GetKey(forwardKey) || Input.GetKey(KeyCode.UpArrow);
            bool backward = Input.GetKey(backwardKey) || Input.GetKey(KeyCode.DownArrow);
            bool turnLeft = Input.GetKey(leftKey) || Input.GetKey(KeyCode.LeftArrow);
            bool turnRight = Input.GetKey(rightKey) || Input.GetKey(KeyCode.RightArrow);

            if (forward && !backward)
                message.linear.x = linearSpeed;
            else if (backward && !forward)
                message.linear.x = -linearSpeed;

            if (turnLeft && !turnRight)
                message.angular.z = angularSpeed;
            else if (turnRight && !turnLeft)
                message.angular.z = -angularSpeed;

            if (debug_message)
            {
                Debug.Log($"[keyboardtobase] topic={Topic} | linX={message.linear.x:F2} m/s | angZ={message.angular.z:F2} rad/s");
            }

            Publish(message);
        }
    }
}
