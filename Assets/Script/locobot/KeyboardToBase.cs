using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class KeyboardToBase : UnityPublisher<MessageTypes.Geometry.Twist>
    {
        [Header("Movement Settings")]
        [Tooltip("前/後直線速度（m/s）")]
        public float linearSpeed = 0.25f;

        [Tooltip("旋轉角速度（rad/s）")]
        public float angularSpeed = 0.60f;

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

            if (Input.GetKey(KeyCode.UpArrow))
                message.linear.x = linearSpeed;
            else if (Input.GetKey(KeyCode.DownArrow))
                message.linear.x = -linearSpeed;

            if (Input.GetKey(KeyCode.LeftArrow))
                message.angular.z = angularSpeed;
            else if (Input.GetKey(KeyCode.RightArrow))
                message.angular.z = -angularSpeed;

            if (debug_message)
            {
                Debug.Log($"[KeyboardToBase] topic={Topic} | linX={message.linear.x:F2} | angZ={message.angular.z:F2}");
            }

            Publish(message);
        }
    }
}