using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class KeyboardToBase : MonoBehaviour
{
    [Header("ROS-TCP 設定")]
    public string topicName = "/locobot/commands/velocity";

    [Header("Movement Settings")]
    public float linearSpeed = 0.25f;
    public float angularSpeed = 0.6f;

    [Header("Debug")]
    public bool Debug_message = false;

    private ROSConnection ros;
    private TwistMsg message;
    private float handLinear = 0f;
    private float handAngular = 0f;

    void Start()
    {
        // 1. 取得 ROS-TCP 連線單例
        ros = ROSConnection.GetOrCreateInstance();
        
        // 2. 向 ROS 註冊此 Topic 的 Publisher
        ros.RegisterPublisher<TwistMsg>(topicName);

        // 3. 初始化 Twist 訊息
        message = new TwistMsg();
    }

    void Update()
    {
        // 優先讀取手勢速度 (由 XRHandThumbsUpDrive 傳入)
        if (handLinear != 0f || handAngular != 0f)
        {
            message.linear.x = handLinear;
            message.angular.z = handAngular;
            ros.Publish(topicName, message);
            return;
        }

        // 鍵盤備案操控 (WASD)
        float lx = Input.GetAxis("Vertical") * linearSpeed;
        float az = -Input.GetAxis("Horizontal") * angularSpeed;

        if (lx != 0f || az != 0f)
        {
            message.linear.x = lx;
            message.angular.z = az;
            ros.Publish(topicName, message);
        }
    }

    // 提供給 XRHandThumbsUpDrive 手勢腳本呼叫的介面
    public void SetHandVelocity(float linear, float angular)
    {
        handLinear = linear;
        handAngular = angular;
    }
}