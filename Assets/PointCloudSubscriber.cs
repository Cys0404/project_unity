using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class PointCloudSubscriber : MonoBehaviour
{
    // 加上 public，Inspector 才會顯示輸入框
    [Header("ROS Settings")]
    public string topicName = "/camera/depth/color/points"; 

    [Header("Visualization")]
    public ParticleSystem pointCloudParticles; 
    public float pointSize = 0.02f;

    private ParticleSystem.Particle[] particles;
    private Vector3[] pointCloudPoints;
    private bool hasNewData = false;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<PointCloud2Msg>(topicName, PointCloudCallback);
    }

    void PointCloudCallback(PointCloud2Msg message)
    {
        int pointCount = (int)(message.width * message.height);
        if (pointCloudPoints == null || pointCloudPoints.Length != pointCount)
        {
            pointCloudPoints = new Vector3[pointCount];
            particles = new ParticleSystem.Particle[pointCount];
        }

        int pointStep = (int)message.point_step;
        byte[] data = message.data;

        for (int i = 0; i < pointCount; i++)
        {
            int offset = i * pointStep;

            float x = System.BitConverter.ToSingle(data, offset);
            float y = System.BitConverter.ToSingle(data, offset + 4);
            float z = System.BitConverter.ToSingle(data, offset + 8);

            // ROS 座標轉 Unity 座標
            pointCloudPoints[i] = new Vector3(-y, z, x);
        }

        hasNewData = true;
    }

    void Update()
    {
        if (hasNewData && pointCloudParticles != null)
        {
            for (int i = 0; i < pointCloudPoints.Length; i++)
            {
                particles[i].position = pointCloudPoints[i];
                particles[i].startColor = Color.cyan;
                particles[i].startSize = pointSize;
            }

            pointCloudParticles.SetParticles(particles, pointCloudPoints.Length);
            hasNewData = false;
        }
    }
}