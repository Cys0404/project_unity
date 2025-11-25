using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using System;
using System.Linq;
using UnityEngine;
using OpenCvSharp;

public class RGBDMerger : MonoBehaviour
{
    public ImageSubscriberPointCloud rgbImageSub;
    public DepthImageSubscriber depthImageSub;

    RosSharp.RosBridgeClient.MessageTypes.Sensor.Image rgbImage;
    RosSharp.RosBridgeClient.MessageTypes.Sensor.Image depthImage;

    Mat rgb_img, depth_img;
    Mat rgb_buff, depth_buff;

    public float fx, fy, cx, cy;
    public int height_start = 0, height_end = 480, width_start = 0, width_end = 640;
    public int depth_limit = 3000;

    public int downsample_factor = 3;
    int i, j, k;

    float x, y, z, r, g, b;

    private List<Vector3> pcl = new List<Vector3>();
    private List<float> dis = new List<float>();
    private List<Color> pcl_color = new List<Color>();

    private Vector3 tmp_vector3;
    private Color tmp_color;

    [Tooltip("Point size multiplier, can be very small like 0.001 for fine detail")]
    [Range(0.001f, 10f)]
    public float point_size_factor = 1f;

    [Tooltip("Base point size for near objects (< 1m)")]
    public float near_point_size = 0.012f;
    [Tooltip("Base point size for medium distance (1-5m)")]
    public float medium_point_size = 0.02f;
    [Tooltip("Base point size for far objects (> 5m)")]
    public float far_point_size = 0.05f;

    [Tooltip("If the depth value is too far, move the point to the depth_limit of the point cloud")]
    public bool move_too_far_points = false;

    // Start is called before the first frame update
    void Start()
    {
        rgbImage = new RosSharp.RosBridgeClient.MessageTypes.Sensor.Image();
        depthImage = new RosSharp.RosBridgeClient.MessageTypes.Sensor.Image();
    }

    // Update is called once per frame
    void Update()
    {
        // Make sure both images have been received
        if (rgbImageSub.ImageData != null && depthImageSub.ImageData != null)
        {
            DecompressImages();
        }
    }

    protected void DecompressRGB()
    {
        rgb_img = Mat.FromImageData(rgbImageSub.ImageData, ImreadModes.Color);
    }

    protected void DecompressDepth()
    {
        depth_img = Mat.FromImageData(depthImageSub.ImageData, ImreadModes.Unchanged);
        depth_img.ConvertTo(depth_img, MatType.CV_16UC1);

    }

    protected void DecompressImages()
    {
        DecompressRGB();
        DecompressDepth();
        PointCloudRendering();
    }

    void PointCloudRendering()
    {
        if (depth_img != null && depth_img.Width > 0 && depth_img.Height > 0)
        { //check if depth image is valid of update FPS is faster than image streaming
            pcl.Clear();
            pcl_color.Clear();
            dis.Clear();

            Mat<Vec3b> rgb_mat = new Mat<Vec3b>(rgb_img);
            MatIndexer<Vec3b> rgb_indexer = rgb_mat.GetIndexer();
            Mat<ushort> depth_mat = new Mat<ushort>(depth_img);
            MatIndexer<ushort> depth_indexer = depth_mat.GetIndexer();

            foreach (int j in Enumerable.Range(height_start, height_end - height_start).Where(number => number % downsample_factor == 0)/*= 0; j < height; j+=2 */ /*j+=batch_size*/)
            {
                foreach (int k in Enumerable.Range(width_start, width_end - width_start).Where(number => number % downsample_factor == 0)/*= 0; k < width; k+=2 */ /*k+=batch_size*/)
                {
                    try
                    {
                        if (j >= 0 && j < depth_img.Height && k >= 0 && k < depth_img.Width)
                        {
                            if (depth_indexer[j, k] > depth_limit)
                            {
                                continue;
                            }
                            if (depth_indexer[j, k] == 0 && move_too_far_points)
                            {
                                depth_indexer[j, k] = (ushort)depth_limit;//move the too far point(no depth info) to the depth_limit of the point cloud.
                            }
                            else if (depth_indexer[j, k] == 0 && !move_too_far_points)//skip the point if it is too far from camera phyiscally.
                            {
                                continue;
                            }


                            z = depth_indexer[j, k] * 0.001f;
                            x = (k - cx) / fx * z;
                            y = (j - cy) / fy * z;

                            r = (float)rgb_indexer[j, k][2] / 255.0f;
                            g = (float)rgb_indexer[j, k][1] / 255.0f;
                            b = (float)rgb_indexer[j, k][0] / 255.0f;

                            tmp_vector3 = new Vector3(x, z, y);
                            tmp_color = new Color(r, g, b);
                            if (depth_indexer[j, k] < 1000)
                            {
                                tmp_color.a = near_point_size * point_size_factor;
                            }
                            else if (depth_indexer[j, k] >= 1000 && depth_indexer[j, k] <= 5000)
                            {
                                tmp_color.a = medium_point_size * point_size_factor;
                            }
                            else
                            {
                                tmp_color.a = far_point_size * point_size_factor;
                            }
                            pcl.Add(tmp_vector3);

                            pcl_color.Add(tmp_color);


                        }
                        else
                        {
                            Debug.LogError($"Index out of range: j={j}, k={k}");
                        }
                    }
                    catch (ArgumentOutOfRangeException e)
                    {
                        // handle error: log details and take corrective action
                        Debug.LogError("Out of range access: " + e.Message);
                    }
                }
            }
        }
        else
        {
            return;
        }

    }

    public Vector3[] GetPCL(int index)
    {
        return pcl.ToArray();
    }

    public Color[] GetPCLColor(int index)
    {
        return pcl_color.ToArray();
    }

    public float[] GetPCLDis(int index)
    {
        return dis.ToArray();
    }

}
