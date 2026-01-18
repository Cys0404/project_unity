using System.Collections;
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using RosSharp.RosBridgeClient;

public class RGBDRender : MonoBehaviour
{
    public RGBDMerger subscriber;
    // Mesh stores the positions and colours of every point in the cloud
    // The renderer and filter are used to display it
    Mesh mesh;
    MeshRenderer meshRenderer;
    MeshFilter mf;

    // The size, positions and colours of each of the pointcloud
    // public float pointSize = 1f;
    // public float radius = 0.1f;

    [Header("MAKE SURE THESE LISTS ARE MINIMISED OR EDITOR WILL CRASH")]
    private Vector3[] positions = new Vector3[] { new Vector3(0, 0, 0) };
    private Color[] colours = new Color[] { new Color(1f, 1f, 1f) };
    // private float[] dis = new float[] {0.0f}; 
    public Transform offset; // Put any gameobject that faciliatates adjusting the origin of the pointcloud in VR. 

    void Start()
    {
        // Give all the required components to the gameObject
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        mesh = new Mesh();
        mf = gameObject.AddComponent<MeshFilter>();
        meshRenderer.material = new Material(Shader.Find("Custom/PointCloudGeometryShader"));
        // Use 32 bit integer values for the mesh, allows for stupid amount of vertices (2,147,483,647 I think?)
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        transform.position = offset.position;
        transform.rotation = offset.rotation;

    }

    void UpdateMesh()
    {

        mesh.Clear();
        //positions = subscriber.pcl;
        positions = subscriber.GetPCL(3);
        colours = subscriber.GetPCLColor(3);
        // Debug.Log("Positions: " + positions.Length);
        // Debug.Log("Colours: " + colours.Length);
        if (positions == null || colours == null)
        {
            return;
        }

        mesh.vertices = positions;
        mesh.colors = colours;

        int[] indices = new int[positions.Length];
        for (int i = 0; i < indices.Length; i++)
        {
            indices[i] = i; // Direct mapping
        }
        mesh.SetIndices(indices, MeshTopology.Points, 0); // Use MeshTopology.Points

        mf.mesh = mesh;
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = offset.position;
        transform.rotation = offset.rotation;
        //meshRenderer.material.SetFloat("_PointSize", pointSize);

        // meshRenderer.material.SetFloat("_PointSize", pointSize);
        UpdateMesh();
        // Debug.Log(DateTime.Now.ToString("HH:mm:ss.ffffff"));
    }
}
