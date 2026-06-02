using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VR_mapping_locobot_HT : MonoBehaviour
{

    public GameObject targetEndEffector;
    private Transform vr_controller;

    public GameObject targetPitch;
    public GameObject targetRoll;
    private float xVRr, zVRr;

    public Transform sourceVRController;
    void Start()
    {
    }

    void Update()
    {
        vr_controller = sourceVRController;
        //Debug.Log(vr_controller.position.x);

        zVRr = vr_controller.eulerAngles.z;
        xVRr = vr_controller.eulerAngles.x;

        targetEndEffector.transform.position = vr_controller.position;
        targetEndEffector.transform.rotation = vr_controller.rotation;

        targetPitch.transform.eulerAngles = new Vector3(xVRr, transform.eulerAngles.y, transform.eulerAngles.z);
        targetRoll.transform.eulerAngles = new Vector3(transform.eulerAngles.x, transform.eulerAngles.y, zVRr);
    }
}
