using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VR_mapping_koch : MonoBehaviour
{
    public GameObject targetEndEffector;
    private Transform vr_controller;
    public GameObject targetPitch;
    public GameObject targetRoll;
    public GameObject Control_source; // VR controller or hand
    public float user_arm_length = 77; // Input user arm length to modify the mapping of human arm to robot arm 

    private float xVRr, zVRr;
    void Start()
    {
       
    }

    void Update()
    {
        vr_controller = Control_source.transform;
        Debug.Log(vr_controller.position.x);

        zVRr = vr_controller.eulerAngles.z;
        xVRr = vr_controller.eulerAngles.x;

        targetEndEffector.transform.rotation = vr_controller.rotation;

        targetEndEffector.transform.position = new Vector3(
            vr_controller.position.x ,
            vr_controller.position.y ,
            vr_controller.position.z * 35 / user_arm_length
        );


        targetPitch.transform.rotation = Quaternion.Euler(targetPitch.transform.eulerAngles.x, targetPitch.transform.eulerAngles.y, xVRr - 90);
        targetRoll.transform.rotation = Quaternion.Euler(zVRr, targetRoll.transform.eulerAngles.y, targetRoll.transform.eulerAngles.z);
        
        // targetPitch.transform.eulerAngles = new Vector3(xVRr, transform.eulerAngles.y, transform.eulerAngles.z);
        // targetRoll.transform.eulerAngles = new Vector3(transform.eulerAngles.x, transform.eulerAngles.y, zVRr-90);
    }

}
