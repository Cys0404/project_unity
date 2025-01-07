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

    private float xVRr, yVRr, zVRr;

    void Start()
    {
        // 確保 Control_source 已被指定
        if (Control_source == null)
        {
            Debug.LogError("Control_source 尚未被指定！");
        }
        if (targetPitch == null)
        {
            Debug.LogError("targetPitch 尚未被指定！");
        }
        if (targetRoll == null)
        {
            Debug.LogError("targetRoll 尚未被指定！");
        }
    }

    void Update()
    {
        if (Control_source == null)
        {
            Debug.LogWarning("Control_source 未被設定，跳過更新。");
            return;
        }

        vr_controller = Control_source.transform;

        // 獲取手腕的 Euler 角度
        Vector3 vrEuler = vr_controller.eulerAngles;
        xVRr = vrEuler.x;
        yVRr = vrEuler.y;
        zVRr = vrEuler.z;

        // 調試輸出，確認手腕的旋轉數據
        Debug.Log($"R_Wrist Euler Angles: X={xVRr}, Y={yVRr}, Z={zVRr}");

        // 更新 End Effector 的旋轉
        targetEndEffector.transform.rotation = vr_controller.rotation;

        // 更新 End Effector 的位置
        targetEndEffector.transform.position = new Vector3(
            vr_controller.position.x,
            vr_controller.position.y,
            vr_controller.position.z * 35 / user_arm_length
        );

        // 更新 Target_Pitch 的旋轉
        targetPitch.transform.rotation = Quaternion.Euler(
            targetPitch.transform.eulerAngles.x,
            targetPitch.transform.eulerAngles.y,
            xVRr - 90
        );

        // 更新 Target_Roll 的旋轉
        targetRoll.transform.rotation = Quaternion.Euler(
            targetRoll.transform.eulerAngles.x,
            targetRoll.transform.eulerAngles.y,
            zVRr - 90
        );

        // 調試輸出，確認 Target_Roll 的旋轉
        Debug.Log($"Target_Roll Rotation: {targetRoll.transform.rotation.eulerAngles}");
    }
}
