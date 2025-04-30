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
    private GameObject targetGripper;
    public GameObject Control_source; // VR controller or hand
    public GameObject IndexProximal;
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

        // 獲取手指root的 Euler 角度
        float xINDEXr = IndexProximal.transform.eulerAngles.x;



        // 調試輸出，確認手腕的旋轉數據
        //Debug.Log($"R_Wrist Euler Angles: X={xVRr}, Y={yVRr}, Z={zVRr}");

        // 更新 End Effector 的旋轉
        targetEndEffector.transform.rotation = vr_controller.rotation;

        // 更新 End Effector 的位置
        targetEndEffector.transform.position = new Vector3(
            vr_controller.position.x,
            vr_controller.position.y,
            vr_controller.position.z * 35 / user_arm_length
        );
        //對z軸單獨做mapping是因為手的長度是在z軸上，若是koch arm和手臂在同一水平線上，則將手打直只有z軸的長度會不一致。


        //_____________________以下為pitch的範圍___________________

        // 計算旋轉角度
        //float pitchAngle = xVRr - 90;

        //// 加入限制
        //if (pitchAngle < 210 && pitchAngle > 190 )
        //{
        //    pitchAngle = 210; // 如果小於 210 度，且大於 190 度，則設置為 210 度
        //}
        //Debug.Log($"pitchAngle: {pitchAngle}");

        // Note : 一律用Debug.Log檢查轉角輸出的值，直接看inspector不準確。

        // 更新 Target_Pitch 的旋轉


        ////_________test_________________
        float PitchAngle_targetPitch = xVRr - 90;  // 拿到R_Wrist的x軸的旋轉值後對offset做修正
        //Debug.Log($"PitchAngle_targetPitch: {PitchAngle_targetPitch}");

        //Quaternion targetPitchRotation = Quaternion.AngleAxis(PitchAngle_targetPitch, Vector3.forward);

        //targetPitch.transform.localRotation = targetPitchRotation;
        ////________test_end_____________



        targetPitch.transform.rotation = Quaternion.Euler(
            targetPitch.transform.eulerAngles.x,
            targetPitch.transform.eulerAngles.y,
            PitchAngle_targetPitch
        );


        //_____________________以下為roll的範圍___________________


        // 繞「本地 X 軸」轉 rollAngle 度
        float rollAngle_tagertPitch = -zVRr + 90;  // 拿到R_Wrist的z軸的旋轉值後對offset做修正

        //// 加入限制，因為pitch到超過一定的值後roll會一起旋轉。
        //if (rollAngle < 70 && rollAngle > -90)
        //{
        //    rollAngle = 70;
        //}

        Quaternion rollRotation = Quaternion.AngleAxis(rollAngle_tagertPitch, Vector3.right);

        // 直接設定給 localRotation (用localEulerAngles會遇到萬向鎖的問題，超麻煩)
        targetRoll.transform.localRotation = rollRotation;


        // 調試輸出，確認 Target_Roll 的旋轉
        // Debug.Log($"Target_Roll Rotation: {targetRoll.transform.rotation.eulerAngles}");
        // Debug.Log($"rollAngle: {rollAngle_tagertPitch}");


        //_____________________以下為gripper的範圍___________________

        float PitchAngle_targetGripper = xINDEXr - 90;  // 拿到R_IndexProximal的x軸的旋轉值後對offset做修正
        Debug.Log($"PitchAngle_tagertGripper: {PitchAngle_targetGripper}");

        Quaternion PitchRotation = Quaternion.AngleAxis(PitchAngle_targetGripper, Vector3.right);

        targetGripper.transform.localRotation = PitchRotation;



    }
}
