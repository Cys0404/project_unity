
# Koch Arm Unity

This Unity project uses **ROS#** to control a real Koch arm.  
**Unity version:** 2022.3.51f1  

## Setup
1. Clone this repo on windows machine
```
git clone git@github.com:hrc-pme/koch_robot_unity.git
```
2. Check folder Assests/model is exist, and the robot model you are using is exist.
if not, check [Google_drive](https://drive.google.com/drive/folders/1_n22OSgRQmWDHSPZfQvmv3m8MH8g9A4N?usp=sharing) for your project resources.  

3. check the dependency is correct
    * Inverse IK -> FinalIK (Assets/plugin/RootMotion)  
    * Package manage -> Nuget (Assets/Nuget)  
    * Pointcloud -> OpenCvSharp4.4.10.0.20241108 (Assets/Packages)  
        ⚠️ warning OpenCvsharp4 will have two folder: OpenCvSharp4.4.10.0.20241108, OpenCvSharp4.runtime.win.4.10.0.20241108 please install with Nuget  
        Press **open_anyway** to open the project and **Nuget > Manage Nuget Packages** to show the packages of nuget  
        **Installed > click all packages > Refresh**, the missing package will download.
---

## Real Robot Repository  

To control the real robot, use this Unity project alongside the real arm control project:  
➡️ [koch_robot_arm](https://github.com/hrc-pme/koch_robot_arm)  

Refer to this repository for setup and running instructions for the real arm.  

---

## Example Usage  

Run the example scenes located in `Scenes > Connect_to_real_example`.  
For detailed instructions, see the **[Example Scene README](https://github.com/hrc-pme/koch_robot_unity/blob/master/Assets/Scenes/Connect_to_real_example)**.

---
