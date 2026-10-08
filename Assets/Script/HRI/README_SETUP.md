# Unity：HRI pure RGB delta 操作與擺放

目標場景是 `Assets/Scenes/sunnyxi/sunntxi——recon 1.unity`。本次交付腳本及 **HRI > Configure pure RGB delta scene** 編輯器設定工具。工具不進入Play、不啟動Robot；必須完成下述實體外參與場景綁定，再儲存場景。原scene YAML未自動覆寫。

## 先理解三個空間

```text
Scene（所有根物件world scale=1）
├─ reconstruction / worldAdjust        ← 原MASt3R basis保留；只這裡調scale
│  └─ Mast3rKeyframeDeltaViewer + RuntimePointCloudCollider
├─ stretch3_real                       ← 真實回授URDF，保持米制
│  └─ base / lift / arm / head / wrist
│     ├─ camera_color_optical_frame     ← 可動頭相機
│     ├─ FixedD455Optical               ← 實測固定D455外參
│     ├─ LeftD415Optical / RightD415Optical
│     └─ gripper_camera_color_optical_frame
├─ Stretch3 command rig                ← 原手勢的命令模型，不是回授模型
├─ Stationary Teleop Tracking Space    ← 固定控制校準座標，不掛在avatar/XR下
├─ PlayerArmature
│  └─ AvatarEye                        ← 頭/眼高度，建議初始約1.6m
├─ XR Origin                          ← 唯一XR rig，不掛在URDF或縮放map下
│  └─ Camera Offset / Main Camera      ← 保持TrackedPoseDriver
└─ HRI System                         ← 工具建立的模式/菜單/尺度/RGBD控制
```

`stretch3_real` 位置來自 MASt3R **camera** pose，再扣除 D455→robot 的實際外參；不能把 camera軌跡直接當base_link。調scale時map和camera軌跡位置改變，URDF、D405、D415、手臂長度保持真實米制。不要把整個URDF放進worldAdjust，否則尺寸會被一起放大。

## 1. 保留現有avatar和小地圖

打開 `sunntxi——recon 1`，保留 PlayerArmature。主 `reconstruction` 的 regular `Mast3rKeyframeDeltaViewer` 使用 `/koch/mast3r/delta/keyframe_metadata`、`/koch/mast3r/delta/keyframe_cloud_local`、`/koch/mast3r/camera_trajectory`。設定工具會綁回同物件的RuntimePointCloudCollider，將 `controlledRobot` 指定為PlayerArmature，接回既有導航/軌跡回放。

原小地圖地面/點選導航/高度切片/紅色軌跡/往返回放保留。VR Map頁使用同一張小地圖texture及同一套導航，點選是讓 **avatar導航到目的地**，不是讓實體Robot走過去。

## 2. 帶入既有Stretch3和XR Hands

從 `Assets/Scenes/Dual_arm_hand_tracking/stretch3_only.unity` 複製你原本可工作的 Stretch3 組及XR Hands rig到目標scene，可先建立prefab再拖入。場景有兩組同名connector，應選 `ws://192.168.0.60:9090` 那組；不要複製所有其他機器人的connector。

保留 `stretch3_real_connector` 的 `/stretch/joint_states` JointStateSubscriber以及它的JointNames/JointStateWriters綁定，這才是現實關節回授。控制用 `stretch3_control_connector` 的JointStatePublisher、HandToBase、HandToGripper及手臂驅動綁定留在獨立command rig；不要讓手勢driver和回授subscriber寫同一套可見關節。

回授模型根位置交給 `HriRobotMapAnchor`，移除/停用同根 `/pose`、Odometry或其他根Transform writer。可有一個control connector加一個state connector，兩個都連同一Robot；每種control topic只准一個publisher。工具只會更改你指定的RosConnector URL，另一個請在Inspector核對。

只保留一套XR Origin與HMD Camera。複製過來的舊robot相機只作參考anchor，不另開XR Camera/AudioListener。停用原desktop Main Camera、Cinemachine virtual camera/brain對XR的控制。

## 3. 先把Stretch3與XR物件放進目標Scene

### 3.1 開兩個Scene並複製正確的Stretch3

1. 在Project視窗打開 `Assets/Scenes/sunnyxi/sunntxi——recon 1.unity`。
2. 再用Additive方式打開 `Assets/Scenes/Dual_arm_hand_tracking/stretch3_only.unity`：在Project視窗對scene按右鍵，選 **Open Scene Additive**。
3. 在 `stretch3_only` 的Hierarchy中，複製你截圖下方那個根物件 **`stretch3`**。它下面應看得到 `Stretch3_robot_base/control`、`stretch3_transparent`、`base_link`、`stretch3_control_connector` 和 `real`。
4. 把複製出的 `stretch3` 拖到 `sunntxi——recon 1` scene的根層級。可以複製這一組；不要把上方整個 `old_stuff/XR Origin` 一起複製過來。
5. 複製完成後，在Hierarchy對 `sunntxi——recon 1` 的scene標題按右鍵選 **Set Active Scene**。設定工具也會拒絕任何仍指向來源scene的欄位，避免改到來源scene或別台Robot。
6. 展開新複製的 `stretch3`，逐個找 `RosConnector` component。控制與回授connector都改成 **`ws://192.168.0.60:9090`**。注意這只適用ROS#／rosbridge；不要把Raw RGBD或Unity Robotics ROS-TCP也改成9090。
7. 確認可見的真實回授模型是 `real` 那一支；控制用透明/命令模型與 `real` 必須是不同Transform，不可讓publisher與subscriber同時寫同一組關節。

如果目標scene已經有一套可工作的XR Origin和左右手追蹤，保留它，不要複製第二套。若完全沒有XR rig，才從原scene只複製一套 `XR Origin`、`Left Hand Tracking`、`Right Hand Tracking`；最後Hierarchy中只能有一個啟用的HMD Camera及一個AudioListener。

### 3.2 建立AvatarEye、RobotEye與BaseHeading

以下都用Hierarchy右鍵 **Create Empty** 建立；建立後先將local Position設 `(0,0,0)`、local Rotation設 `(0,0,0)`、Scale設 `(1,1,1)`，再依位置調整。

1. 在 `PlayerArmature` 的頭／眼睛骨架下建立 `AvatarEye`，把它移到兩眼中央。若Avatar沒有頭骨，就直接放在PlayerArmature下，世界高度先用約1.6 m。
2. 在Stretch3回授模型的可動頭相機支架下建立 `RobotEye`，位置放在實際頭相機鏡頭中心。它只提供第一視角的位置；不要把它掛到worldAdjust。
3. 在回授模型的 `base_link` 下建立 `BaseHeading`。令此物件的Unity **藍色+Z箭頭指向機器人正前方，綠色+Y朝上**。
4. `BaseHeading` 不可放在pan/tilt關節下，否則你轉頭時機器人第一視角會被重複旋轉。

### 3.3 建立四個相機光學frame

1. 找到回授模型中實際會跟頭部pan/tilt移動的相機frame。若已有 `camera_color_optical_frame`，直接使用它作 **Head optical frame**，不要再建一顆假frame。
2. 在固定D455的實際支架父物件下建立 `FixedD455Optical`。
3. 在左D415支架下建立 `LeftD415Optical`；在右D415支架下建立 `RightD415Optical`。
4. 依實測外參填三個物件的local Position與Rotation。Position單位是公尺，例如10 cm填`0.10`，不是`10`。
5. Scene視窗切到Local座標，逐顆檢查原VRRGBD約定：**X右、Y前、Z下**。若你的frame採一般Unity Camera約定（Z前、Y上），四顆都必須一致，並在稍後把 `HriRgbdViews.urdfOpticalYForward` 與 `HriRobotMapAnchor.urdfOpticalYForward` 一起關掉。
6. 固定D455必須是送入MASt3R的 `mast3r_camera`，不是可動頭部相機。未量測外參不要用目測數字冒充校正值。

完成後，建議Hierarchy大致如下：

```text
sunntxi——recon 1
├─ reconstruction/worldAdjust            ← 地圖可縮放
├─ PlayerArmature/AvatarEye
├─ XR Origin/Camera Offset/Main Camera    ← 唯一XR相機
└─ stretch3
   ├─ Stretch3_robot_base/control         ← 命令模型
   ├─ stretch3_control_connector          ← ws://192.168.0.60:9090
   └─ real                                ← 回授URDF，保持1公尺制
      └─ base_link
         ├─ BaseHeading
         ├─ FixedD455Optical
         ├─ LeftD415Optical
         ├─ RightD415Optical
         └─ ...pan/tilt.../camera_color_optical_frame
            └─ RobotEye
```

## 4. 一格一格填HRI設定工具

1. 先Save Scene，然後點Unity上方選單 **HRI > Configure pure RGB delta scene**。
2. 若看不到HRI選單，先打開Console修完所有紅色編譯錯誤；Editor腳本只有編譯成功才會出現。
3. 按下列順序從Hierarchy拖入欄位：

| 工具欄位 | 要拖入的物件 | 不要拖入 |
|---|---|---|
| Regular delta viewer | `reconstruction`中訂閱`/koch/mast3r/delta/...`的Mast3rKeyframeDeltaViewer | compare/full viewer |
| XR Origin | 唯一XR Origin根物件 | Main Camera |
| Tracked HMD Camera | XR Origin下有Tracked Pose Driver的Main Camera | 桌面相機、RobotEye |
| Metric Stretch3 root | `stretch3/real`回授URDF根 | 透明control rig、整個stretch3父物件 |
| Robot view anchor | `RobotEye` | 相機模型mesh |
| Base heading (+Z forward) | `BaseHeading` | pan/tilt link |
| PlayerArmature root | `PlayerArmature`根 | AvatarEye |
| PlayerArmature eye anchor | `AvatarEye` | HMD Main Camera |
| Single Robot RosConnector | `.60:9090`且包含Stretch3控制publisher的connector | `.212`或其他機器人connector |
| Head optical frame | 可動頭相機`camera_color_optical_frame` | RobotEye |
| Fixed D455 optical frame | `FixedD455Optical` | 頭相機frame |
| Left/Right D415 optical frame | `LeftD415Optical`／`RightD415Optical` | 左右手追蹤物件 |

4. `Robot host/IP`填 `192.168.0.60`，`Thor host/IP`填 `192.168.0.30`。
5. 確認所有欄位非None後按 **Create HRI objects (Undo supported)**。按鈕是灰色代表至少一欄沒填或Hierarchy不合法；看工具上方提示逐項補齊。
6. 成功後Hierarchy根層會出現 `HRI System` 與 `Stationary Teleop Tracking Space`。立刻按 `Ctrl+S` 儲存scene。
7. 若提示Already configured，不要再按出第二套。選現有 `HRI System`修改；若確定要重建，先備份scene，再刪除舊HRI System和它建立的Stationary Teleop Tracking Space，重新跑工具一次。
8. 設定完成後關閉Additive開啟的 `stretch3_only` 來源scene，選 **Remove Scene**，不要Save來源scene。進Play前Hierarchy中只保留目標scene；工具也會在ROSConnection或外參subscriber重複時拒絕建立。

工具會把Unity Robotics的唯一ROS-TCP endpoint設為 **`192.168.0.30:10003`**。三相機外參由Robot先發布，再由Thor domain bridge轉發到這個endpoint；因此不要再把ROSConnection改到Robot `:10000`，否則MASt3R地圖topic會斷線。Raw RGBD仍直接使用Robot `192.168.0.60:50010`，控制仍使用`ws://192.168.0.60:9090`。

## 5. 建立後逐個檢查Inspector掛件

選中 `HRI System`，由上到下檢查：

1. **HriInteractionModeController**：`Allow Robot Commands`先保持關閉。工具只會從你指定RosConnector所在的最上層Stretch3根物件內自動收集，絕不掃描其他Scene。Teleoperation Behaviours應只包含Stretch3的HandToBase、HandToGripper、HandToShoulder、HandToWrist及真正的JointStatePublisher；若手勢腳本掛在XR Hands而不在Stretch3根內，請逐一手動拖入。不要加入來源scene、別台機器人或重複publisher。Head Control Behaviours只放目標Stretch3的HeadCameraControl。
2. **HriHandMenuInput**：Tracking Space拖入左右手pose共同使用的座標根，一般是`XR Origin/Camera Offset`，不是Main Camera。
3. **HriFloatingMenu**：Modes、Map、Replay、Scene Scale、Camera Extrinsics都不得是None。Camera Extrinsics必須指同一HRI System上的`CameraExtrinsicsSubscriber`。
4. **HriSceneScaleController**：Viewer指regular delta viewer，Map Adjustment指原`worldAdjust`。不要把URDF根拖進Map Adjustment。
5. **HriRobotMapAnchor**：Robot Root是`real`回授URDF，Slam Camera Optical Frame是`FixedD455Optical`。
6. **HriRgbdViews**：四個optical frame逐項正確；Layer 28/29沒有被其他物件使用。想調畫面2觀看角度，可在D455附近建空物件`SurroundViewAnchor`，旋轉成鳥瞰角度後拖入。
7. **RgbdTcpReceiver**：Host=`192.168.0.60`、Port=`50010`。整個scene只留一個啟用的receiver。
8. **RgbdCameraManager**：Extrinsics Apply Mode必須是`Manual`。Reference Camera ID運行時為`mast3r_camera`，其他固定兩顆為`camera_left`、`camera_right`。
9. **CameraExtrinsicsSubscriber**：Topic=`/unity/camera_extrinsics`、Camera Manager指上一步manager、Subscribe On Start勾選。不要開任何自動Apply腳本。
10. **HriWristRgbSubscriber**在RosConnector物件上，topic應是`/gripper_camera/gripper_camera/color/image_rect_raw/compressed`，Menu指HRI Floating Menu。

Main HMD Camera的Culling Mask要包含Layer 28和29，才能在世界中看到即時點雲；HRI生成的兩顆Render Camera沒有AudioListener且不是XR stereo。`HeadCameraControl.cameraObject`要指Tracked HMD Camera，panObject／tiltObject仍指原命令rig關節。

## 6. 第一次Play：只看，不控制Robot

1. Robot/Thor先按各端啟動文件啟動影像、bridge、SLAM與endpoint；不要開Unity的`Allow Robot Commands`。
2. 按Play後看Console，確認沒有NullReference、重複subscriber或連線反覆建立。
3. HRI System的RgbdTcpReceiver應顯示連線；RgbdCameraManager的FPS應看到`mast3r_camera`、`camera_left`、`camera_right`、`head_camera`。
4. CameraExtrinsicsSubscriber的`Received Messages`應增加。此時三相機外參只是pending，點雲不會自動跳動。
5. 桌面先按`M`開菜單。選 **Scene / Alignment > Three-camera point-cloud alignment**，確認狀態能看到topic、收到數量及最後camera ID。
6. 按 **Apply latest ROS extrinsics once**一次。效果等同Inspector的 **Apply Pending Manual Extrinsics**：把目前pending的D455參考系與左右D415外參套用一次，之後新ROS訊息仍只暫存，不會自動再套用。
7. 若要再次校正，等`Received`數字增加且相機姿態穩定後，再手按一次。不要連續快速按；先觀察三份點雲是否落在同一牆面／桌面。

## 7. VR菜單如何操作

- `Space`：Robot第一視角與Avatar第一視角切換。
- `M`：桌面測試開／關菜單。
- VR召喚：左手張開、掌心朝眼睛約0.65秒；手先放下再舉起才會再次觸發。
- 確認：左右任一手的射線從掌心射出，方向由掌心到中指根部決定，因此拇指與食指捏合時不會改變瞄準方向。射線命中菜單時停在菜單表面；拇指和食指先放開、再輕捏一次才點擊。雙手同時捏合時每幀只接受一次點擊，避免按鈕重複執行。
- Map：點小地圖只移動Avatar，不會命令實體Robot；保留Follow trajectory / Return與Stop avatar。
- Views：畫面1頭部點雲、畫面2固定D455+左右D415、畫面3 D405 RGB。
- Scene / Alignment共有兩個子選項：`Scene scale alignment`只改MASt3R地圖與軌跡比例；`Three-camera point-cloud alignment`只在手按時套用最新ROS外參。

在菜單捏合期間會暫停Robot手勢，避免按UI同時發運動命令。Avatar頭部始終跟HMD轉動；Views頁仍可使用同一組Robot／手臂手勢，但只有你明確打開`Allow Robot Commands`後publisher才會啟用。失去app focus後會自動關閉，需要人工再開。

## 8. 尺度對齊與三相機對齊不要混淆

`Scene scale alignment`的±10%與Reset只縮放MASt3R map和trajectory；Stretch3 URDF、手臂長度與RGBD相機實體間距維持公尺。精確值可在HriSceneScaleController填`Requested Scale`，從component選單執行`Apply Inspector Scale`。Known Distance及Three Corresponding Landmarks也是手動工具，不是自動ICP。

`Three-camera point-cloud alignment`不改場景scale，只套用Robot送來的D455↔D415外參。D455是reference，左右D415移到其座標系。看到三份雲整體方向一致但大小與MASt3R大場景不一致時，應調Scene scale；三份RGBD彼此錯位時才按三相機對齊。

只讓頭部相機看到手臂AprilTag，若沒有tag在SLAM map中的對應點，不能單獨解出MASt3R比例。全局scale也不能修復SLAM局部漂移。

## 9. 正式整套系統仍需你驗證

先在命令OFF狀態確認：三畫面、MASt3R紅色軌跡、URDF關節回授、Space切換、頭部視角、掌心開菜單、pinch按鈕、scale改變時URDF尺寸不變、外參只有按鈕時才套用。再確認只有一組`/stretch/cmd_vel`、`/stretch/joint_states_control`、`/stretch/gripper_control` publisher，並核對實體左右、前後、pan/tilt正負。最後才由你在VS Code正式啟動全系統並打開`Allow Robot Commands`做硬體測試。

## 10. HRI System Inspector欄位完整說明

### XR Hands看不到／手勢沒有反應的快速檢查

1. Play後選`HRI System > Hri Hand Menu Input`，看`Subsystem Status`：
   - `No XRHandSubsystem discovered`：目前build target或OpenXR runtime沒有提供XR Hands。
   - `Found ... but none is running`：XR Hands package存在，但XR loader/runtime沒有真正啟動。
   - `XRHandSubsystem is running`但Left/Right Hand Tracked為OFF：頭顯沒有追蹤到裸手；放下控制器、確認頭顯手部追蹤與光線。
2. `XR Origin (stretch3) > Camera Offset`下必須保留`Left Hand Tracking`、`Right Hand Tracking`與`Hand Visualizer`；Hand Visualizer的Draw Meshes須為ON。目前`sunntxi——recon 1`已經具備這些物件。
3. PC/Quest Link在`Project Settings > XR Plug-in Management > OpenXR > Standalone`確認`Hand Tracking Subsystem`與`Meta Hand Tracking Aim`為ON，且Quest Link runtime本身允許hand tracking。
   - Meta Quest Link不要啟用`Microsoft Hand Interaction Profile`；它會送出`/interaction_profiles/microsoft/hand_interaction`，在Meta runtime可能得到`XR_ERROR_HANDLE_INVALID`。目前專案Standalone已將它關閉。
4. 若直接Build到Quest，在Android頁同時開`Initialize XR on Startup`、OpenXR loader、`Hand Tracking Subsystem`與`Meta Hand Tracking Aim`。目前專案Android設定預設沒有啟動，未調整前APK不會有XR Hands。
5. 先在Avatar模式按鍵盤`M`測試菜單本體；M能開而掌心不能開，表示UI正常，問題只在XR Hands追蹤。`Allow Robot Commands`只控制是否發布機器人指令，不影響手模型顯示或菜單手勢。

菜單開啟後，左右手都會從掌心顯示射線，瞄準方向使用掌心到中指根部，不受拇指與食指捏合動作影響。藍色為一般指向、綠色表示射線位於菜單內、黃色表示正在捏合；命中菜單時射線只畫到菜單表面，不會穿過菜單。任一手先鬆開拇指與食指，再輕捏一次即可點選。可在`Hri Hand Menu Input > Two-hand menu rays`調整Ray Length、Ray Width與三種顏色；不需要加入XR Ray Interactor，避免同一次捏合被兩套UI重複處理。

### 10.1 Hri Interaction Mode Controller

| 欄位 | 用途／怎麼調 |
|---|---|
| XR Origin | 唯一XR rig根；不要拖Main Camera。 |
| Tracked Camera | 有HMD tracked pose的Main Camera。 |
| Robot Eye Anchor | Robot第一視角的位置錨點。只調位置；頭轉由HMD／HeadCameraControl處理。 |
| Robot Heading Anchor | Robot底座朝向；Unity +Z必須指機器人前方，不可跟pan/tilt轉。 |
| Avatar Eye Anchor | Avatar第一視角眼睛位置。 |
| Teleoperation Behaviours | 允許控制時才啟用的Stretch3手勢driver與publisher。漏放會無法控制；重複放會重複發命令。 |
| Head Control Behaviours | 頭部pan/tilt控制腳本。 |
| Conflicting View Controllers | 正常由工具清空；舊視角writer不可與HRI同時運作。 |
| Allow Robot Commands | 安全總開關。視覺測試保持OFF；只有正式有人監督時才開。 |
| Initial Mode | `Avatar`建議值；Play後預設Avatar第一視角。 |

### 10.2 Hri Hand Menu Input

| 欄位 | 用途／建議 |
|---|---|
| Modes / Menu | 必須指同一HRI System上的兩個component。 |
| Tracking Space | 一般拖`XR Origin/Camera Offset`。 |
| Palm Hold Seconds | 掌心召喚菜單需維持多久；預設0.65s。誤觸多就提高到0.8–1.0；太難叫出就降到0.45–0.6。 |
| Pinch Distance | 拇指與食指小於此距離算按下；預設0.025m。難按可增至0.03，誤按則降到0.018–0.022。 |
| Release Distance | 大於此距離才允許下一次點擊；必須大於Pinch Distance。預設0.04m。 |

### 10.3 Hri Floating Menu

| 欄位 | 用途／建議 |
|---|---|
| Modes / Map / Replay / Scene Scale / Camera Extrinsics | 功能引用，正常由工具自動連好，不作畫質調整。 |
| Head View / Surround View / Wrist View | 運行時由相機與D405 subscriber填入；不要手動塞另一張Texture。 |
| Distance | 菜單生成在HMD前方的距離，預設1m；近視野可用0.7–0.9，太近容易難聚焦。 |

### 10.4 Hri RGBD Views：即時點雲品質主要在這裡調

| 欄位 | 效果 | 建議值 |
|---|---|---|
| Point Cloud Downsample | 深度像素抽樣步長。數字越小越密、負載越高。640×480時：1約307,200點/相機；2約76,800；4約19,200；8約4,800。 | Quest/一般GPU先用4；近看手部或細桌邊用2；只有高階GPU才建議1；卡頓用6–8。 |
| Max Depth Meters | 超過距離的點不畫；不會提升原始深度精度，但能減少遠端噪點與overdraw。 | 室內桌面3–5m；房間6–8m；不建議無理由設15m。 |
| Point Size Meters | 點在畫面上的實體大小，只改視覺密實度，不改幾何。 | 0.006–0.010；downsample越大可稍增到0.012–0.02填洞。 |
| Fixed Rig Point Cloud Euler Correction | 只調D455與左右D415三顆固定相機點雲的視覺軸向；不修改ROS外參或URDF。Play中可即時調整。 | 若Apply後固定三顆繞Z軸差90°，先試`(0, 0, 90)`；方向相反用`(0, 0, -90)`。 |
| Head Point Cloud Euler Correction | 只調可移動D435i（`head_camera`）點雲，與固定三顆分開。 | D435i本來正確就保持`(0, 0, 0)`；需要時獨立填±90°。 |
| Left / Right Alignment Root | 可選；要讓可見D415模型跟點雲一起移動，就拖入左右相機各自的模型根物件。留空仍會移動Optical Frame與點雲。 | 不要填D455或整台Stretch3根物件。 |
| Preview Width / Height | 菜單畫面1、2的RenderTexture解析度；只影響預覽清晰度與render成本，不改點數。修改後重新進Play。 | 640×480平衡；低配用480×360；清晰預覽用960×720。 |
| Preview Field Of View | 畫面1、2視野角。 | 60–80°；越大看得廣但物體更小。 |
| Preview Far Clip Meters | 預覽相機最遠裁切；通常應大於Max Depth。 | Max Depth + 2m左右。 |
| Head / D455 / Left / Right Optical Frame | 各點雲的座標來源；綁錯會整片旋轉或位移錯誤。 |
| Surround View Anchor | 畫面2的觀察相機位置；留空跟D455視角，指定空物件可做第三人稱／鳥瞰。 |
| URDF Optical Y Forward | ON代表X右/Y前/Z下；若你的所有frame採Unity Z前/Y上才關閉。不可只改一邊。 |
| Head Layer / Surround Layer | 頭部點雲與固定三相機點雲的隔離Layer。兩者不可相同，也不可被其他內容占用。 |

Downsample、Max Depth、Point Size、兩組Point Cloud Euler Correction可在Play中修改並即時生效；Preview解析度、FOV、Far Clip建議停止Play後修改，再重新Play。

真正的輸入畫質還受Robot端影響：`hri_new/cameras.yaml`的`fps/stream_fps`控制相機與傳送頻率；`rgbd_stream.py`的`default_jpeg_quality`預設80只影響RGB壓縮，**不提升深度精度**。Unity Downsample也只是少畫點，不會改RealSense原始深度解析度。

### 10.5 RGBD TCP Receiver

| 欄位 | 用途／建議 |
|---|---|
| Host / Port | Robot `192.168.0.60:50010`。不是9090，也不是ROS-TCP 10003。 |
| Connect On Start | 保持ON。 |
| Reconnect Delay Ms | 斷線重連間隔；1000ms正常，網路不穩可用2000。 |
| Receive Buffer Bytes | socket接收buffer；預設4MB。高解析／多相機封包截斷或吞吐不足可提高到8–16MB，但不是點雲密度控制。 |
| Max Header/RGB/Depth Bytes | 防止異常大封包的安全上限；只有明確看到`Invalid RGBD packet sizes`才調大。 |
| Verbose | 除錯時開；正常關閉避免Console大量訊息。 |
| Runtime Debug | IsConnected、Received/Dropped Frames與各camera latency；Dropped持續快速增加代表Unity消費不及或stream FPS太高。 |

### 10.6 RGBD Camera Manager

Receiver與Point Cloud FPS是主要可見欄位。相機IDs、reference、投影模式與畫質由HriRgbdViews在Awake設定：reference固定`mast3r_camera`，固定rig只有`camera_left/right`可手動套外參，`head_camera`排除。不要在Play中直接改內部camera IDs。Point Cloud FPS可判斷每路是否正常；某一路0Hz先檢查Robot sender與camera ID。

### 10.7 Camera Extrinsics Subscriber

| 欄位 | 用途／建議 |
|---|---|
| Extrinsics Topic | 固定`/unity/camera_extrinsics`。 |
| Camera Manager | 指HRI System上的同一Manager。 |
| Subscribe On Start | 保持ON。 |
| Persist Last Manual Calibration | 保持ON。每次按Apply成功後保存當時真正pending的左右相機外參；停止Play或重開後會自動載入為pending，但基於安全規則仍須再按一次Apply才移動。你不必重新看AprilTag或重算。 |
| Calibration Persistence Key | 預設`VRRGBD.HRI.FixedRigExtrinsics.v1`，程式還會自動加上Scene名稱與reference camera ID，避免不同場景誤用。不同機器人仍建議填不同名稱。 |
| Verbose | JSON解析／ID問題時才開。 |
| Manual Test JSON | 可貼測試矩陣，但正式校正不要手寫猜測。 |
| Received Messages | 有增加代表ROS-TCP通；不代表已有左右矩陣。 |
| Applied Extrinsics / Last Camera ID / Last Status | 解析狀態。真正可按Apply前，JSON的`extrinsics`必須有`camera_left`或`camera_right`。 |
| Apply Pending Manual Extrinsics | 現在只套左右D415；中間D455固定，head_camera永遠忽略。 |

重新進Play後若Last Status顯示`Loaded ... as pending`，直接按一次Apply即可恢復上次結果；不會在啟動時自動移動。需要作廢舊結果時，從component右上角選單執行`Clear Saved Manual Calibration`。
如果更換Robot、D455/D415實體、USB相機對應或相機安裝位置，必須先Clear，再重新量測；不要沿用舊保存值。

### 10.8 Hri Scene Scale Controller

| 欄位 | 用途／建議 |
|---|---|
| Minimum / Maximum Scale | 防止錯誤比例；預設0.01–100通常不用改。 |
| Requested Scale | 手動精確比例；填完從component選單按`Apply Inspector Scale`。 |
| Known Distance / Measured Raw Map Distance | 同一段物體的真實公尺與SLAM量測距離；按`Apply Known Distance Scale`得到比例。 |
| Raw / Metric World Landmarks | 三組非共線對應點；可同時解scale、rotation、translation。 |
| Maximum Alignment Error Meters | 三點配準允許RMS誤差；預設0.05m。越小越嚴格。 |
| Calibration Status | 顯示成功、拒絕原因與誤差。 |

### 10.9 Hri Robot Map Anchor

Viewer、Robot Root及Slam Camera Optical Frame決定URDF如何沿MASt3R相機軌跡放置。`Stale After Seconds`預設2秒，只影響狀態顯示，不會預測新姿態。Robot Root世界Scale必須是1；URDF Optical Y Forward必須與HriRgbdViews一致。

### 10.10 Runtime Point Cloud Collider／小地圖

這個component通常在regular delta viewer而不是HRI System根上：`Mini Map Resolution`提高會讓小地圖細，但重建較慢；`Mini Map View Height Above Ground`決定平面切片高度；`Point Radius Pixels`調點粗細；`Ground Height Percentile/Offset`調地板估計；`Move Speed/Stopping Distance`只控制Avatar點選導航，不控制實體Robot。`Maximum Ground Samples`越大估計越穩但CPU成本越高。

### 10.11 建議畫質預設組合

| 模式 | Downsample | Max Depth | Point Size | Preview |
|---|---:|---:|---:|---:|
| 穩定／低延遲 | 6–8 | 4–6m | 0.012–0.018m | 480×360 |
| 日常建議 | 4 | 6–8m | 0.008–0.012m | 640×480 |
| 高品質近距離 | 2 | 3–5m | 0.005–0.008m | 960×720 |
| 全解析測試 | 1 | 3–5m | 0.003–0.006m | 960×720；先監看GPU與FPS |
