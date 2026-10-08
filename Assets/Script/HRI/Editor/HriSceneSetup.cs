using System.Linq;
using HRI;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using VRRGBD;

/// <summary>Builds wiring in edit mode only. Operator supplies calibrated robot/XR references.</summary>
public sealed class HriSceneSetup : EditorWindow
{
    private Transform xrOrigin, robotRoot, robotEye, robotHeading, avatarRoot, avatarEye, head, d455, left, right;
    private Camera trackedCamera;
    private Mast3rKeyframeDeltaViewer viewer;
    private RosSharp.RosBridgeClient.RosConnector rosbridge;
    private string robotHost = "192.168.0.60", thorHost = "192.168.0.30";
    [MenuItem("HRI/Configure pure RGB delta scene")]
    private static void Open() => GetWindow<HriSceneSetup>("HRI Scene Setup");
    private void OnGUI()
    {
        EditorGUILayout.HelpBox("Open sunntxi——recon 1. Add the existing Stretch3 URDF/control rig and one XR Hands rig first. Assign calibrated optical frames below. This creates edit-mode objects only; commands default OFF.", MessageType.Info);
        viewer = Field("Regular delta viewer", viewer);
        xrOrigin = Field("XR Origin", xrOrigin); trackedCamera = Field("Tracked HMD Camera", trackedCamera);
        robotRoot = Field("Metric Stretch3 root", robotRoot); robotEye = Field("Robot view anchor", robotEye);
        robotHeading = Field("Base heading (+Z forward)", robotHeading);
        avatarRoot = Field("PlayerArmature root", avatarRoot); avatarEye = Field("PlayerArmature eye anchor", avatarEye);
        rosbridge = Field("Single Robot RosConnector", rosbridge);
        head = Field("Head optical frame", head); d455 = Field("Fixed D455 optical frame", d455);
        left = Field("Left D415 optical frame", left); right = Field("Right D415 optical frame", right);
        robotHost = EditorGUILayout.TextField("Robot host/IP", robotHost);
        thorHost = EditorGUILayout.TextField("Thor host/IP", thorHost);
        using (new EditorGUI.DisabledScope(Application.isPlaying || viewer == null || xrOrigin == null || trackedCamera == null ||
            robotRoot == null || robotEye == null || robotHeading == null || avatarRoot == null || avatarEye == null || rosbridge == null || head == null || d455 == null || left == null || right == null))
            if (GUILayout.Button("Create HRI objects (Undo supported)")) Build();
    }
    private static T Field<T>(string label, T value) where T : Object => (T)EditorGUILayout.ObjectField(label, value, typeof(T), true);
    private void Build()
    {
        Scene targetScene = viewer.gameObject.scene;
        Component[] requiredSceneObjects = { viewer, xrOrigin, trackedCamera, robotRoot, robotEye, robotHeading,
            avatarRoot, avatarEye, rosbridge, head, d455, left, right };
        if (requiredSceneObjects.Any(item => item == null || item.gameObject.scene != targetScene))
        { EditorUtility.DisplayDialog("Mixed scenes", "Every assigned object must belong to the same target scene as Regular delta viewer. Copy the Stretch3/XR objects into sunntxi——recon 1 first; do not assign objects from the additive source scene.", "OK"); return; }
        if (FindInScene<HriInteractionModeController>(targetScene).Any())
        { EditorUtility.DisplayDialog("Already configured", "An HRI mode controller exists. Edit its fields instead of creating duplicate receivers.", "OK"); return; }
        if (viewer.worldAdjust == null || !trackedCamera.transform.IsChildOf(xrOrigin) || !d455.IsChildOf(robotRoot) ||
            robotRoot.IsChildOf(viewer.worldAdjust) || viewer.worldAdjust.IsChildOf(robotRoot))
        { EditorUtility.DisplayDialog("Invalid hierarchy", "Assign worldAdjust, place HMD under XR Origin, D455 optical under metric robot root, and keep scaled map separate from robot root.", "OK"); return; }
        var rosConnections = Object.FindObjectsOfType<Unity.Robotics.ROSTCPConnector.ROSConnection>(true);
        if (rosConnections.Length > 1 || (rosConnections.Length == 1 && rosConnections[0].gameObject.scene != targetScene))
        { EditorUtility.DisplayDialog("ROS-TCP must be unique", "Keep zero or one ROSConnection in all loaded scenes, and it must belong to the target scene. Close the additive source scene or remove duplicate ROSConnection objects, then run setup again.", "OK"); return; }
        if (Object.FindObjectsOfType<CameraExtrinsicsSubscriber>(true).Length > 0)
        { EditorUtility.DisplayDialog("Duplicate extrinsics subscriber", "A CameraExtrinsicsSubscriber already exists in a loaded scene. Remove the old subscriber (or close its source scene) before creating the HRI System.", "OK"); return; }
        int undo = Undo.GetCurrentGroup(); Undo.SetCurrentGroupName("Configure HRI");
        GameObject root = new GameObject("HRI System"); Undo.RegisterCreatedObjectUndo(root, "HRI System");
        SceneManager.MoveGameObjectToScene(root, targetScene);
        var modes = Undo.AddComponent<HriInteractionModeController>(root);
        var menu = Undo.AddComponent<HriFloatingMenu>(root);
        var hand = Undo.AddComponent<HriHandMenuInput>(root);
        var scale = Undo.AddComponent<HriSceneScaleController>(root);
        var anchor = Undo.AddComponent<HriRobotMapAnchor>(root);
        var views = Undo.AddComponent<HriRgbdViews>(root);
        var receiver = Undo.AddComponent<RgbdTcpReceiver>(root);
        var manager = Undo.AddComponent<RgbdCameraManager>(root);
        var extrinsics = Undo.AddComponent<CameraExtrinsicsSubscriber>(root);
        var replay = viewer.GetComponent<Mast3rTrajectoryReplayController>();
        if (replay == null) replay = Undo.AddComponent<Mast3rTrajectoryReplayController>(viewer.gameObject);
        Undo.RecordObject(replay, "HRI replay");
        Undo.RecordObject(viewer, "Connect ground and map");
        if (viewer.runtimeCollider == null) viewer.runtimeCollider = viewer.GetComponent<RuntimePointCloudCollider>();
        if (viewer.runtimeCollider == null) viewer.runtimeCollider = Undo.AddComponent<RuntimePointCloudCollider>(viewer.gameObject);
        modes.xrOrigin = xrOrigin; modes.trackedCamera = trackedCamera; modes.robotEyeAnchor = robotEye; modes.avatarEyeAnchor = avatarEye;
        modes.robotHeadingAnchor = robotHeading;
        modes.allowRobotCommands = false;
        string[] drivers = { "HandToBase", "HandToGripper", "HandToShoulder", "HandToWrist", "keyboardtobase" };
        Transform commandScope = rosbridge.transform;
        while (commandScope.parent != null) commandScope = commandScope.parent;
        modes.teleoperationBehaviours = commandScope.GetComponentsInChildren<MonoBehaviour>(true).Where(b =>
        {
            var field = b.GetType().GetField("Topic");
            string topic = field != null ? field.GetValue(b) as string : null;
            return drivers.Contains(b.GetType().Name) || (b.GetType().Name.Contains("Publisher") &&
                (topic == "/stretch/cmd_vel" || topic == "/stretch/joint_states_control" || topic == "/stretch/gripper_control"));
        }).Cast<Behaviour>().ToArray();
        modes.headControlBehaviours = commandScope.GetComponentsInChildren<HeadCameraControl>(true).Cast<Behaviour>().ToArray();
        foreach (var publisher in modes.teleoperationBehaviours.Concat(modes.headControlBehaviours))
        { Undo.RecordObject(publisher, "Start with commands disabled"); publisher.enabled = false; }
        // The legacy shoulder driver uses wrist world position. Give it a stationary calibrated space,
        // so avatar teleports/XR-origin translation cannot become arm target jumps.
        var commandSpace = new GameObject("Stationary Teleop Tracking Space");
        Undo.RegisterCreatedObjectUndo(commandSpace, "Teleop space");
        SceneManager.MoveGameObjectToScene(commandSpace, targetScene);
        commandSpace.transform.SetPositionAndRotation(trackedCamera.transform.parent.position, trackedCamera.transform.parent.rotation);
        foreach (var driver in modes.teleoperationBehaviours)
        {
            string fieldName = driver is HandToShoulder ? "rWrist" : null;
            if (fieldName == null) continue;
            var field = driver.GetType().GetField(fieldName); var source = field.GetValue(driver) as Transform;
            if (source == null) continue;
            var proxyObject = new GameObject("HRI " + fieldName + " Command Proxy");
            Undo.RegisterCreatedObjectUndo(proxyObject, "Hand proxy"); proxyObject.transform.SetParent(commandSpace.transform, false);
            var proxy = Undo.AddComponent<HriTrackedPoseProxy>(proxyObject);
            proxy.source = source; proxy.trackingSpace = trackedCamera.transform.parent; proxy.commandTrackingSpace = commandSpace.transform;
            Undo.RecordObject(driver, "Use tracking-local hand"); field.SetValue(driver, proxyObject.transform);
        }
        menu.modes = modes; menu.map = viewer.runtimeCollider; menu.replay = replay; menu.sceneScale = scale;
        menu.cameraExtrinsics = extrinsics;
        hand.modes = modes; hand.menu = menu; hand.trackingSpace = trackedCamera.transform.parent;
        scale.viewer = viewer; scale.mapAdjustment = viewer.worldAdjust;
        anchor.viewer = viewer; anchor.robotRoot = robotRoot; anchor.slamCameraOpticalFrame = d455;
        views.receiver = receiver; views.manager = manager; views.menu = menu;
        views.headOpticalFrame = head; views.d455OpticalFrame = d455; views.leftOpticalFrame = left; views.rightOpticalFrame = right;
        receiver.host = robotHost; receiver.port = 50010; manager.receiver = receiver;
        extrinsics.cameraManager = manager;
        extrinsics.extrinsicsTopic = "/unity/camera_extrinsics";
        extrinsics.subscribeOnStart = true;
        replay.trajectorySource = viewer.runtimeCollider;
        if (viewer.runtimeCollider != null)
        {
            Undo.RecordObject(viewer.runtimeCollider, "HRI navigation");
            viewer.runtimeCollider.controlledRobot = avatarRoot;
            viewer.runtimeCollider.robotCharacterController = avatarRoot.GetComponent<CharacterController>();
            replay.controlledRobot = avatarRoot; replay.characterController = avatarRoot.GetComponent<CharacterController>();
            viewer.runtimeCollider.enableClickToMove = true;
        }
        // Retain existing joint subscribers; add only the wrist image consumer.
        var wrist = rosbridge.GetComponent<HriWristRgbSubscriber>();
        if (wrist == null) wrist = Undo.AddComponent<HriWristRgbSubscriber>(rosbridge.gameObject);
        wrist.menu = menu;
        Undo.RecordObject(rosbridge, "HRI Robot URL"); rosbridge.RosBridgeServerUrl = "ws://" + robotHost + ":9090";
        var ros = rosConnections.SingleOrDefault();
        if (ros == null) ros = Undo.AddComponent<Unity.Robotics.ROSTCPConnector.ROSConnection>(root);
        Undo.RecordObject(ros, "HRI Thor endpoint"); ros.RosIPAddress = thorHost; ros.RosPort = 10003;
        modes.conflictingViewControllers = FindInScene<MonoBehaviour>(targetScene)
            .Where(b => b is ViewpointSwitcher || b is AlignVRToRobotHead || b.GetType().Name == "CinemachineBrain" || b.GetType().Name.StartsWith("RobotPublisherSwitcher"))
            .Cast<Behaviour>().ToArray();
        foreach (var conflict in modes.conflictingViewControllers) Undo.DestroyObjectImmediate(conflict);
        modes.conflictingViewControllers = new Behaviour[0];
        // Comparison subscribers belong to a different profile and cannot share this ROS singleton.
        foreach (var other in FindInScene<Mast3rKeyframeDeltaViewer>(targetScene))
            if (other != viewer) { Undo.RecordObject(other, "Disable comparison viewer"); other.enabled = false; }
        foreach (var other in FindInScene<MultiRobotRosTcpMapViewer>(targetScene))
        { Undo.RecordObject(other, "Disable full viewer"); other.enabled = false; }
        Undo.RecordObject(viewer, "HRI delta profile"); viewer.syncTrajectoryToCloudUpdates = false;
        foreach (var old in FindInScene<RgbdBridgeBootstrap>(targetScene))
            Undo.DestroyObjectImmediate(old); // Awake on a disabled bootstrap can still replace the ROS singleton endpoint.
        foreach (var old in FindInScene<RgbdTcpReceiver>(targetScene))
            if (old != receiver) { Undo.RecordObject(old, "Disable duplicate RGBD receiver"); old.enabled = false; }
        foreach (var old in FindInScene<RgbdCameraManager>(targetScene))
            if (old != manager) { Undo.RecordObject(old, "Disable duplicate RGBD consumer"); old.enabled = false; }
        EditorSceneManager.MarkSceneDirty(root.scene); Selection.activeGameObject = root;
        Undo.CollapseUndoOperations(undo);
        EditorUtility.DisplayDialog("Wiring created", "Assign ALL actual command publishers and hand drivers to Teleoperation Behaviours; assign HeadCameraControl to Head Control Behaviours. Disable duplicate receivers/publishers and other robot-root pose writers. Verify optical extrinsics and reserved layers 28/29. Keep Allow Robot Commands OFF for initial visual tests. Save the scene explicitly.", "OK");
    }

    private static T[] FindInScene<T>(Scene scene) where T : Component
    {
        return scene.GetRootGameObjects()
            .SelectMany(rootObject => rootObject.GetComponentsInChildren<T>(true))
            .ToArray();
    }
}
