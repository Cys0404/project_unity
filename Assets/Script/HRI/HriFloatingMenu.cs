using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.UI;
using VRRGBD;

namespace HRI
{
    /// <summary>World-space UI, with explicit hand-ray input; no robot command publishing.</summary>
    public sealed class HriFloatingMenu : MonoBehaviour
    {
        public HriInteractionModeController modes;
        public RuntimePointCloudCollider map;
        public Mast3rTrajectoryReplayController replay;
        public HriSceneScaleController sceneScale;
        public CameraExtrinsicsSubscriber cameraExtrinsics;
        public Texture headView;
        public Texture surroundView;
        public Texture wristView;
        public float distance = 1.0f;
        public RawImage WristImage { get; private set; }
        private Canvas canvas;
        private GameObject mapPage, viewsPage, scenePage, alignmentChoicePage, scaleAlignmentPage, cameraAlignmentPage;
        private RawImage mapImage;
        private RawImage headImage, surroundImage;
        private Text scaleText, statusText, cameraAlignmentStatusText;
        private readonly List<Button> buttons = new List<Button>();
        private RectTransform panel;
        private RectTransform avatarMarker;
        private readonly List<RectTransform> trajectoryLines = new List<RectTransform>();
        private float nextRefresh;
        private bool wasOpen;
        private int manualAlignmentClicks;
        private string manualAlignmentStatus = "Waiting for pending ROS extrinsics.";

        private enum AlignmentPage { Choice, Scale, ThreeCamera }
        private AlignmentPage alignmentPage = AlignmentPage.Choice;

        private void Start()
        {
            if (cameraExtrinsics == null) cameraExtrinsics = FindObjectOfType<CameraExtrinsicsSubscriber>();
            Build();
            if (modes != null) modes.StateChanged += RefreshState;
            if (sceneScale != null) sceneScale.Changed += StopOldNavigation;
            RefreshState();
        }
        private void OnDestroy()
        {
            if (modes != null) modes.StateChanged -= RefreshState;
            if (sceneScale != null) sceneScale.Changed -= StopOldNavigation;
            if (canvas != null) Destroy(canvas.gameObject);
        }
        private void StopOldNavigation()
        {
            if (replay != null) replay.StopReplay();
            if (map != null) map.CancelNavigation();
        }
        private void Build()
        {
            var root = new GameObject("HRI World Menu", typeof(RectTransform), typeof(Canvas), typeof(GraphicRaycaster));
            canvas = root.GetComponent<Canvas>(); canvas.renderMode = RenderMode.WorldSpace;
            panel = root.GetComponent<RectTransform>(); panel.sizeDelta = new Vector2(1000, 670);
            panel.localScale = Vector3.one * 0.0012f;
            RectTransform bg = Rect("Background", panel, new Vector2(500, 335), new Vector2(1000, 670));
            bg.gameObject.AddComponent<Image>().color = new Color(0.035f, 0.05f, 0.07f, 0.96f);
            AddButton(panel, "Map", 20, 595, 230, () => modes.SelectTab(HriInteractionModeController.MenuTab.Map));
            AddButton(panel, "Views", 270, 595, 230, () => modes.SelectTab(HriInteractionModeController.MenuTab.Views));
            AddButton(panel, "Scene / Alignment", 520, 595, 230, OpenAlignmentChoices);
            AddButton(panel, "Close", 770, 595, 210, () => modes.ToggleMenu());
            mapPage = Page("Map", panel); viewsPage = Page("Views", panel); scenePage = Page("Scene", panel);
            mapImage = Raw(mapPage.transform, "Live map", 20, 25, 540, 540);
            avatarMarker = Rect("Avatar marker", mapImage.transform, Vector2.zero, Vector2.one * 12);
            avatarMarker.gameObject.AddComponent<Image>().color = Color.green;
            AddButton(mapPage.transform, "Follow trajectory / Return", 590, 440, 380, () => { if (replay != null) replay.StartNextTraversal(); });
            AddButton(mapPage.transform, "Stop avatar", 590, 360, 380, StopOldNavigation);
            AddButton(mapPage.transform, "Map height +", 590, 270, 380, () => { if (map != null) map.miniMapViewHeightAboveGround += 0.1f; });
            AddButton(mapPage.transform, "Map height -", 590, 190, 380, () => { if (map != null) map.miniMapViewHeightAboveGround = Mathf.Max(0.1f, map.miniMapViewHeightAboveGround - 0.1f); });
            Label(mapPage.transform, "Right index + thumb pinch to select.\nGreen: avatar. Red: robot camera path.", 590, 40, 370, 120, 24);
            headImage = FramedView(viewsPage.transform, "1  Head point cloud", 20);
            surroundImage = FramedView(viewsPage.transform, "2  D455 + D415 point clouds", 350);
            WristImage = FramedView(viewsPage.transform, "3  Wrist RGB", 680);
            Label(viewsPage.transform, "Robot gestures active only when Allow Robot Commands is enabled.\nPinching a menu control temporarily suppresses robot gestures.", 30, 60, 930, 90, 24);
            alignmentChoicePage = Page("Alignment choices", scenePage.transform);
            Label(alignmentChoicePage.transform, "Choose a manual alignment tool", 50, 440, 900, 70, 34);
            AddButton(alignmentChoicePage.transform, "1  Scene scale alignment", 150, 300, 700, () => ShowAlignmentPage(AlignmentPage.Scale));
            AddButton(alignmentChoicePage.transform, "2  Three-camera point-cloud alignment", 150, 190, 700, () => ShowAlignmentPage(AlignmentPage.ThreeCamera));
            Label(alignmentChoicePage.transform, "Both tools are manual. No transform is applied until you press its button.", 80, 70, 840, 80, 24);

            scaleAlignmentPage = Page("Scale alignment", scenePage.transform);
            scaleText = Label(scaleAlignmentPage.transform, "Scale", 50, 420, 900, 70, 36);
            AddButton(scaleAlignmentPage.transform, "- 10%", 50, 330, 260, () => ChangeScale(1f / 1.1f));
            AddButton(scaleAlignmentPage.transform, "+ 10%", 350, 330, 260, () => ChangeScale(1.1f));
            AddButton(scaleAlignmentPage.transform, "Reset 1.0", 650, 330, 280, () => { if (sceneScale != null) sceneScale.ResetScale(); });
            Label(scaleAlignmentPage.transform, "Map and trajectory scale together. Robot dimensions stay in metres.\nExact scale and measured landmark calibration are available in the\nHriSceneScaleController Inspector.", 50, 160, 900, 150, 26);
            statusText = Label(scaleAlignmentPage.transform, "", 50, 55, 900, 90, 22);
            AddButton(scaleAlignmentPage.transform, "Back", 20, 5, 180, () => ShowAlignmentPage(AlignmentPage.Choice));

            cameraAlignmentPage = Page("Three-camera alignment", scenePage.transform);
            Label(cameraAlignmentPage.transform, "D455 + left/right D415 point-cloud alignment", 50, 445, 900, 70, 32);
            Label(cameraAlignmentPage.transform, "ROS extrinsics are received and kept pending. Press the button once to apply\nthe latest pending transforms. There is no automatic apply mode.", 80, 315, 840, 110, 25);
            AddButton(cameraAlignmentPage.transform, "Apply latest ROS extrinsics once", 170, 210, 660, ApplyCameraExtrinsicsOnce);
            cameraAlignmentStatusText = Label(cameraAlignmentPage.transform, "", 80, 75, 840, 115, 22);
            AddButton(cameraAlignmentPage.transform, "Back", 20, 5, 180, () => ShowAlignmentPage(AlignmentPage.Choice));
            ShowAlignmentPage(AlignmentPage.Choice);
        }
        private void OpenAlignmentChoices()
        {
            alignmentPage = AlignmentPage.Choice;
            modes.SelectTab(HriInteractionModeController.MenuTab.Scene);
            RefreshAlignmentPages();
        }
        private void ShowAlignmentPage(AlignmentPage page)
        {
            alignmentPage = page;
            RefreshAlignmentPages();
        }
        private void RefreshAlignmentPages()
        {
            if (alignmentChoicePage == null) return;
            alignmentChoicePage.SetActive(alignmentPage == AlignmentPage.Choice);
            scaleAlignmentPage.SetActive(alignmentPage == AlignmentPage.Scale);
            cameraAlignmentPage.SetActive(alignmentPage == AlignmentPage.ThreeCamera);
        }
        private void ApplyCameraExtrinsicsOnce()
        {
            if (cameraExtrinsics == null) cameraExtrinsics = FindObjectOfType<CameraExtrinsicsSubscriber>();
            if (cameraExtrinsics == null || cameraExtrinsics.cameraManager == null)
            { manualAlignmentStatus = "No CameraExtrinsicsSubscriber / RgbdCameraManager assigned."; return; }
            int pending = cameraExtrinsics.cameraManager.PendingManualExtrinsicsCountForCameras(
                "camera_left", "camera_right");
            if (pending <= 0)
            { manualAlignmentStatus = "Nothing applied: no pending camera extrinsics."; return; }
            int applied = cameraExtrinsics.ApplyPendingFixedRigExtrinsics();
            if (applied <= 0)
            { manualAlignmentStatus = "Nothing applied: left/right transforms were not available."; return; }
            manualAlignmentClicks++;
            manualAlignmentStatus = "Applied " + applied + " fixed-rig transform(s); centre D455 stayed fixed; head excluded.";
        }
        private void ChangeScale(float ratio) { if (sceneScale != null) sceneScale.SetScale(sceneScale.Scale * ratio); }
        private void RefreshState()
        {
            if (canvas == null || modes == null) return;
            bool open = modes.MenuOpen;
            if (open && !wasOpen && modes.trackedCamera != null)
            {
                Transform eye = modes.trackedCamera.transform;
                Vector3 forward = Vector3.ProjectOnPlane(eye.forward, Vector3.up).normalized;
                if (forward.sqrMagnitude < 0.01f) forward = Vector3.forward;
                panel.SetPositionAndRotation(eye.position + forward * distance, Quaternion.LookRotation(forward));
            }
            canvas.gameObject.SetActive(open); wasOpen = open;
            mapPage.SetActive(modes.Tab == HriInteractionModeController.MenuTab.Map);
            viewsPage.SetActive(modes.Tab == HriInteractionModeController.MenuTab.Views);
            scenePage.SetActive(modes.Tab == HriInteractionModeController.MenuTab.Scene);
            RefreshAlignmentPages();
        }
        private void Update()
        {
            if (canvas == null || modes == null || !modes.MenuOpen) return;
            headImage.texture = headView; surroundImage.texture = surroundView;
            if (wristView != null) WristImage.texture = wristView;
            if (sceneScale != null) { scaleText.text = "Scene scale: " + sceneScale.Scale.ToString("F4", CultureInfo.InvariantCulture); statusText.text = sceneScale.calibrationStatus; }
            if (cameraAlignmentStatusText != null)
            {
                cameraAlignmentStatusText.text = cameraExtrinsics == null || cameraExtrinsics.cameraManager == null
                    ? "CameraExtrinsicsSubscriber is not assigned. Run HRI scene setup or assign it in Inspector."
                    : "Topic: " + cameraExtrinsics.extrinsicsTopic + "\nReceived: " + cameraExtrinsics.receivedMessages +
                      "   Last camera: " + (string.IsNullOrEmpty(cameraExtrinsics.lastCameraId) ? "-" : cameraExtrinsics.lastCameraId) +
                      "   Pending left/right: " + cameraExtrinsics.cameraManager.PendingManualExtrinsicsCountForCameras("camera_left", "camera_right") +
                      "   Successful manual applies: " + manualAlignmentClicks + "\n" + manualAlignmentStatus +
                      "\nReceiver: " + cameraExtrinsics.lastStatus;
            }
            if (Time.unscaledTime < nextRefresh || map == null) return;
            nextRefresh = Time.unscaledTime + 0.2f;
            mapImage.texture = map.MiniMapTexture;
            if (map.controlledRobot != null && map.TryWorldToMiniMap(map.controlledRobot.position, out Vector2 uv))
                avatarMarker.anchoredPosition = uv * 540;
            int count = 0;
            if (map.TryGetCameraTrajectoryWorld(out Vector3[] path))
            {
                int step = Mathf.Max(1, path.Length / 200);
                for (int i = step; i < path.Length; i += step)
                {
                    if (!map.TryWorldToMiniMap(path[i - step], out Vector2 a) || !map.TryWorldToMiniMap(path[i], out Vector2 b)) continue;
                    if (count == trajectoryLines.Count)
                    {
                        RectTransform line = Rect("Trajectory", mapImage.transform, Vector2.zero, Vector2.one);
                        line.gameObject.AddComponent<Image>().color = Color.red;
                        trajectoryLines.Add(line);
                    }
                    RectTransform item = trajectoryLines[count++]; item.gameObject.SetActive(true);
                    Vector2 delta = (b - a) * 540;
                    item.anchoredPosition = (a + b) * 270;
                    item.sizeDelta = new Vector2(delta.magnitude, 3);
                    item.localRotation = Quaternion.Euler(0, 0, Mathf.Atan2(delta.y, delta.x) * Mathf.Rad2Deg);
                }
            }
            for (int i = count; i < trajectoryLines.Count; i++) trajectoryLines[i].gameObject.SetActive(false);
            avatarMarker.SetAsLastSibling();
        }
        public bool Point(Ray ray, bool press)
        {
            return Point(ray, press, out _);
        }
        public bool Point(Ray ray, bool press, out float hitDistance)
        {
            hitDistance = 0f;
            if (canvas == null || modes == null || !modes.MenuOpen) return false;
            if (!new Plane(panel.forward, panel.position).Raycast(ray, out float distanceToPlane) || distanceToPlane < 0) return false;
            Vector3 world = ray.GetPoint(distanceToPlane);
            if (!panel.rect.Contains(panel.InverseTransformPoint(world))) return false;
            hitDistance = distanceToPlane;
            if (!press) return true;
            foreach (Button button in buttons)
                if (button.gameObject.activeInHierarchy && ((RectTransform)button.transform).rect.Contains(button.transform.InverseTransformPoint(world)))
                { button.onClick.Invoke(); return true; }
            if (mapPage.activeSelf && mapImage.rectTransform.rect.Contains(mapImage.transform.InverseTransformPoint(world)))
            {
                Vector2 local = mapImage.transform.InverseTransformPoint(world);
                Vector2 uv = (local + mapImage.rectTransform.rect.size * 0.5f) / 540f;
                if (replay != null) replay.StopReplay();
                if (map != null) map.NavigateMiniMap(uv);
            }
            return true;
        }
        private static RectTransform Rect(string name, Transform parent, Vector2 center, Vector2 size)
        {
            var go = new GameObject(name, typeof(RectTransform));
            var rect = (RectTransform)go.transform; rect.SetParent(parent, false);
            rect.anchorMin = rect.anchorMax = Vector2.zero; rect.pivot = Vector2.one * 0.5f;
            rect.anchoredPosition = center; rect.sizeDelta = size; return rect;
        }
        private static GameObject Page(string name, Transform parent)
        { return Rect(name, parent, new Vector2(500, 290), new Vector2(1000, 580)).gameObject; }
        private static Text Label(Transform parent, string text, float x, float y, float w, float h, int size)
        {
            Text label = Rect(text, parent, new Vector2(x + w / 2, y + h / 2), new Vector2(w, h)).gameObject.AddComponent<Text>();
            label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf"); label.text = text; label.fontSize = size;
            label.color = Color.white; label.alignment = TextAnchor.MiddleCenter; label.raycastTarget = false; return label;
        }
        private void AddButton(Transform parent, string text, float x, float y, float width, Action action)
        {
            RectTransform rect = Rect(text, parent, new Vector2(x + width / 2, y + 30), new Vector2(width, 60));
            rect.gameObject.AddComponent<Image>().color = new Color(0.1f, 0.27f, 0.38f);
            Button button = rect.gameObject.AddComponent<Button>(); button.onClick.AddListener(() => action()); buttons.Add(button);
            Label(rect, text, 0, 0, width, 60, 25);
        }
        private static RawImage Raw(Transform parent, string name, float x, float y, float w, float h)
        { var image = Rect(name, parent, new Vector2(x + w / 2, y + h / 2), new Vector2(w, h)).gameObject.AddComponent<RawImage>(); image.color = Color.white; return image; }
        private RawImage FramedView(Transform parent, string name, float x)
        {
            RectTransform border = Rect(name + " border", parent, new Vector2(x + 150, 360), new Vector2(310, 270));
            border.gameObject.AddComponent<Image>().color = Color.gray;
            RawImage result = Raw(border, name, 5, 5, 300, 260);
            Label(parent, name, x, 505, 310, 65, 23); return result;
        }
    }
}
