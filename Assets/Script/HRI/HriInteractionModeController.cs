using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace HRI
{
    /// <summary>One tracked camera, two viewing anchors. This component never publishes ROS commands.</summary>
    [DefaultExecutionOrder(-200)]
    public sealed class HriInteractionModeController : MonoBehaviour
    {
        public enum ViewMode { Robot, Avatar }
        public enum MenuTab { Map, Views, Scene }
        public Transform xrOrigin;
        public Camera trackedCamera;
        public Transform robotEyeAnchor;
        [Tooltip("Base heading only, Unity +Z forward/+Y up. Must not include head pan/tilt.")]
        public Transform robotHeadingAnchor;
        public Transform avatarEyeAnchor;
        [Tooltip("Actual ROS command publishers AND hand-to-joint drivers; do not include feedback subscribers.")]
        public Behaviour[] teleoperationBehaviours = new Behaviour[0];
        public Behaviour[] headControlBehaviours = new Behaviour[0];
        [Tooltip("Old Space switcher, AlignVRToRobotHead, Cinemachine brain, or other rig writers.")]
        public Behaviour[] conflictingViewControllers = new Behaviour[0];
        [Tooltip("Off by default. Enable only for the user's supervised robot experiment.")]
        public bool allowRobotCommands;
        public ViewMode initialMode = ViewMode.Avatar;
        public ViewMode Mode { get; private set; }
        public MenuTab Tab { get; private set; }
        public bool MenuOpen { get; private set; }
        public bool UiGestureActive { get; set; }
        public bool CommandsPermitted => allowRobotCommands && !UiGestureActive &&
            (Mode == ViewMode.Robot || (MenuOpen && Tab == MenuTab.Views));
        public event System.Action StateChanged;
        private bool lastPermission;
        private Vector3 avatarViewPosition;
        private bool freezeAvatarPosition;
        private float avatarHeading;

        private void Awake()
        {
            foreach (Behaviour b in conflictingViewControllers) if (b != null && b != this) b.enabled = false;
            Mode = initialMode;
            avatarHeading = xrOrigin != null ? xrOrigin.eulerAngles.y : 0f;
            SetCommands(false);
        }

        private void Update()
        {
#if ENABLE_INPUT_SYSTEM
            bool space = Keyboard.current != null && Keyboard.current.spaceKey.wasPressedThisFrame;
            bool menu = Keyboard.current != null && Keyboard.current.mKey.wasPressedThisFrame;
#else
            bool space = Input.GetKeyDown(KeyCode.Space);
            bool menu = Input.GetKeyDown(KeyCode.M);
#endif
            if (space) ToggleMode();
            if (menu) ToggleMenu();
            SetCommands(CommandsPermitted);
        }

        private void LateUpdate()
        {
            if (xrOrigin == null || trackedCamera == null || !trackedCamera.transform.IsChildOf(xrOrigin)) return;
            Transform anchor = Mode == ViewMode.Robot ? robotEyeAnchor : avatarEyeAnchor;
            if (anchor == null) return;
            float heading = Mode == ViewMode.Robot && robotHeadingAnchor != null ? robotHeadingAnchor.eulerAngles.y : avatarHeading;
            xrOrigin.rotation = Quaternion.Euler(0, heading, 0);
            // Keep HMD tracking intact. Subtract its current positional offset to place the eye at the anchor.
            // Do not inherit robot pan/tilt rotation: doing so would apply the user's head rotation twice.
            Vector3 eye = freezeAvatarPosition && Mode == ViewMode.Avatar ? avatarViewPosition : anchor.position;
            xrOrigin.position += eye - trackedCamera.transform.position;
        }

        public void ToggleMode()
        {
            if (Mode == ViewMode.Avatar && xrOrigin != null) avatarHeading = xrOrigin.eulerAngles.y;
            Mode = Mode == ViewMode.Robot ? ViewMode.Avatar : ViewMode.Robot;
            MenuOpen = false;
            freezeAvatarPosition = false;
            UiGestureActive = false;
            SetCommands(CommandsPermitted);
            StateChanged?.Invoke();
        }

        public void ToggleMenu()
        {
            if (Mode != ViewMode.Avatar) return;
            MenuOpen = !MenuOpen;
            if (MenuOpen) Tab = MenuTab.Map;
            UpdateFreeze();
            SetCommands(CommandsPermitted);
            StateChanged?.Invoke();
        }

        public void SelectTab(MenuTab value)
        {
            Tab = value;
            UpdateFreeze();
            SetCommands(CommandsPermitted);
            StateChanged?.Invoke();
        }

        private void UpdateFreeze()
        {
            bool next = MenuOpen && Tab == MenuTab.Views && Mode == ViewMode.Avatar;
            if (next && !freezeAvatarPosition && trackedCamera != null) avatarViewPosition = trackedCamera.transform.position;
            freezeAvatarPosition = next;
        }

        private void SetCommands(bool value)
        {
            foreach (Behaviour b in teleoperationBehaviours) if (b != null && b != this) b.enabled = value;
            foreach (Behaviour b in headControlBehaviours) if (b != null && b != this) b.enabled = value;
            lastPermission = value;
        }

        private void OnDisable() { SetCommands(false); }
        private void OnApplicationFocus(bool focus) { if (!focus) { allowRobotCommands = false; SetCommands(false); } }
    }
}
