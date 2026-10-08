using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Hands;

namespace HRI
{
    [DefaultExecutionOrder(-300)]
    public sealed class HriHandMenuInput : MonoBehaviour
    {
        public HriInteractionModeController modes;
        public HriFloatingMenu menu;
        [Tooltip("XR tracking-space transform, usually Camera Offset. Must be the same space used by XR Hands.")]
        public Transform trackingSpace;
        [Min(0.1f)] public float palmHoldSeconds = 0.65f;
        [Min(0.005f)] public float pinchDistance = 0.025f;
        [Min(0.005f)] public float releaseDistance = 0.04f;
        [Header("Two-hand menu rays")]
        public bool showHandRays = true;
        [Min(0.2f)] public float rayLength = 3f;
        [Min(0.001f)] public float rayWidth = 0.004f;
        public Color rayColor = new Color(0.2f, 0.75f, 1f, 0.9f);
        public Color rayHoverColor = new Color(0.25f, 1f, 0.35f, 1f);
        public Color rayPinchColor = new Color(1f, 0.8f, 0.15f, 1f);
        [Header("Runtime Debug (read only during Play)")]
        [Tooltip("No running subsystem means the active OpenXR runtime/build target is not providing XR Hands.")]
        public string subsystemStatus = "Waiting for XR Hands";
        public int discoveredSubsystems;
        public bool leftHandTracked;
        public bool rightHandTracked;
        public bool leftPinchJointsAvailable;
        public bool rightPinchJointsAvailable;
        private readonly List<XRHandSubsystem> systems = new List<XRHandSubsystem>();
        private XRHandSubsystem hands;
        private float palmHeld;
        private bool palmLatched;
        private readonly PinchTracker leftPinch = new PinchTracker();
        private readonly PinchTracker rightPinch = new PinchTracker();
        private LineRenderer leftRayVisual, rightRayVisual;
        private Material rayMaterial;

        private sealed class PinchTracker
        {
            public bool pinched;
            public bool armed;
        }

        private void Awake()
        {
            Shader shader = Shader.Find("Sprites/Default");
            if (shader == null) shader = Shader.Find("Unlit/Color");
            if (shader != null) rayMaterial = new Material(shader) { name = "HRI Hand Ray Material" };
            leftRayVisual = CreateRayVisual("Left XR Hand Ray");
            rightRayVisual = CreateRayVisual("Right XR Hand Ray");
        }

        private void Update()
        {
            if (modes == null || menu == null || trackingSpace == null)
            {
                subsystemStatus = "Missing Modes, Menu, or Tracking Space reference.";
                HideRays();
                ResetInput();
                return;
            }
            if (hands == null || !hands.running)
            {
                systems.Clear();
                SubsystemManager.GetSubsystems(systems);
                discoveredSubsystems = systems.Count;
                hands = systems.Find(s => s.running);
            }
            if (hands == null || !hands.running)
            {
                subsystemStatus = discoveredSubsystems == 0
                    ? "No XRHandSubsystem discovered. Enable OpenXR Hand Tracking for the active build target/runtime."
                    : $"Found {discoveredSubsystems} XRHandSubsystem(s), but none is running.";
                leftHandTracked = rightHandTracked = leftPinchJointsAvailable = rightPinchJointsAvailable = false;
                HideRays();
                ResetInput();
                return;
            }
            subsystemStatus = "XRHandSubsystem is running.";
            leftHandTracked = hands.leftHand.isTracked;
            rightHandTracked = hands.rightHand.isTracked;
            bool summon = modes.Mode == HriInteractionModeController.ViewMode.Avatar && OpenPalmFacingHead(hands.leftHand);
            if (summon)
            {
                palmHeld += Time.unscaledDeltaTime;
                if (!palmLatched && palmHeld >= palmHoldSeconds)
                {
                    modes.ToggleMenu();
                    palmLatched = true;
                }
            }
            else { palmHeld = 0f; palmLatched = false; }

            bool leftOverMenu = UpdateHandRay(hands.leftHand, leftPinch, leftRayVisual, true,
                out leftPinchJointsAvailable, out bool leftPressConsumed);
            bool rightOverMenu = UpdateHandRay(hands.rightHand, rightPinch, rightRayVisual, !leftPressConsumed,
                out rightPinchJointsAvailable, out _);
            modes.UiGestureActive = summon || (leftOverMenu && leftPinch.pinched) ||
                (rightOverMenu && rightPinch.pinched);
        }

        private bool UpdateHandRay(XRHand hand, PinchTracker pinch, LineRenderer visual, bool allowPress,
            out bool pinchJointsAvailable, out bool pressConsumed)
        {
            pressConsumed = false;
            Pose thumb = default, indexTip = default, palm = default, middleProximal = default;
            bool rayTracked = hand.isTracked && Joint(hand, XRHandJointID.ThumbTip, out thumb) &&
                Joint(hand, XRHandJointID.IndexTip, out indexTip) &&
                Joint(hand, XRHandJointID.Palm, out palm) &&
                Joint(hand, XRHandJointID.MiddleProximal, out middleProximal);
            pinchJointsAvailable = rayTracked;
            if (!rayTracked)
            {
                ResetPinch(pinch);
                SetRayVisible(visual, false);
                return false;
            }

            float distance = Vector3.Distance(thumb.position, indexTip.position);
            // Require a release before every click, including after tracking recovery.
            if (distance >= releaseDistance) { pinch.armed = true; pinch.pinched = false; }
            bool down = pinch.armed && !pinch.pinched && distance <= pinchDistance;
            if (down) { pinch.pinched = true; pinch.armed = false; }

            // Aim with the palm and middle-finger base so thumb/index pinching does not shake the ray.
            Vector3 origin = trackingSpace.TransformPoint(palm.position);
            Vector3 localDirection = middleProximal.position - palm.position;
            Vector3 direction = localDirection.sqrMagnitude > 0.000001f
                ? trackingSpace.TransformDirection(localDirection).normalized
                : trackingSpace.TransformDirection(palm.rotation * Vector3.forward).normalized;
            Ray ray = new Ray(origin, direction);
            bool sendPress = allowPress && down;
            float menuHitDistance = rayLength;
            bool overMenu = modes.MenuOpen && menu.Point(ray, sendPress, out menuHitDistance);
            pressConsumed = sendPress && overMenu;
            UpdateRayVisual(visual, ray, overMenu, pinch.pinched,
                overMenu ? Mathf.Min(menuHitDistance, rayLength) : rayLength);
            return overMenu;
        }

        private LineRenderer CreateRayVisual(string objectName)
        {
            var rayObject = new GameObject(objectName);
            rayObject.transform.SetParent(transform, false);
            var line = rayObject.AddComponent<LineRenderer>();
            line.useWorldSpace = true;
            line.positionCount = 2;
            line.sharedMaterial = rayMaterial;
            line.widthMultiplier = rayWidth;
            line.numCapVertices = 4;
            line.enabled = false;
            return line;
        }

        private void UpdateRayVisual(LineRenderer line, Ray ray, bool overMenu, bool isPinched, float visibleLength)
        {
            if (line == null) return;
            line.widthMultiplier = rayWidth;
            line.SetPosition(0, ray.origin);
            line.SetPosition(1, ray.origin + ray.direction * Mathf.Max(0f, visibleLength));
            Color color = isPinched ? rayPinchColor : overMenu ? rayHoverColor : rayColor;
            line.startColor = line.endColor = color;
            SetRayVisible(line, showHandRays && modes.MenuOpen);
        }

        private static void SetRayVisible(LineRenderer line, bool visible)
        {
            if (line != null) line.enabled = visible;
        }

        private void HideRays()
        {
            SetRayVisible(leftRayVisual, false);
            SetRayVisible(rightRayVisual, false);
        }

        private bool OpenPalmFacingHead(XRHand hand)
        {
            if (!hand.isTracked || modes.trackedCamera == null || !Joint(hand, XRHandJointID.Palm, out Pose palm)) return false;
            if (!Joint(hand, XRHandJointID.Wrist, out Pose wrist)) return false;
            XRHandJointID[] tips = { XRHandJointID.IndexTip, XRHandJointID.MiddleTip, XRHandJointID.RingTip, XRHandJointID.LittleTip };
            foreach (XRHandJointID id in tips)
                if (!Joint(hand, id, out Pose tip) || Vector3.Distance(tip.position, wrist.position) < 0.13f) return false;
            Vector3 p = trackingSpace.TransformPoint(palm.position);
            Vector3 normal = trackingSpace.TransformDirection(palm.rotation * -Vector3.up);
            Vector3 towardEye = modes.trackedCamera.transform.position - p;
            return towardEye.magnitude < 0.8f && Vector3.Dot(normal.normalized, towardEye.normalized) > 0.65f;
        }
        private static bool Joint(XRHand hand, XRHandJointID id, out Pose pose) => hand.GetJoint(id).TryGetPose(out pose);
        private static void ResetPinch(PinchTracker pinch) { pinch.armed = false; pinch.pinched = false; }
        private void ResetInput()
        {
            ResetPinch(leftPinch); ResetPinch(rightPinch);
            palmHeld = 0f; palmLatched = false;
            if (modes != null) modes.UiGestureActive = false;
        }
        private void OnDisable()
        {
            subsystemStatus = "HriHandMenuInput is disabled.";
            leftHandTracked = rightHandTracked = leftPinchJointsAvailable = rightPinchJointsAvailable = false;
            HideRays();
            ResetInput();
        }
        private void OnDestroy()
        {
            if (rayMaterial != null) Destroy(rayMaterial);
        }
    }
}
