using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Drives a character along the red MASt3R camera trajectory. The first press
/// traverses start-to-end; after arrival, the next press traverses end-to-start.
/// Attach this as an independent component and enable it explicitly.
/// </summary>
[DisallowMultipleComponent]
public sealed class Mast3rTrajectoryReplayController : MonoBehaviour
{
    [Header("Trajectory and character")]
    [Tooltip("RuntimePointCloudCollider that displays the red camera trajectory.")]
    public RuntimePointCloudCollider trajectorySource;
    [Tooltip("Character root to move. Empty uses trajectorySource.controlledRobot.")]
    public Transform controlledRobot;
    public CharacterController characterController;
    public Animator robotAnimator;

    [Header("Replay controls")]
    [Tooltip("Works with both legacy and new Input System because it is read through OnGUI.")]
    public KeyCode replayKey = KeyCode.T;
    public bool showReplayButton = true;
    [Tooltip("Place the character exactly at the correct endpoint before each traversal.")]
    public bool snapToEndpointBeforeEachRun = true;
    [Min(0.05f)] public float moveSpeed = 1.2f;
    [Min(1f)] public float rotationSpeedDegrees = 360f;
    [Min(0.005f)] public float waypointTolerance = 0.04f;
    [Tooltip("Discard nearly identical consecutive trajectory samples.")]
    [Min(0f)] public float minimumWaypointSpacing = 0.03f;
    [Min(0f)] public float downwardStickSpeed = 2f;

    [Header("HUD")]
    public float hudLeft = 500f;
    public float hudTop = 170f;
    public float hudWidth = 360f;
    public bool verboseLogs = true;

    public bool IsReplaying { get; private set; }
    public bool NextRunGoesForward { get; private set; } = true;
    public int CurrentWaypoint { get; private set; }
    public int WaypointCount => replayPath != null ? replayPath.Length : 0;

    private static readonly int AnimSpeed = Animator.StringToHash("Speed");
    private static readonly int AnimMotionSpeed = Animator.StringToHash("MotionSpeed");
    private static readonly int AnimGrounded = Animator.StringToHash("Grounded");
    private static readonly int AnimFreeFall = Animator.StringToHash("FreeFall");

    private Vector3[] replayPath = Array.Empty<Vector3>();
    private int direction = 1;
    private Behaviour manualMovement;
    private bool manualMovementWasEnabled;
    private bool clickToMoveWasEnabled;
    private bool capturedClickToMove;
    private string status = "Waiting for a red camera trajectory";

    private void OnEnable()
    {
        if (!Application.isPlaying) return;
        ResolveReferences();
        status = $"Ready. Press {replayKey} or the button to run start → end";
    }

    private void Update()
    {
        if (IsReplaying) AdvanceAlongTrajectory();
    }

    [ContextMenu("Replay / Return Along Camera Trajectory")]
    public void StartNextTraversal()
    {
        if (IsReplaying)
        {
            status = "Already replaying; wait until the endpoint";
            return;
        }

        ResolveReferences();
        if (trajectorySource == null || controlledRobot == null)
        {
            status = "Assign Trajectory Source and Controlled Robot";
            Debug.LogWarning($"[TrajectoryReplay] {status}");
            return;
        }
        if (!trajectorySource.isActiveAndEnabled)
        {
            status = "Trajectory Source component is disabled";
            return;
        }
        if (!trajectorySource.TryGetCameraTrajectoryWorld(out Vector3[] sourcePath))
        {
            status = "Waiting for at least two red trajectory points";
            return;
        }

        replayPath = BuildGroundPath(sourcePath, controlledRobot.position.y);
        if (replayPath.Length < 2)
        {
            status = "Trajectory is too short after filtering";
            return;
        }

        bool forward = NextRunGoesForward;
        direction = forward ? 1 : -1;
        int startIndex = forward ? 0 : replayPath.Length - 1;
        CurrentWaypoint = startIndex + direction;

        TakeCharacterControl();
        if (snapToEndpointBeforeEachRun) TeleportRobot(replayPath[startIndex]);
        IsReplaying = true;
        status = forward ? "Running toward trajectory end" : "Returning toward trajectory start";
        SetAnimation(moveSpeed);

        if (verboseLogs)
        {
            Debug.Log(
                $"[TrajectoryReplay] {status}; waypoints={replayPath.Length} " +
                $"speed={moveSpeed:F2} Unity units/s");
        }
    }

    [ContextMenu("Stop Trajectory Replay")]
    public void StopReplay()
    {
        StopReplayInternal(false);
    }

    private void AdvanceAlongTrajectory()
    {
        if (controlledRobot == null || replayPath == null || replayPath.Length < 2)
        {
            StopReplayInternal(false);
            return;
        }

        while (CurrentWaypoint >= 0 && CurrentWaypoint < replayPath.Length)
        {
            Vector3 offset = replayPath[CurrentWaypoint] - controlledRobot.position;
            offset.y = 0f;
            if (offset.magnitude > Mathf.Max(0.005f, waypointTolerance)) break;
            CurrentWaypoint += direction;
        }

        if (CurrentWaypoint < 0 || CurrentWaypoint >= replayPath.Length)
        {
            CompleteTraversal();
            return;
        }

        Vector3 difference = replayPath[CurrentWaypoint] - controlledRobot.position;
        difference.y = 0f;
        float remaining = difference.magnitude;
        if (remaining <= 0.00001f) return;

        Vector3 heading = difference / remaining;
        float step = Mathf.Min(Mathf.Max(0.05f, moveSpeed) * Time.deltaTime, remaining);
        Vector3 motion = heading * step;
        if (characterController != null && characterController.enabled)
        {
            motion += Vector3.down * Mathf.Max(0f, downwardStickSpeed) * Time.deltaTime;
            characterController.Move(motion);
        }
        else
        {
            controlledRobot.position += motion;
        }

        Quaternion targetRotation = Quaternion.LookRotation(heading, Vector3.up);
        controlledRobot.rotation = Quaternion.RotateTowards(
            controlledRobot.rotation,
            targetRotation,
            Mathf.Max(1f, rotationSpeedDegrees) * Time.deltaTime);
        SetAnimation(moveSpeed);
    }

    private void CompleteTraversal()
    {
        bool arrivedAtEnd = direction > 0;
        int endpoint = arrivedAtEnd ? replayPath.Length - 1 : 0;
        Vector3 finalPosition = replayPath[endpoint];
        finalPosition.y = controlledRobot.position.y;
        TeleportRobot(finalPosition);

        IsReplaying = false;
        NextRunGoesForward = !arrivedAtEnd;
        status = arrivedAtEnd
            ? "At trajectory end. Press again to return to start"
            : "At trajectory start. Press again to run to end";
        SetAnimation(0f);
        ReleaseCharacterControl();
        if (verboseLogs) Debug.Log($"[TrajectoryReplay] {status}");
    }

    private Vector3[] BuildGroundPath(Vector3[] source, float groundY)
    {
        List<Vector3> filtered = new List<Vector3>(source.Length);
        float spacing = Mathf.Max(0f, minimumWaypointSpacing);
        for (int i = 0; i < source.Length; i++)
        {
            Vector3 point = source[i];
            if (!IsFinite(point.x) || !IsFinite(point.y) || !IsFinite(point.z)) continue;
            point.y = groundY;
            if (filtered.Count > 0)
            {
                Vector3 delta = point - filtered[filtered.Count - 1];
                delta.y = 0f;
                if (delta.sqrMagnitude < spacing * spacing && i != source.Length - 1) continue;
            }
            filtered.Add(point);
        }
        return filtered.ToArray();
    }

    private void ResolveReferences()
    {
        if (trajectorySource == null) trajectorySource = GetComponent<RuntimePointCloudCollider>();
        if (controlledRobot == null && trajectorySource != null)
            controlledRobot = trajectorySource.controlledRobot;
        if (controlledRobot == null) return;
        if (characterController == null)
            characterController = controlledRobot.GetComponent<CharacterController>();
        if (robotAnimator == null)
            robotAnimator = controlledRobot.GetComponentInChildren<Animator>();
    }

    private void TakeCharacterControl()
    {
        trajectorySource.CancelNavigation();
        if (!capturedClickToMove)
        {
            clickToMoveWasEnabled = trajectorySource.enableClickToMove;
            capturedClickToMove = true;
        }
        trajectorySource.enableClickToMove = false;

        manualMovement = controlledRobot.GetComponent("ThirdPersonController") as Behaviour;
        manualMovementWasEnabled = manualMovement != null && manualMovement.enabled;
        if (manualMovementWasEnabled) manualMovement.enabled = false;
    }

    private void ReleaseCharacterControl()
    {
        if (manualMovement != null && manualMovementWasEnabled) manualMovement.enabled = true;
        manualMovement = null;
        manualMovementWasEnabled = false;
        if (capturedClickToMove && trajectorySource != null)
        {
            trajectorySource.enableClickToMove = clickToMoveWasEnabled;
            capturedClickToMove = false;
        }
    }

    private void TeleportRobot(Vector3 worldPosition)
    {
        bool controllerWasEnabled = characterController != null && characterController.enabled;
        if (controllerWasEnabled) characterController.enabled = false;
        controlledRobot.position = worldPosition;
        if (controllerWasEnabled) characterController.enabled = true;
    }

    private void SetAnimation(float speed)
    {
        if (robotAnimator == null) return;
        robotAnimator.SetFloat(AnimSpeed, speed);
        robotAnimator.SetFloat(AnimMotionSpeed, speed > 0f ? 1f : 0f);
        robotAnimator.SetBool(AnimGrounded, true);
        robotAnimator.SetBool(AnimFreeFall, false);
    }

    private void StopReplayInternal(bool disabling)
    {
        IsReplaying = false;
        SetAnimation(0f);
        ReleaseCharacterControl();
        status = disabling
            ? "Trajectory replay disabled"
            : "Stopped; press again to restart this direction";
    }

    private void OnGUI()
    {
        Event current = Event.current;
        if (current != null && current.type == EventType.KeyDown && current.keyCode == replayKey)
        {
            StartNextTraversal();
            current.Use();
        }

        if (!showReplayButton) return;
        Rect area = new Rect(hudLeft, hudTop, Mathf.Max(260f, hudWidth), 96f);
        GUILayout.BeginArea(area, GUI.skin.box);
        GUILayout.Label($"[Trajectory Replay] {status}");
        GUILayout.Label($"Waypoints: {WaypointCount}  Key: {replayKey}");
        bool previousGuiEnabled = GUI.enabled;
        GUI.enabled = previousGuiEnabled && !IsReplaying;
        string label = NextRunGoesForward
            ? "Run to trajectory END"
            : "Return to trajectory START";
        if (GUILayout.Button(label)) StartNextTraversal();
        GUI.enabled = previousGuiEnabled;
        GUILayout.EndArea();
    }

    private void OnDisable()
    {
        StopReplayInternal(true);
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
