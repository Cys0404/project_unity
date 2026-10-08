using System;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

/// <summary>
/// Runtime two-point ruler for procedurally rendered MASt3R point clouds.
/// It picks directly from immutable CPU point arrays on a worker thread, so no
/// per-point GameObjects or colliders are required.
/// </summary>
[DisallowMultipleComponent]
public sealed class PointCloudMeasurementTool : MonoBehaviour
{
    [Header("Point Cloud and Camera")]
    public RuntimePointCloudCollider pointCloudSource;
    [Tooltip("Camera used for point picking. Empty uses Camera.main.")]
    public Camera measurementCamera;

    [Header("Controls")]
    [Tooltip("Press this key in Play Mode to enter or leave measurement mode.")]
    public KeyCode toggleKey = KeyCode.M;
    [Tooltip("Maximum cursor-to-point distance in screen pixels.")]
    [Range(2f, 40f)] public float pickRadiusPixels = 12f;
    [Tooltip("Zero scans every point. A positive value evenly samples very large maps per click.")]
    [Min(0)] public int maximumPointsToTest = 3000000;
    [Tooltip("Reject points farther than this world-space distance from the camera. Zero disables the limit.")]
    [Min(0f)] public float maximumPickDistance = 100f;
    [Tooltip("Unity world units represented by one metre.")]
    [Min(0.000001f)] public float unityUnitsPerMetre = 1f;
    public bool startInMeasurementMode = false;

    [Header("Appearance")]
    public Color firstPointColor = new Color(0.1f, 1f, 0.25f, 1f);
    public Color secondPointColor = new Color(1f, 0.25f, 0.1f, 1f);
    public Color lineColor = new Color(1f, 0.9f, 0.1f, 1f);
    [Min(0.001f)] public float markerDiameterMetres = 0.06f;
    [Min(0.001f)] public float lineWidthMetres = 0.015f;

    [Header("HUD")]
    public bool showHud = true;
    public float hudLeft = 500f;
    public float hudTop = 10f;
    public float hudWidth = 390f;
    public bool verboseLogs = true;

    public bool MeasurementMode { get; private set; }
    public bool HasFirstPoint { get; private set; }
    public bool HasSecondPoint { get; private set; }
    public Vector3 FirstPointWorld { get; private set; }
    public Vector3 SecondPointWorld { get; private set; }
    public float DistanceMetres =>
        HasFirstPoint && HasSecondPoint
            ? Vector3.Distance(FirstPointWorld, SecondPointWorld) /
              Mathf.Max(0.000001f, unityUnitsPerMetre)
            : 0f;

    private sealed class PickResult
    {
        public bool found;
        public int revision;
        public Vector3 worldPoint;
        public float screenDistancePixels;
        public long testedPoints;
        public long sourcePoints;
    }

    private Task<PickResult> pickTask;
    private CancellationTokenSource pickCancellation;
    private int pickGeneration;
    private string status = "Press M to measure";
    private int firstPointRevision = -1;
    private GameObject visualRoot;
    private GameObject firstMarker;
    private GameObject secondMarker;
    private LineRenderer rulerLine;
    private Material firstMarkerMaterial;
    private Material secondMarkerMaterial;
    private Material lineMaterial;
    private bool clickToMoveWasEnabled;
    private bool capturedClickToMove;

    private void OnEnable()
    {
        if (!Application.isPlaying) return;
        if (pointCloudSource == null) pointCloudSource = GetComponent<RuntimePointCloudCollider>();
        if (visualRoot == null) CreateVisuals();
        if (visualRoot != null) visualRoot.SetActive(true);
        SetMeasurementMode(startInMeasurementMode);
    }

    private void Update()
    {
        FinishPickIfReady();
        UpdateVisuals();
    }

    private void OnGUI()
    {
        HandleInputEvent(Event.current);
        if (!showHud) return;

        float height = HasSecondPoint ? 150f : 112f;
        Rect area = new Rect(hudLeft, hudTop, Mathf.Max(300f, hudWidth), height);
        GUILayout.BeginArea(area, GUI.skin.box);
        Color oldColor = GUI.color;
        GUI.color = MeasurementMode ? Color.yellow : Color.white;
        GUILayout.Label($"[Point Cloud Ruler] {(MeasurementMode ? "ON" : "OFF")}  ({toggleKey}: toggle)");
        GUI.color = oldColor;
        GUILayout.Label(status);

        if (HasFirstPoint)
        {
            GUILayout.Label($"A: {FormatPoint(FirstPointWorld)} m");
        }
        if (HasSecondPoint)
        {
            Vector3 deltaMetres = (SecondPointWorld - FirstPointWorld) /
                                  Mathf.Max(0.000001f, unityUnitsPerMetre);
            GUILayout.Label($"B: {FormatPoint(SecondPointWorld)} m");
            GUILayout.Label(
                $"Distance: {DistanceMetres:F4} m  ({DistanceMetres * 100f:F2} cm)");
            GUILayout.Label(
                $"ΔX={Mathf.Abs(deltaMetres.x):F4}  " +
                $"ΔY={Mathf.Abs(deltaMetres.y):F4}  " +
                $"ΔZ={Mathf.Abs(deltaMetres.z):F4} m");
        }
        GUILayout.EndArea();

        DrawWorldLabel();
    }

    private void HandleInputEvent(Event current)
    {
        if (current == null) return;

        if (current.type == EventType.KeyDown && current.keyCode == toggleKey)
        {
            SetMeasurementMode(!MeasurementMode);
            current.Use();
            return;
        }

        if (!MeasurementMode) return;
        if (current.type == EventType.KeyDown && current.keyCode == KeyCode.Escape)
        {
            ClearMeasurement();
            current.Use();
            return;
        }
        if (current.type != EventType.MouseDown) return;
        if (current.button == 1)
        {
            ClearMeasurement();
            current.Use();
            return;
        }
        if (current.button != 0 || pickTask != null) return;

        Camera camera = ResolveCamera();
        if (camera == null)
        {
            status = "No measurement camera; assign one or tag the Game camera MainCamera";
            return;
        }

        Vector2 guiPosition = current.mousePosition;
        Vector2 screenPosition = new Vector2(guiPosition.x, Screen.height - guiPosition.y);
        BeginPick(camera, screenPosition);
        current.Use();
    }

    private void BeginPick(Camera camera, Vector2 screenPosition)
    {
        if (pointCloudSource == null || !pointCloudSource.isActiveAndEnabled ||
            !pointCloudSource.TryCreatePointPickSnapshot(
                out RuntimePointCloudCollider.PointPickSnapshot snapshot))
        {
            status = "Waiting for a complete point-cloud map";
            return;
        }

        Matrix4x4 viewProjection = camera.projectionMatrix * camera.worldToCameraMatrix;
        Vector3 cameraPosition = camera.transform.position;
        int screenWidth = Screen.width;
        int screenHeight = Screen.height;
        float radius = Mathf.Max(2f, pickRadiusPixels);
        int pointLimit = Mathf.Max(0, maximumPointsToTest);
        float distanceLimit = Mathf.Max(0f, maximumPickDistance);
        int generation = ++pickGeneration;

        pickCancellation?.Cancel();
        pickCancellation?.Dispose();
        pickCancellation = new CancellationTokenSource();
        CancellationToken token = pickCancellation.Token;
        status = $"Picking from revision {snapshot.revision}...";

        pickTask = Task.Run(
            () => FindClosestPoint(
                snapshot,
                viewProjection,
                cameraPosition,
                screenPosition,
                screenWidth,
                screenHeight,
                radius,
                pointLimit,
                distanceLimit,
                token),
            token);

        if (verboseLogs)
        {
            Debug.Log(
                $"[PointCloudRuler] Pick #{generation} rev={snapshot.revision} " +
                $"source={snapshot.pointCount:N0} cursor={screenPosition}");
        }
    }

    private void FinishPickIfReady()
    {
        if (pickTask == null || !pickTask.IsCompleted) return;

        Task<PickResult> completed = pickTask;
        pickTask = null;
        if (completed.IsCanceled) return;
        if (completed.IsFaulted)
        {
            Exception exception = completed.Exception != null
                ? completed.Exception.GetBaseException()
                : null;
            status = $"Pick failed: {exception?.Message}";
            Debug.LogError($"[PointCloudRuler] {status}");
            return;
        }

        PickResult result = completed.Result;
        if (!result.found)
        {
            status =
                $"No point within {pickRadiusPixels:F0} px " +
                $"({result.testedPoints:N0}/{result.sourcePoints:N0} tested)";
            return;
        }

        if (!HasFirstPoint || HasSecondPoint || firstPointRevision != result.revision)
        {
            FirstPointWorld = result.worldPoint;
            firstPointRevision = result.revision;
            HasFirstPoint = true;
            HasSecondPoint = false;
            status =
                $"A selected (rev {result.revision}, {result.screenDistancePixels:F1} px). " +
                "Left-click B; right-click/Esc clears.";
        }
        else
        {
            SecondPointWorld = result.worldPoint;
            HasSecondPoint = true;
            status =
                $"Measured {DistanceMetres:F4} m. Left-click starts a new measurement; " +
                "right-click/Esc clears.";
            if (verboseLogs)
            {
                Debug.Log(
                    $"[PointCloudRuler] A={FirstPointWorld} B={SecondPointWorld} " +
                    $"distance={DistanceMetres:F6}m revision={result.revision}");
            }
        }
        UpdateVisuals();
    }

    private static PickResult FindClosestPoint(
        RuntimePointCloudCollider.PointPickSnapshot snapshot,
        Matrix4x4 viewProjection,
        Vector3 cameraPosition,
        Vector2 screenPosition,
        int screenWidth,
        int screenHeight,
        float radiusPixels,
        int maximumPoints,
        float maximumDistance,
        CancellationToken token)
    {
        PickResult result = new PickResult
        {
            revision = snapshot.revision,
            sourcePoints = snapshot.pointCount,
        };
        if (screenWidth <= 0 || screenHeight <= 0) return result;

        long stride = maximumPoints > 0 && snapshot.pointCount > maximumPoints
            ? Math.Max(1L, (long)Math.Ceiling(snapshot.pointCount / (double)maximumPoints))
            : 1L;
        float radiusSquared = radiusPixels * radiusPixels;
        float maximumDistanceSquared = maximumDistance > 0f
            ? maximumDistance * maximumDistance
            : float.PositiveInfinity;
        float bestScore = float.PositiveInfinity;
        long globalIndex = 0L;

        for (int chunkIndex = 0; chunkIndex < snapshot.chunks.Length; chunkIndex++)
        {
            Vector3[] chunk = snapshot.chunks[chunkIndex];
            if (chunk == null) continue;
            Matrix4x4 localToWorld = snapshot.chunkToWorld[chunkIndex];
            for (int i = 0; i < chunk.Length; i++, globalIndex++)
            {
                if (globalIndex % stride != 0L) continue;
                if ((result.testedPoints & 0x3FFF) == 0) token.ThrowIfCancellationRequested();
                result.testedPoints++;

                Vector3 world = TransformPoint(localToWorld, chunk[i]);
                float dx = world.x - cameraPosition.x;
                float dy = world.y - cameraPosition.y;
                float dz = world.z - cameraPosition.z;
                float distanceSquared = dx * dx + dy * dy + dz * dz;
                if (distanceSquared > maximumDistanceSquared) continue;

                float clipX =
                    viewProjection.m00 * world.x + viewProjection.m01 * world.y +
                    viewProjection.m02 * world.z + viewProjection.m03;
                float clipY =
                    viewProjection.m10 * world.x + viewProjection.m11 * world.y +
                    viewProjection.m12 * world.z + viewProjection.m13;
                float clipW =
                    viewProjection.m30 * world.x + viewProjection.m31 * world.y +
                    viewProjection.m32 * world.z + viewProjection.m33;
                if (clipW <= 0.000001f) continue;

                float pixelX = (clipX / clipW * 0.5f + 0.5f) * screenWidth;
                float pixelY = (clipY / clipW * 0.5f + 0.5f) * screenHeight;
                float pixelDx = pixelX - screenPosition.x;
                float pixelDy = pixelY - screenPosition.y;
                float screenDistanceSquared = pixelDx * pixelDx + pixelDy * pixelDy;
                if (screenDistanceSquared > radiusSquared) continue;

                // Screen alignment dominates; a tiny depth term resolves nearly
                // identical projected points in favour of the visible surface.
                float score = screenDistanceSquared + Mathf.Sqrt(distanceSquared) * 0.001f;
                if (score >= bestScore) continue;
                bestScore = score;
                result.found = true;
                result.worldPoint = world;
                result.screenDistancePixels = Mathf.Sqrt(screenDistanceSquared);
            }
        }

        return result;
    }

    private void SetMeasurementMode(bool enabled)
    {
        MeasurementMode = enabled;
        if (pointCloudSource != null)
        {
            if (enabled)
            {
                if (!capturedClickToMove)
                {
                    clickToMoveWasEnabled = pointCloudSource.enableClickToMove;
                    capturedClickToMove = true;
                }
                pointCloudSource.enableClickToMove = false;
            }
            else if (capturedClickToMove)
            {
                pointCloudSource.enableClickToMove = clickToMoveWasEnabled;
                capturedClickToMove = false;
            }
        }

        if (enabled)
        {
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
            status = "Left-click point A; right-click/Esc clears";
        }
        else
        {
            CancelPick();
            status = $"Press {toggleKey} to measure";
        }
    }

    [ContextMenu("Clear Measurement")]
    public void ClearMeasurement()
    {
        CancelPick();
        HasFirstPoint = false;
        HasSecondPoint = false;
        firstPointRevision = -1;
        status = MeasurementMode
            ? "Cleared. Left-click point A"
            : $"Press {toggleKey} to measure";
        UpdateVisuals();
    }

    private void CancelPick()
    {
        pickGeneration++;
        if (pickCancellation != null)
        {
            pickCancellation.Cancel();
            pickCancellation.Dispose();
            pickCancellation = null;
        }
        pickTask = null;
    }

    private Camera ResolveCamera()
    {
        if (measurementCamera != null && measurementCamera.isActiveAndEnabled)
            return measurementCamera;
        Camera main = Camera.main;
        if (main != null && main.isActiveAndEnabled) return main;
        Camera[] cameras = Camera.allCameras;
        foreach (Camera camera in cameras)
        {
            if (camera != null && camera.isActiveAndEnabled &&
                camera.cameraType == CameraType.Game) return camera;
        }
        return null;
    }

    private void CreateVisuals()
    {
        if (visualRoot != null) return;
        visualRoot = new GameObject("PointCloudMeasurementVisuals");
        visualRoot.transform.position = Vector3.zero;
        visualRoot.transform.rotation = Quaternion.identity;
        visualRoot.transform.localScale = Vector3.one;

        firstMarker = CreateMarker("MeasurePointA", firstPointColor, out firstMarkerMaterial);
        secondMarker = CreateMarker("MeasurePointB", secondPointColor, out secondMarkerMaterial);

        GameObject lineObject = new GameObject("MeasureLine");
        lineObject.transform.SetParent(visualRoot.transform, false);
        rulerLine = lineObject.AddComponent<LineRenderer>();
        rulerLine.useWorldSpace = true;
        rulerLine.positionCount = 2;
        rulerLine.numCapVertices = 4;
        rulerLine.numCornerVertices = 2;
        rulerLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        rulerLine.receiveShadows = false;
        lineMaterial = CreateColorMaterial("Point Cloud Ruler Line", lineColor);
        if (lineMaterial != null) rulerLine.sharedMaterial = lineMaterial;
        rulerLine.startColor = lineColor;
        rulerLine.endColor = lineColor;
        rulerLine.enabled = false;
    }

    private GameObject CreateMarker(string markerName, Color color, out Material material)
    {
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = markerName;
        marker.transform.SetParent(visualRoot.transform, true);
        Collider collider = marker.GetComponent<Collider>();
        if (collider != null) Destroy(collider);
        material = CreateColorMaterial(markerName + " Material", color);
        Renderer renderer = marker.GetComponent<Renderer>();
        if (renderer != null && material != null) renderer.sharedMaterial = material;
        marker.SetActive(false);
        return marker;
    }

    private static Material CreateColorMaterial(string materialName, Color color)
    {
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null) shader = Shader.Find("Sprites/Default");
        if (shader == null) shader = Shader.Find("Unlit/Color");
        if (shader == null) shader = Shader.Find("Standard");
        if (shader == null) return null;
        Material material = new Material(shader) { name = materialName, color = color };
        if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
        if (material.HasProperty("_Color")) material.SetColor("_Color", color);
        return material;
    }

    private void UpdateVisuals()
    {
        float unitsPerMetre = Mathf.Max(0.000001f, unityUnitsPerMetre);
        float markerScale = Mathf.Max(0.001f, markerDiameterMetres * unitsPerMetre);
        float lineWidth = Mathf.Max(0.001f, lineWidthMetres * unitsPerMetre);

        if (firstMarker != null)
        {
            firstMarker.SetActive(HasFirstPoint);
            firstMarker.transform.position = FirstPointWorld;
            firstMarker.transform.localScale = Vector3.one * markerScale;
        }
        if (secondMarker != null)
        {
            secondMarker.SetActive(HasSecondPoint);
            secondMarker.transform.position = SecondPointWorld;
            secondMarker.transform.localScale = Vector3.one * markerScale;
        }
        if (rulerLine != null)
        {
            rulerLine.enabled = HasFirstPoint && HasSecondPoint;
            rulerLine.startWidth = lineWidth;
            rulerLine.endWidth = lineWidth;
            if (rulerLine.enabled)
            {
                rulerLine.SetPosition(0, FirstPointWorld);
                rulerLine.SetPosition(1, SecondPointWorld);
            }
        }
    }

    private void DrawWorldLabel()
    {
        if (!HasFirstPoint || !HasSecondPoint) return;
        Camera camera = ResolveCamera();
        if (camera == null) return;
        Vector3 screen = camera.WorldToScreenPoint((FirstPointWorld + SecondPointWorld) * 0.5f);
        if (screen.z <= 0f) return;
        string text = $"{DistanceMetres:F3} m\n{DistanceMetres * 100f:F1} cm";
        Vector2 size = GUI.skin.box.CalcSize(new GUIContent(text));
        GUI.Box(
            new Rect(
                screen.x - size.x * 0.5f - 6f,
                Screen.height - screen.y - size.y * 0.5f - 4f,
                size.x + 12f,
                size.y + 8f),
            text);
    }

    private string FormatPoint(Vector3 worldPoint)
    {
        Vector3 metres = worldPoint / Mathf.Max(0.000001f, unityUnitsPerMetre);
        return $"({metres.x:F3}, {metres.y:F3}, {metres.z:F3})";
    }

    private static Vector3 TransformPoint(Matrix4x4 matrix, Vector3 point)
    {
        return new Vector3(
            matrix.m00 * point.x + matrix.m01 * point.y + matrix.m02 * point.z + matrix.m03,
            matrix.m10 * point.x + matrix.m11 * point.y + matrix.m12 * point.z + matrix.m13,
            matrix.m20 * point.x + matrix.m21 * point.y + matrix.m22 * point.z + matrix.m23);
    }

    private void OnDisable()
    {
        SetMeasurementMode(false);
        ClearMeasurement();
        if (visualRoot != null) visualRoot.SetActive(false);
    }

    private void OnDestroy()
    {
        CancelPick();
        if (visualRoot != null) Destroy(visualRoot);
        if (firstMarkerMaterial != null) Destroy(firstMarkerMaterial);
        if (secondMarkerMaterial != null) Destroy(secondMarkerMaterial);
        if (lineMaterial != null) Destroy(lineMaterial);
    }
}
