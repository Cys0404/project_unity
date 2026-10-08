using System;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Serialization;

/// <summary>
/// Creates one simple ground collider under the reconstructed point cloud,
/// renders a clickable top-down map, and moves a character directly to the
/// clicked position. Point-cloud geometry itself never participates in physics.
/// </summary>
public sealed class RuntimePointCloudCollider : MonoBehaviour
{
    [Header("Ground Base")]
    [Tooltip("Percentile used as the floor height. 0.05 means the lowest five-percent level of sampled points.")]
    [Range(0f, 0.25f)] public float groundHeightPercentile = 0.05f;
    [Tooltip("Trim this fraction from both sides of X/Z bounds to reject far spatial outliers.")]
    [Range(0f, 0.1f)] public float boundsTrimPercentile = 0.01f;
    [Tooltip("Extra point-cloud units added around all four sides of the ground base.")]
    [Min(0f)] public float groundPadding = 1f;
    [Tooltip("Thickness of the BoxCollider used as the ground plane.")]
    [Min(0.01f)] public float groundThickness = 0.10f;
    public bool enableGroundCollider = true;
    [Tooltip("Additional vertical adjustment applied to the estimated floor height.")]
    public float groundHeightOffset = 0f;
    [Tooltip("Maximum point samples used to estimate bounds and ground height.")]
    [Min(1000)] public int maximumGroundSamples = 200000;
    [Range(0, 31)] public int collisionLayer = 0;
    public bool showGroundBase = true;
    public Color groundBaseColor = new Color(0.12f, 0.14f, 0.16f, 1f);

    [Header("Top-Down 2D Map")]
    public bool showMiniMap = true;
    public string miniMapTitle = "2D Point Cloud Map";
    [Range(128, 1024)] public int miniMapResolution = 512;
    [Min(100f)] public float miniMapSizePixels = 240f;
    [Min(0f)] public float miniMapLeft = 10f;
    [Min(0f)] public float miniMapTop = 230f;
    [Tooltip("Anchor the mini-map to the right edge; miniMapLeft becomes the right margin.")]
    public bool alignMiniMapToRight = false;
    [Tooltip("Highest point height included in the RGB top-down map, measured above the estimated floor. Change this while Play Mode is running to move the view below the ceiling.")]
    [FormerlySerializedAs("miniMapSliceMaxHeight")]
    [Min(0.01f)] public float miniMapViewHeightAboveGround = 1.80f;
    [Tooltip("Wait briefly after the height changes before rebuilding, so dragging the Inspector value does not queue many full point-cloud projections.")]
    [Range(0.05f, 1f)] public float miniMapHeightRebuildDelay = 0.20f;
    [Range(0, 3)] public int miniMapPointRadiusPixels = 1;
    public Color miniMapBackground = new Color(0.025f, 0.035f, 0.05f, 1f);
    public Color miniMapPointColor = new Color(0.78f, 0.88f, 1f, 1f);
    [Tooltip("Darken lower surfaces slightly so furniture and wall height remain readable from above.")]
    [Range(0f, 0.75f)] public float miniMapHeightShadingStrength = 0.25f;
    [Tooltip("Draw a dark contour where neighbouring projected surfaces have a large height difference.")]
    public bool miniMapShowHeightEdges = true;
    [Min(0.02f)] public float miniMapHeightEdgeThreshold = 0.18f;
    [Range(0f, 1f)] public float miniMapHeightEdgeDarkening = 0.45f;
    public bool showCameraTrajectory = true;
    public Color cameraTrajectoryColor = new Color(1f, 0.2f, 0.15f, 1f);
    [Range(1f, 8f)] public float cameraTrajectoryWidthPixels = 3f;
    [Min(10)] public int maximumMiniMapTrajectorySegments = 1000;
    [Tooltip("Keep previous X/Z bounds when maps update, preventing the mini-map and trajectory from jumping as bounds are re-estimated.")]
    public bool stabilizeMiniMapBounds = true;
    [Tooltip("Unlock the cursor and disable Starter Assets mouse-look while the clickable map is shown.")]
    public bool unlockCursorForMiniMap = true;

    [Header("Click To Move")]
    public bool enableClickToMove = true;
    [Tooltip("Optional character root. When empty, a Starter Assets ThirdPersonController is found automatically.")]
    public Transform controlledRobot;
    public CharacterController robotCharacterController;
    [Min(0.1f)] public float moveSpeed = 2.5f;
    [Min(1f)] public float rotationSpeedDegrees = 540f;
    [Min(0.05f)] public float stoppingDistance = 0.25f;
    [Min(0f)] public float downwardStickSpeed = 2f;
    [Tooltip("Place the character on the first generated ground base automatically.")]
    public bool autoPlaceRobotOnFirstGround = true;
    [Tooltip("Keep the initial X/Z position when it is inside the map; otherwise use the map center.")]
    public bool keepInitialPositionWhenInsideMap = true;
    [Min(0f)] public float robotGroundClearance = 0.05f;

    [Header("Starter Assets Safety")]
    [Tooltip("Pause the Starter Assets controller until the first ground base is ready.")]
    public bool holdStarterPlayerUntilFirstGround = true;
    public bool verboseLogs = true;

    public string StatusText { get; private set; } = "waiting for a complete map";
    public int ActiveRevision { get; private set; } = -1;
    public Vector3 NavigationDestination { get; private set; }
    public bool IsNavigating { get; private set; }

    /// <summary>
    /// A shallow, immutable view of the latest CPU point arrays for tools such
    /// as runtime measurement. The inner point arrays are shared intentionally:
    /// map decoders never mutate them after publication.
    /// </summary>
    public sealed class PointPickSnapshot
    {
        public int revision;
        public Vector3[][] chunks;
        public Matrix4x4[] chunkToWorld;
        public long pointCount;
    }

    private sealed class CloudSnapshot
    {
        public int revision;
        public Vector3[][] chunks;
        public uint[][] colorChunks;
        public Matrix4x4[] chunkToMap;
        public Matrix4x4 mapToWorld;
        public bool hasStableBounds;
        public float stableMinX;
        public float stableMaxX;
        public float stableMinZ;
        public float stableMaxZ;
    }

    private sealed class GroundData
    {
        public int revision;
        public int sourcePointCount;
        public int sampleCount;
        public float minX;
        public float maxX;
        public float minY;
        public float maxY;
        public float minZ;
        public float maxZ;
        public float groundY;
        public float mapCutoffY;
        public int mapResolution;
        public byte[] mapOccupancy;
        public uint[] mapColors;
        public float[] mapHeights;
    }

    private Matrix4x4 mapToWorldMatrix = Matrix4x4.identity;
    private CloudSnapshot pendingSnapshot;
    private CloudSnapshot latestSourceSnapshot;
    private Task<GroundData> groundTask;
    private GameObject groundObject;
    private BoxCollider groundCollider;
    private Material groundMaterial;
    private Texture2D miniMapTexture;
    public Texture2D MiniMapTexture => miniMapTexture;
    public bool TryWorldToMiniMap(Vector3 world, out Vector2 uv)
    {
        uv = Vector2.zero;
        if (activeGroundData == null) return false;
        uv = new Vector2(Mathf.InverseLerp(activeGroundData.minX, activeGroundData.maxX, world.x),
            Mathf.InverseLerp(activeGroundData.minZ, activeGroundData.maxZ, world.z));
        return true;
    }
    // Uses the same navigation and terrain semantics as the existing desktop minimap.
    public bool NavigateMiniMap(Vector2 uv)
    {
        if (!enableClickToMove || activeGroundData == null || uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1) return false;
        SetNavigationDestination(new Vector3(Mathf.Lerp(activeGroundData.minX, activeGroundData.maxX, uv.x),
            activeGroundData.groundY + groundHeightOffset + robotGroundClearance,
            Mathf.Lerp(activeGroundData.minZ, activeGroundData.maxZ, uv.y)));
        return true;
    }
    private GroundData activeGroundData;
    private Vector3[] cameraTrajectoryMapPoints = Array.Empty<Vector3>();
    private Behaviour starterMovement;
    private bool starterMovementWasEnabled;
    private Animator robotAnimator;
    private Component starterAssetsInputs;
    private bool starterInputSettingsCaptured;
    private bool starterCursorLockedWasEnabled;
    private bool starterCursorLookWasEnabled;
    private float observedMiniMapViewHeight = float.NaN;
    private float miniMapHeightChangedAt;
    private bool miniMapHeightRebuildPending;

    private static readonly int AnimSpeed = Animator.StringToHash("Speed");
    private static readonly int AnimMotionSpeed = Animator.StringToHash("MotionSpeed");
    private static readonly int AnimGrounded = Animator.StringToHash("Grounded");
    private static readonly int AnimFreeFall = Animator.StringToHash("FreeFall");

    private void Start()
    {
        FindControlledRobot();
        TryHoldStarterPlayer();
    }

    public void SetMapSpace(Transform value)
    {
        mapToWorldMatrix = value != null ? value.localToWorldMatrix : Matrix4x4.identity;
    }

    public void SetMapMatrix(Matrix4x4 value)
    {
        mapToWorldMatrix = value;
    }

    public void SetCameraTrajectory(Vector3[] mapPoints)
    {
        if (mapPoints == null || mapPoints.Length == 0)
        {
            cameraTrajectoryMapPoints = Array.Empty<Vector3>();
            return;
        }

        cameraTrajectoryMapPoints = new Vector3[mapPoints.Length];
        Array.Copy(mapPoints, cameraTrajectoryMapPoints, mapPoints.Length);
    }

    /// <summary>
    /// Copies the same red camera trajectory used by the mini-map into world
    /// space. Callers own the returned array, so a live ROS update cannot alter
    /// a traversal already in progress.
    /// </summary>
    public bool TryGetCameraTrajectoryWorld(out Vector3[] worldPoints)
    {
        if (cameraTrajectoryMapPoints == null || cameraTrajectoryMapPoints.Length < 2)
        {
            worldPoints = Array.Empty<Vector3>();
            return false;
        }

        Matrix4x4 mapToWorld = mapToWorldMatrix;
        worldPoints = new Vector3[cameraTrajectoryMapPoints.Length];
        for (int i = 0; i < cameraTrajectoryMapPoints.Length; i++)
            worldPoints[i] = TransformPoint(mapToWorld, cameraTrajectoryMapPoints[i]);
        return true;
    }

    /// <summary>Stops click-to-move before another controller takes ownership.</summary>
    public void CancelNavigation()
    {
        StopNavigation();
    }

    /// <summary>
    /// Returns the latest point cloud without copying its point payload. One
    /// transform per chunk is captured so a background picker never touches a
    /// live Unity Transform or a changing keyframe pose.
    /// </summary>
    public bool TryCreatePointPickSnapshot(out PointPickSnapshot snapshot)
    {
        CloudSnapshot source = latestSourceSnapshot;
        if (source == null || source.chunks == null || source.chunks.Length == 0)
        {
            snapshot = null;
            return false;
        }

        Vector3[][] chunks = (Vector3[][])source.chunks.Clone();
        Matrix4x4[] chunkToWorld = new Matrix4x4[chunks.Length];
        long pointCount = 0L;
        for (int i = 0; i < chunks.Length; i++)
        {
            Matrix4x4 localToMap =
                source.chunkToMap != null && i < source.chunkToMap.Length
                    ? source.chunkToMap[i]
                    : Matrix4x4.identity;
            chunkToWorld[i] = source.mapToWorld * localToMap;
            if (chunks[i] != null) pointCount += chunks[i].LongLength;
        }

        if (pointCount <= 0L)
        {
            snapshot = null;
            return false;
        }

        snapshot = new PointPickSnapshot
        {
            revision = source.revision,
            chunks = chunks,
            chunkToWorld = chunkToWorld,
            pointCount = pointCount,
        };
        return true;
    }

    public void RequestRebuild(int revision, Vector3[][] pointChunks)
    {
        RequestRebuild(revision, pointChunks, null);
    }

    public void RequestRebuild(int revision, Vector3[][] pointChunks, uint[][] colorChunks)
    {
        RequestRebuildTransformed(revision, pointChunks, colorChunks, null);
    }

    /// <summary>
    /// Rebuild from immutable local point arrays and one local-to-map matrix per
    /// chunk.  Delta maps can therefore update keyframe poses without allocating
    /// a second, fully transformed copy of every point.
    /// </summary>
    public void RequestRebuildTransformed(
        int revision,
        Vector3[][] pointChunks,
        Matrix4x4[] chunkToMap)
    {
        RequestRebuildTransformed(revision, pointChunks, null, chunkToMap);
    }

    public void RequestRebuildTransformed(
        int revision,
        Vector3[][] pointChunks,
        uint[][] colorChunks,
        Matrix4x4[] chunkToMap)
    {
        if (!isActiveAndEnabled || pointChunks == null || pointChunks.Length == 0) return;

        Vector3[][] snapshotChunks = new Vector3[pointChunks.Length][];
        int nonEmpty = 0;
        for (int i = 0; i < pointChunks.Length; i++)
        {
            snapshotChunks[i] = pointChunks[i];
            if (snapshotChunks[i] != null && snapshotChunks[i].Length > 0) nonEmpty++;
        }
        if (nonEmpty == 0) return;

        CloudSnapshot snapshot = new CloudSnapshot
        {
            revision = revision,
            chunks = snapshotChunks,
            colorChunks = colorChunks != null ? (uint[][])colorChunks.Clone() : null,
            chunkToMap = chunkToMap != null ? (Matrix4x4[])chunkToMap.Clone() : null,
            mapToWorld = mapToWorldMatrix,
            hasStableBounds = stabilizeMiniMapBounds && activeGroundData != null,
            stableMinX = activeGroundData != null ? activeGroundData.minX : 0f,
            stableMaxX = activeGroundData != null ? activeGroundData.maxX : 0f,
            stableMinZ = activeGroundData != null ? activeGroundData.minZ : 0f,
            stableMaxZ = activeGroundData != null ? activeGroundData.maxZ : 0f,
        };
        latestSourceSnapshot = snapshot;
        pendingSnapshot = snapshot;
        observedMiniMapViewHeight = Mathf.Max(0.01f, miniMapViewHeightAboveGround);
        miniMapHeightRebuildPending = false;
        StatusText = $"ground revision {revision} queued";
        TryStartGroundBuild();
    }

    private void Update()
    {
        if (controlledRobot == null) FindControlledRobot();
        if (ActiveRevision < 0) TryHoldStarterPlayer();

        FinishGroundBuildIfReady();
        QueueManualHeightRebuildIfNeeded();
        if (groundTask == null) TryStartGroundBuild();
        UpdateMiniMapCursorState();
        UpdateClickNavigation();
    }

    private void QueueManualHeightRebuildIfNeeded()
    {
        float height = Mathf.Max(0.01f, miniMapViewHeightAboveGround);
        if (float.IsNaN(observedMiniMapViewHeight))
        {
            observedMiniMapViewHeight = height;
        }
        else if (Mathf.Abs(height - observedMiniMapViewHeight) > 0.0001f)
        {
            observedMiniMapViewHeight = height;
            miniMapHeightChangedAt = Time.realtimeSinceStartup;
            miniMapHeightRebuildPending = true;
            StatusText = $"RGB map height changed to {height:F2}; waiting to rebuild";
        }

        if (!miniMapHeightRebuildPending || latestSourceSnapshot == null) return;
        if (Time.realtimeSinceStartup - miniMapHeightChangedAt <
            Mathf.Max(0.05f, miniMapHeightRebuildDelay)) return;

        // The point/color arrays are immutable after decode. Clone only the
        // small outer arrays so changing the height does not duplicate the map.
        pendingSnapshot = new CloudSnapshot
        {
            revision = latestSourceSnapshot.revision,
            chunks = (Vector3[][])latestSourceSnapshot.chunks.Clone(),
            colorChunks = latestSourceSnapshot.colorChunks != null
                ? (uint[][])latestSourceSnapshot.colorChunks.Clone()
                : null,
            chunkToMap = latestSourceSnapshot.chunkToMap != null
                ? (Matrix4x4[])latestSourceSnapshot.chunkToMap.Clone()
                : null,
            mapToWorld = latestSourceSnapshot.mapToWorld,
            hasStableBounds = stabilizeMiniMapBounds && activeGroundData != null,
            stableMinX = activeGroundData != null ? activeGroundData.minX : 0f,
            stableMaxX = activeGroundData != null ? activeGroundData.maxX : 0f,
            stableMinZ = activeGroundData != null ? activeGroundData.minZ : 0f,
            stableMaxZ = activeGroundData != null ? activeGroundData.maxZ : 0f,
        };
        miniMapHeightRebuildPending = false;
        StatusText = $"rebuilding RGB map at height {height:F2}";
    }

    private void TryStartGroundBuild()
    {
        if (pendingSnapshot == null || groundTask != null) return;

        CloudSnapshot snapshot = pendingSnapshot;
        pendingSnapshot = null;
        int maxSamples = Mathf.Max(1000, maximumGroundSamples);
        float groundPercentile = Mathf.Clamp01(groundHeightPercentile);
        float trimPercentile = Mathf.Clamp(boundsTrimPercentile, 0f, 0.1f);
        int mapResolution = Mathf.Clamp(miniMapResolution, 128, 1024);
        float viewHeightAboveGround = Mathf.Max(0.01f, miniMapViewHeightAboveGround);
        int pointRadiusPixels = Mathf.Clamp(miniMapPointRadiusPixels, 0, 3);

        StatusText = $"building RGB map revision {snapshot.revision} at height {viewHeightAboveGround:F2}";
        groundTask = Task.Run(() => EstimateGround(
            snapshot,
            maxSamples,
            groundPercentile,
            trimPercentile,
            mapResolution,
            viewHeightAboveGround,
            pointRadiusPixels));
    }

    private void FinishGroundBuildIfReady()
    {
        if (groundTask == null || !groundTask.IsCompleted) return;

        if (groundTask.IsFaulted)
        {
            Exception exception = groundTask.Exception != null
                ? groundTask.Exception.GetBaseException()
                : null;
            StatusText = $"ground build failed: {exception?.Message}";
            Debug.LogError($"[PointCloudGround] Build failed: {exception?.Message}");
            groundTask = null;
            return;
        }

        GroundData data = groundTask.Result;
        groundTask = null;
        ApplyGroundBase(data);
    }

    private static GroundData EstimateGround(
        CloudSnapshot snapshot,
        int maxSamples,
        float groundPercentile,
        float trimPercentile,
        int mapResolution,
        float viewHeightAboveGround,
        int pointRadiusPixels)
    {
        long totalPointsLong = 0;
        foreach (Vector3[] chunk in snapshot.chunks)
        {
            if (chunk != null) totalPointsLong += chunk.Length;
        }
        if (totalPointsLong <= 0) throw new InvalidOperationException("Point cloud is empty");

        int sourcePointCount = totalPointsLong > int.MaxValue
            ? int.MaxValue
            : (int)totalPointsLong;
        int stride = Math.Max(1, (int)Math.Ceiling(totalPointsLong / (double)maxSamples));
        int capacity = Math.Min(maxSamples, sourcePointCount);
        float[] xs = new float[capacity];
        float[] ys = new float[capacity];
        float[] zs = new float[capacity];
        long globalIndex = 0;
        int written = 0;

        for (int chunkIndex = 0; chunkIndex < snapshot.chunks.Length; chunkIndex++)
        {
            Vector3[] chunk = snapshot.chunks[chunkIndex];
            if (chunk == null) continue;
            Matrix4x4 localToMap =
                snapshot.chunkToMap != null && chunkIndex < snapshot.chunkToMap.Length
                    ? snapshot.chunkToMap[chunkIndex]
                    : Matrix4x4.identity;
            Matrix4x4 localToWorld = snapshot.mapToWorld * localToMap;
            for (int i = 0; i < chunk.Length; i++, globalIndex++)
            {
                if (globalIndex % stride != 0 || written >= capacity) continue;
                Vector3 point = TransformPoint(localToWorld, chunk[i]);
                if (!IsFinite(point.x) || !IsFinite(point.y) || !IsFinite(point.z)) continue;
                xs[written] = point.x;
                ys[written] = point.y;
                zs[written] = point.z;
                written++;
            }
        }

        if (written < 16) throw new InvalidOperationException("Too few finite point samples for a ground base");
        if (written != capacity)
        {
            Array.Resize(ref xs, written);
            Array.Resize(ref ys, written);
            Array.Resize(ref zs, written);
        }

        Array.Sort(xs);
        Array.Sort(ys);
        Array.Sort(zs);

        int trimIndex = PercentileIndex(written, trimPercentile);
        int upperTrimIndex = written - 1 - trimIndex;
        if (upperTrimIndex <= trimIndex)
        {
            trimIndex = 0;
            upperTrimIndex = written - 1;
        }

        float minX = xs[trimIndex];
        float maxX = xs[upperTrimIndex];
        float minZ = zs[trimIndex];
        float maxZ = zs[upperTrimIndex];
        if (snapshot.hasStableBounds)
        {
            minX = Math.Min(minX, snapshot.stableMinX);
            maxX = Math.Max(maxX, snapshot.stableMaxX);
            minZ = Math.Min(minZ, snapshot.stableMinZ);
            maxZ = Math.Max(maxZ, snapshot.stableMaxZ);
        }
        float groundY = ys[PercentileIndex(written, groundPercentile)];
        float projectionMaxY = groundY + Math.Max(0.01f, viewHeightAboveGround);

        BuildTopDownColorProjection(
            snapshot,
            minX,
            maxX,
            minZ,
            maxZ,
            groundY - 0.08f,
            projectionMaxY,
            mapResolution,
            pointRadiusPixels,
            out byte[] occupancy,
            out uint[] projectedColors,
            out float[] projectedHeights);

        return new GroundData
        {
            revision = snapshot.revision,
            sourcePointCount = sourcePointCount,
            sampleCount = written,
            minX = minX,
            maxX = maxX,
            minY = ys[0],
            maxY = ys[written - 1],
            minZ = minZ,
            maxZ = maxZ,
            groundY = groundY,
            mapCutoffY = projectionMaxY,
            mapResolution = mapResolution,
            mapOccupancy = occupancy,
            mapColors = projectedColors,
            mapHeights = projectedHeights,
        };
    }

    private static void BuildTopDownColorProjection(
        CloudSnapshot snapshot,
        float minX,
        float maxX,
        float minZ,
        float maxZ,
        float minY,
        float maxY,
        int resolution,
        int pointRadius,
        out byte[] occupancy,
        out uint[] colors,
        out float[] heights)
    {
        occupancy = new byte[resolution * resolution];
        colors = new uint[resolution * resolution];
        heights = new float[resolution * resolution];
        for (int i = 0; i < heights.Length; i++) heights[i] = float.NegativeInfinity;

        float width = Math.Max(0.0001f, maxX - minX);
        float depth = Math.Max(0.0001f, maxZ - minZ);

        for (int chunkIndex = 0; chunkIndex < snapshot.chunks.Length; chunkIndex++)
        {
            Vector3[] chunk = snapshot.chunks[chunkIndex];
            if (chunk == null) continue;
            uint[] chunkColors =
                snapshot.colorChunks != null && chunkIndex < snapshot.colorChunks.Length
                    ? snapshot.colorChunks[chunkIndex]
                    : null;
            Matrix4x4 localToMap =
                snapshot.chunkToMap != null && chunkIndex < snapshot.chunkToMap.Length
                    ? snapshot.chunkToMap[chunkIndex]
                    : Matrix4x4.identity;
            Matrix4x4 localToWorld = snapshot.mapToWorld * localToMap;

            for (int i = 0; i < chunk.Length; i++)
            {
                Vector3 point = TransformPoint(localToWorld, chunk[i]);
                if (!IsFinite(point.x) || !IsFinite(point.y) || !IsFinite(point.z) ||
                    point.y < minY || point.y > maxY ||
                    point.x < minX || point.x > maxX ||
                    point.z < minZ || point.z > maxZ) continue;

                uint color = chunkColors != null && i < chunkColors.Length
                    ? chunkColors[i] & 0x00FFFFFFu
                    : 0x00FFFFFFu;
                int centerX = Math.Max(0, Math.Min(
                    resolution - 1,
                    (int)Math.Round((point.x - minX) / width * (resolution - 1))));
                int centerY = Math.Max(0, Math.Min(
                    resolution - 1,
                    (int)Math.Round((point.z - minZ) / depth * (resolution - 1))));

                for (int y = Math.Max(0, centerY - pointRadius);
                     y <= Math.Min(resolution - 1, centerY + pointRadius);
                     y++)
                {
                    for (int x = Math.Max(0, centerX - pointRadius);
                         x <= Math.Min(resolution - 1, centerX + pointRadius);
                         x++)
                    {
                        int pixel = y * resolution + x;
                        if (point.y < heights[pixel]) continue;
                        heights[pixel] = point.y;
                        colors[pixel] = color;
                        occupancy[pixel] = 255;
                    }
                }
            }
        }
    }

    private static int PercentileIndex(int count, float percentile)
    {
        return Math.Max(0, Math.Min(count - 1, (int)Math.Round((count - 1) * percentile)));
    }

    private void ApplyGroundBase(GroundData data)
    {
        float minX = data.minX - groundPadding;
        float maxX = data.maxX + groundPadding;
        float minZ = data.minZ - groundPadding;
        float maxZ = data.maxZ + groundPadding;
        float width = Mathf.Max(0.1f, maxX - minX);
        float depth = Mathf.Max(0.1f, maxZ - minZ);
        float floorY = data.groundY + groundHeightOffset;

        bool isFirstGround = ActiveRevision < 0;
        GameObject newGround = GameObject.CreatePrimitive(PrimitiveType.Cube);
        newGround.name = $"PointCloudGround_Rev_{data.revision}";
        newGround.layer = collisionLayer;
        newGround.transform.SetParent(null, false);
        newGround.transform.position = new Vector3(
            (minX + maxX) * 0.5f,
            floorY - groundThickness * 0.5f,
            (minZ + maxZ) * 0.5f);
        newGround.transform.rotation = Quaternion.identity;
        newGround.transform.localScale = new Vector3(width, groundThickness, depth);

        BoxCollider newCollider = newGround.GetComponent<BoxCollider>();
        newCollider.enabled = enableGroundCollider;
        MeshRenderer renderer = newGround.GetComponent<MeshRenderer>();
        renderer.enabled = showGroundBase;
        if (showGroundBase)
        {
            EnsureGroundMaterial();
            renderer.sharedMaterial = groundMaterial;
        }

        GameObject oldGround = groundObject;
        groundObject = newGround;
        groundCollider = newCollider;
        activeGroundData = data;
        ActiveRevision = data.revision;
        if (oldGround != null) Destroy(oldGround);

        if (isFirstGround && autoPlaceRobotOnFirstGround)
        {
            PlaceRobotOnGround(minX, maxX, minZ, maxZ, floorY);
        }
        UpdateMiniMapTexture(data);
        ReleaseStarterPlayer();

        StatusText =
            $"ground active rev={data.revision}, " +
            $"size={width:F1}x{depth:F1}, y={floorY:F2}, " +
            $"RGB cutoff={data.mapCutoffY:F2}";
        if (verboseLogs)
        {
            Debug.Log(
                $"[PointCloudGround] Activated revision={data.revision} " +
                $"sourcePoints={data.sourcePointCount:N0} samples={data.sampleCount:N0} " +
                $"size={width:F2}x{depth:F2} groundY={floorY:F3} " +
                $"mapCutoffY={data.mapCutoffY:F3} " +
                $"viewHeight={data.mapCutoffY - data.groundY:F3}");
        }
    }

    private void EnsureGroundMaterial()
    {
        if (groundMaterial != null)
        {
            groundMaterial.color = groundBaseColor;
            return;
        }

        Shader shader = Shader.Find("Standard");
        if (shader == null) shader = Shader.Find("Universal Render Pipeline/Lit");
        if (shader == null) shader = Shader.Find("Unlit/Color");
        if (shader == null) return;
        groundMaterial = new Material(shader)
        {
            name = "Runtime Point Cloud Ground Material",
            color = groundBaseColor,
        };
    }

    private void UpdateMiniMapTexture(GroundData data)
    {
        if (!showMiniMap || data.mapOccupancy == null) return;

        int resolution = data.mapResolution;
        if (miniMapTexture == null || miniMapTexture.width != resolution || miniMapTexture.height != resolution)
        {
            if (miniMapTexture != null) Destroy(miniMapTexture);
            miniMapTexture = new Texture2D(resolution, resolution, TextureFormat.RGBA32, false)
            {
                name = "Point Cloud Ground Slice Map",
                wrapMode = TextureWrapMode.Clamp,
                filterMode = FilterMode.Bilinear,
            };
        }

        Color32 background = miniMapBackground;
        Color32 foreground = miniMapPointColor;
        Color32[] pixels = new Color32[data.mapOccupancy.Length];
        float visibleHeight = Mathf.Max(0.1f, data.mapCutoffY - data.groundY);
        float shadeStrength = Mathf.Clamp01(miniMapHeightShadingStrength);
        float edgeThreshold = Mathf.Max(0.02f, miniMapHeightEdgeThreshold);
        float edgeMultiplier = 1f - Mathf.Clamp01(miniMapHeightEdgeDarkening);
        for (int i = 0; i < pixels.Length; i++)
        {
            if (data.mapOccupancy[i] == 0)
            {
                pixels[i] = background;
                continue;
            }

            uint packed = data.mapColors != null && i < data.mapColors.Length
                ? data.mapColors[i]
                : 0u;
            Color32 source = data.mapColors != null && i < data.mapColors.Length
                ? new Color32(
                    (byte)((packed >> 16) & 0xFF),
                    (byte)((packed >> 8) & 0xFF),
                    (byte)(packed & 0xFF),
                    255)
                : foreground;
            float height01 = data.mapHeights != null && i < data.mapHeights.Length
                ? Mathf.Clamp01((data.mapHeights[i] - data.groundY) / visibleHeight)
                : 1f;
            float multiplier = 1f - shadeStrength * (1f - height01);
            if (miniMapShowHeightEdges && IsHeightEdge(
                i,
                resolution,
                data.mapOccupancy,
                data.mapHeights,
                edgeThreshold))
            {
                multiplier *= edgeMultiplier;
            }
            pixels[i] = MultiplyRgb(source, multiplier);
        }
        miniMapTexture.SetPixels32(pixels);
        miniMapTexture.Apply(false, false);
    }

    private static bool IsHeightEdge(
        int index,
        int resolution,
        byte[] occupancy,
        float[] heights,
        float threshold)
    {
        if (heights == null || index >= heights.Length) return false;
        int x = index % resolution;
        int y = index / resolution;
        float height = heights[index];
        if (x > 0 && occupancy[index - 1] != 0 && Math.Abs(height - heights[index - 1]) >= threshold) return true;
        if (x + 1 < resolution && occupancy[index + 1] != 0 && Math.Abs(height - heights[index + 1]) >= threshold) return true;
        if (y > 0 && occupancy[index - resolution] != 0 && Math.Abs(height - heights[index - resolution]) >= threshold) return true;
        if (y + 1 < resolution && occupancy[index + resolution] != 0 && Math.Abs(height - heights[index + resolution]) >= threshold) return true;
        return false;
    }

    private static Color32 MultiplyRgb(Color32 color, float multiplier)
    {
        multiplier = Mathf.Clamp01(multiplier);
        return new Color32(
            (byte)Mathf.Clamp(Mathf.RoundToInt(color.r * multiplier), 0, 255),
            (byte)Mathf.Clamp(Mathf.RoundToInt(color.g * multiplier), 0, 255),
            (byte)Mathf.Clamp(Mathf.RoundToInt(color.b * multiplier), 0, 255),
            255);
    }

    private static Vector3 TransformPoint(Matrix4x4 matrix, Vector3 point)
    {
        return new Vector3(
            matrix.m00 * point.x + matrix.m01 * point.y + matrix.m02 * point.z + matrix.m03,
            matrix.m10 * point.x + matrix.m11 * point.y + matrix.m12 * point.z + matrix.m13,
            matrix.m20 * point.x + matrix.m21 * point.y + matrix.m22 * point.z + matrix.m23);
    }

    private void PlaceRobotOnGround(
        float minX,
        float maxX,
        float minZ,
        float maxZ,
        float floorY)
    {
        FindControlledRobot();
        if (controlledRobot == null) return;

        Vector3 current = controlledRobot.position;
        bool initialPositionIsInside =
            current.x >= minX && current.x <= maxX &&
            current.z >= minZ && current.z <= maxZ;
        float x = keepInitialPositionWhenInsideMap && initialPositionIsInside
            ? current.x
            : (minX + maxX) * 0.5f;
        float z = keepInitialPositionWhenInsideMap && initialPositionIsInside
            ? current.z
            : (minZ + maxZ) * 0.5f;
        Vector3 spawnPosition = new Vector3(x, floorY + robotGroundClearance, z);

        bool controllerWasEnabled = robotCharacterController != null && robotCharacterController.enabled;
        if (controllerWasEnabled) robotCharacterController.enabled = false;
        controlledRobot.position = spawnPosition;
        if (controllerWasEnabled) robotCharacterController.enabled = true;
        if (verboseLogs)
        {
            Debug.Log($"[PointCloudGround] Placed robot at {spawnPosition}");
        }
    }

    private void OnGUI()
    {
        if (!showMiniMap || miniMapTexture == null || activeGroundData == null) return;

        float size = Mathf.Max(100f, miniMapSizePixels);
        float left = alignMiniMapToRight
            ? Mathf.Max(0f, Screen.width - miniMapLeft - size)
            : miniMapLeft;
        Rect outerRect = new Rect(left, miniMapTop, size, size);
        Rect mapRect = new Rect(outerRect.x + 3f, outerRect.y + 20f, outerRect.width - 6f, outerRect.height - 23f);

        Color oldColor = GUI.color;
        GUI.color = Color.white;
        string interaction = enableClickToMove ? " - Click to Move" : "";
        GUI.Box(
            outerRect,
            $"{miniMapTitle} H≤{Mathf.Max(0.01f, miniMapViewHeightAboveGround):F2}{interaction}");
        GUI.DrawTexture(mapRect, miniMapTexture, ScaleMode.StretchToFill, false);

        DrawMiniMapCameraTrajectory(mapRect);
        if (cameraTrajectoryMapPoints.Length > 0)
        {
            Vector3 cameraPosition = TransformPoint(
                mapToWorldMatrix,
                cameraTrajectoryMapPoints[cameraTrajectoryMapPoints.Length - 1]);
            DrawMiniMapMarker(mapRect, cameraPosition, cameraTrajectoryColor, true);
        }
        DrawMiniMapMarker(mapRect, NavigationDestination, Color.yellow, IsNavigating);
        // Draw the player last so the camera marker or trajectory can never hide it.
        DrawMiniMapMarker(mapRect, controlledRobot != null ? controlledRobot.position : Vector3.zero, Color.green, controlledRobot != null);
        GUI.color = oldColor;

        Event current = Event.current;
        if (!enableClickToMove || current == null || current.type != EventType.MouseDown || current.button != 0) return;
        if (!mapRect.Contains(current.mousePosition)) return;

        float u = Mathf.InverseLerp(mapRect.xMin, mapRect.xMax, current.mousePosition.x);
        float v = 1f - Mathf.InverseLerp(mapRect.yMin, mapRect.yMax, current.mousePosition.y);
        Vector3 destination = new Vector3(
            Mathf.Lerp(activeGroundData.minX, activeGroundData.maxX, u),
            activeGroundData.groundY + groundHeightOffset + robotGroundClearance,
            Mathf.Lerp(activeGroundData.minZ, activeGroundData.maxZ, v));
        SetNavigationDestination(destination);
        current.Use();
    }

    private void DrawMiniMapCameraTrajectory(Rect mapRect)
    {
        if (!showCameraTrajectory || activeGroundData == null || cameraTrajectoryMapPoints.Length < 2) return;

        int segmentLimit = Mathf.Max(10, maximumMiniMapTrajectorySegments);
        int stride = Mathf.Max(1, (cameraTrajectoryMapPoints.Length - 1 + segmentLimit - 1) / segmentLimit);
        Vector3 previous = TransformPoint(mapToWorldMatrix, cameraTrajectoryMapPoints[0]);

        for (int i = stride; i < cameraTrajectoryMapPoints.Length; i += stride)
        {
            Vector3 current = TransformPoint(mapToWorldMatrix, cameraTrajectoryMapPoints[i]);
            DrawMiniMapLineSegment(mapRect, previous, current);
            previous = current;
        }

        int lastIndex = cameraTrajectoryMapPoints.Length - 1;
        if ((lastIndex % stride) != 0)
        {
            Vector3 current = TransformPoint(mapToWorldMatrix, cameraTrajectoryMapPoints[lastIndex]);
            DrawMiniMapLineSegment(mapRect, previous, current);
        }
    }

    private void DrawMiniMapLineSegment(Rect mapRect, Vector3 fromWorld, Vector3 toWorld)
    {
        float rangeX = activeGroundData.maxX - activeGroundData.minX;
        float rangeZ = activeGroundData.maxZ - activeGroundData.minZ;
        if (rangeX <= 0.0001f || rangeZ <= 0.0001f) return;

        float fromU = (fromWorld.x - activeGroundData.minX) / rangeX;
        float fromV = (fromWorld.z - activeGroundData.minZ) / rangeZ;
        float toU = (toWorld.x - activeGroundData.minX) / rangeX;
        float toV = (toWorld.z - activeGroundData.minZ) / rangeZ;
        if (fromU < 0f || fromU > 1f || fromV < 0f || fromV > 1f ||
            toU < 0f || toU > 1f || toV < 0f || toV > 1f)
        {
            return;
        }

        Vector2 from = new Vector2(
            mapRect.x + fromU * mapRect.width,
            mapRect.y + (1f - fromV) * mapRect.height);
        Vector2 to = new Vector2(
            mapRect.x + toU * mapRect.width,
            mapRect.y + (1f - toV) * mapRect.height);
        Vector2 delta = to - from;
        float length = delta.magnitude;
        if (length < 0.5f) return;

        Matrix4x4 oldMatrix = GUI.matrix;
        Color oldColor = GUI.color;
        GUI.color = cameraTrajectoryColor;
        GUIUtility.RotateAroundPivot(Mathf.Atan2(delta.y, delta.x) * Mathf.Rad2Deg, from);
        GUI.DrawTexture(
            new Rect(from.x, from.y - cameraTrajectoryWidthPixels * 0.5f, length, cameraTrajectoryWidthPixels),
            Texture2D.whiteTexture);
        GUI.matrix = oldMatrix;
        GUI.color = oldColor;
    }

    private void DrawMiniMapMarker(Rect mapRect, Vector3 worldPosition, Color color, bool visible)
    {
        if (!visible || activeGroundData == null) return;
        float u = Mathf.InverseLerp(activeGroundData.minX, activeGroundData.maxX, worldPosition.x);
        float v = Mathf.InverseLerp(activeGroundData.minZ, activeGroundData.maxZ, worldPosition.z);
        if (u < 0f || u > 1f || v < 0f || v > 1f) return;

        float x = mapRect.x + u * mapRect.width;
        float y = mapRect.y + (1f - v) * mapRect.height;
        Color oldColor = GUI.color;
        GUI.color = color;
        GUI.DrawTexture(new Rect(x - 5f, y - 5f, 10f, 10f), Texture2D.whiteTexture);
        GUI.color = oldColor;
    }

    private void SetNavigationDestination(Vector3 worldPoint)
    {
        FindControlledRobot();
        if (controlledRobot == null)
        {
            Debug.LogWarning("[PointCloudGround] No controlled robot was found for click-to-move.");
            return;
        }

        NavigationDestination = worldPoint;
        IsNavigating = true;
        DisableStarterMovementForNavigation();
        if (verboseLogs) Debug.Log($"[PointCloudGround] Navigate to {worldPoint}");
    }

    private void UpdateClickNavigation()
    {
        if (!IsNavigating || controlledRobot == null) return;

        Vector3 position = controlledRobot.position;
        Vector3 difference = NavigationDestination - position;
        Vector3 horizontal = Vector3.ProjectOnPlane(difference, Vector3.up);
        float distance = horizontal.magnitude;
        if (distance <= stoppingDistance)
        {
            StopNavigation();
            return;
        }

        Vector3 direction = horizontal / Mathf.Max(distance, 0.0001f);
        float step = Mathf.Min(moveSpeed * Time.deltaTime, distance);
        Vector3 motion = direction * step;
        if (robotCharacterController != null && robotCharacterController.enabled)
        {
            motion += Vector3.down * downwardStickSpeed * Time.deltaTime;
            robotCharacterController.Move(motion);
        }
        else
        {
            controlledRobot.position += motion;
        }

        Quaternion targetRotation = Quaternion.LookRotation(direction, Vector3.up);
        controlledRobot.rotation = Quaternion.RotateTowards(
            controlledRobot.rotation,
            targetRotation,
            rotationSpeedDegrees * Time.deltaTime);
        SetRobotAnimation(moveSpeed);
    }

    private void StopNavigation()
    {
        IsNavigating = false;
        SetRobotAnimation(0f);
        if (starterMovement != null && starterMovementWasEnabled)
        {
            starterMovement.enabled = true;
        }
        starterMovement = null;
        starterMovementWasEnabled = false;
    }

    private void SetRobotAnimation(float speed)
    {
        if (robotAnimator == null) return;
        robotAnimator.SetFloat(AnimSpeed, speed);
        robotAnimator.SetFloat(AnimMotionSpeed, speed > 0f ? 1f : 0f);
        robotAnimator.SetBool(AnimGrounded, true);
        robotAnimator.SetBool(AnimFreeFall, false);
    }

    private void FindControlledRobot()
    {
        if (controlledRobot != null)
        {
            if (robotCharacterController == null)
            {
                robotCharacterController = controlledRobot.GetComponent<CharacterController>();
            }
            if (robotAnimator == null) robotAnimator = controlledRobot.GetComponentInChildren<Animator>();
            return;
        }

        CharacterController[] controllers = FindObjectsOfType<CharacterController>();
        foreach (CharacterController controller in controllers)
        {
            Component movement = controller.GetComponent("ThirdPersonController");
            if (movement == null) continue;
            robotCharacterController = controller;
            controlledRobot = controller.transform;
            robotAnimator = controller.GetComponentInChildren<Animator>();
            return;
        }
    }

    private void UpdateMiniMapCursorState()
    {
        if (!showMiniMap || !enableClickToMove || !unlockCursorForMiniMap) return;

        Cursor.lockState = CursorLockMode.None;
        Cursor.visible = true;
        if (controlledRobot == null) return;

        if (starterAssetsInputs == null)
        {
            starterAssetsInputs = controlledRobot.GetComponent("StarterAssetsInputs");
        }
        if (starterAssetsInputs == null) return;

        Type inputType = starterAssetsInputs.GetType();
        System.Reflection.FieldInfo cursorLockedField = inputType.GetField("cursorLocked");
        System.Reflection.FieldInfo cursorLookField = inputType.GetField("cursorInputForLook");
        if (!starterInputSettingsCaptured)
        {
            if (cursorLockedField != null) starterCursorLockedWasEnabled = (bool)cursorLockedField.GetValue(starterAssetsInputs);
            if (cursorLookField != null) starterCursorLookWasEnabled = (bool)cursorLookField.GetValue(starterAssetsInputs);
            starterInputSettingsCaptured = true;
        }
        if (cursorLockedField != null) cursorLockedField.SetValue(starterAssetsInputs, false);
        if (cursorLookField != null) cursorLookField.SetValue(starterAssetsInputs, false);
    }

    private void RestoreStarterInputSettings()
    {
        if (!starterInputSettingsCaptured || starterAssetsInputs == null) return;
        Type inputType = starterAssetsInputs.GetType();
        System.Reflection.FieldInfo cursorLockedField = inputType.GetField("cursorLocked");
        System.Reflection.FieldInfo cursorLookField = inputType.GetField("cursorInputForLook");
        if (cursorLockedField != null) cursorLockedField.SetValue(starterAssetsInputs, starterCursorLockedWasEnabled);
        if (cursorLookField != null) cursorLookField.SetValue(starterAssetsInputs, starterCursorLookWasEnabled);
    }

    private void DisableStarterMovementForNavigation()
    {
        if (controlledRobot == null || starterMovement != null) return;
        Behaviour movement = controlledRobot.GetComponent("ThirdPersonController") as Behaviour;
        if (movement == null) return;
        starterMovement = movement;
        starterMovementWasEnabled = movement.enabled;
        if (starterMovementWasEnabled) movement.enabled = false;
    }

    private void TryHoldStarterPlayer()
    {
        if (!holdStarterPlayerUntilFirstGround || ActiveRevision >= 0 || starterMovement != null) return;
        FindControlledRobot();
        if (controlledRobot == null) return;

        Behaviour movement = controlledRobot.GetComponent("ThirdPersonController") as Behaviour;
        if (movement == null) return;
        starterMovement = movement;
        starterMovementWasEnabled = movement.enabled;
        if (starterMovementWasEnabled)
        {
            movement.enabled = false;
            if (verboseLogs) Debug.Log("[PointCloudGround] Holding player until the ground base is ready.");
        }
    }

    private void ReleaseStarterPlayer()
    {
        if (starterMovement == null) return;
        if (starterMovementWasEnabled) starterMovement.enabled = true;
        starterMovement = null;
        starterMovementWasEnabled = false;
    }

    private static bool IsFinite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private void OnDestroy()
    {
        StopNavigation();
        ReleaseStarterPlayer();
        RestoreStarterInputSettings();
        pendingSnapshot = null;
        latestSourceSnapshot = null;
        if (groundObject != null) Destroy(groundObject);
        if (miniMapTexture != null) Destroy(miniMapTexture);
        if (groundMaterial != null) Destroy(groundMaterial);
    }
}
