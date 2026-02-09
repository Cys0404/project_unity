// =====================
//  ICP 對齊器 (純 C#，無外部依賴)
//  用於 Keyframe 點雲對齊
// =====================

using System;
using System.Collections.Generic;
using UnityEngine;

public class ICPAlignerUnity
{
    // 參數
    public int MaxIterations = 50;
    public float Tolerance = 1e-6f;
    public float MaxCorrespondenceDistance = 0.5f;
    public float DownsampleVoxelSize = 0.05f;

    // 結果
    public struct AlignmentResult
    {
        public Matrix4x4 Transform;
        public float FinalError;
        public int Iterations;
        public int Inliers;
        public bool Success;
    }

    /// <summary>
    /// 將 source 點雲對齊到 target 點雲
    /// </summary>
    public AlignmentResult Align(Vector3[] source, Vector3[] target)
    {
        var result = new AlignmentResult
        {
            Transform = Matrix4x4.identity,
            Success = false
        };

        if (source == null || target == null || source.Length < 10 || target.Length < 10)
            return result;

        // 降採樣
        Vector3[] srcDown = Downsample(source, DownsampleVoxelSize);
        Vector3[] tgtDown = Downsample(target, DownsampleVoxelSize);

        if (srcDown.Length < 10 || tgtDown.Length < 10)
            return result;

        // 建立 target 的 KD-Tree 近似（用 voxel grid）
        var targetGrid = BuildVoxelGrid(tgtDown, MaxCorrespondenceDistance);

        // ICP 迭代
        Matrix4x4 cumulative = Matrix4x4.identity;
        Vector3[] currentSource = (Vector3[])srcDown.Clone();
        float prevError = float.MaxValue;

        for (int iter = 0; iter < MaxIterations; iter++)
        {
            // 找對應點
            var (srcCorr, tgtCorr, error, inliers) = FindCorrespondences(
                currentSource, tgtDown, targetGrid, MaxCorrespondenceDistance);

            if (srcCorr.Count < 10)
                break;

            result.FinalError = error;
            result.Inliers = inliers;
            result.Iterations = iter + 1;

            // 檢查收斂
            if (Mathf.Abs(prevError - error) < Tolerance)
            {
                result.Success = true;
                break;
            }
            prevError = error;

            // 計算最佳變換
            Matrix4x4 stepTransform = ComputeRigidTransform(srcCorr, tgtCorr);

            // 累積變換
            cumulative = stepTransform * cumulative;

            // 更新 source
            for (int i = 0; i < currentSource.Length; i++)
                currentSource[i] = stepTransform.MultiplyPoint3x4(currentSource[i]);
        }

        result.Transform = cumulative;
        if (result.Inliers > source.Length * 0.1f)
            result.Success = true;

        return result;
    }

    /// <summary>
    /// 降採樣
    /// </summary>
    private Vector3[] Downsample(Vector3[] points, float voxelSize)
    {
        if (voxelSize <= 0) return points;

        var voxelDict = new Dictionary<Vector3Int, List<Vector3>>();
        float inv = 1f / voxelSize;

        foreach (var p in points)
        {
            if (float.IsNaN(p.x) || float.IsInfinity(p.x)) continue;

            var key = new Vector3Int(
                Mathf.FloorToInt(p.x * inv),
                Mathf.FloorToInt(p.y * inv),
                Mathf.FloorToInt(p.z * inv)
            );

            if (!voxelDict.ContainsKey(key))
                voxelDict[key] = new List<Vector3>();
            voxelDict[key].Add(p);
        }

        var result = new Vector3[voxelDict.Count];
        int idx = 0;
        foreach (var kv in voxelDict)
        {
            Vector3 sum = Vector3.zero;
            foreach (var p in kv.Value)
                sum += p;
            result[idx++] = sum / kv.Value.Count;
        }

        return result;
    }

    /// <summary>
    /// 建立 Voxel Grid 用於快速查找
    /// </summary>
    private Dictionary<Vector3Int, List<int>> BuildVoxelGrid(Vector3[] points, float cellSize)
    {
        var grid = new Dictionary<Vector3Int, List<int>>();
        float inv = 1f / cellSize;

        for (int i = 0; i < points.Length; i++)
        {
            var key = new Vector3Int(
                Mathf.FloorToInt(points[i].x * inv),
                Mathf.FloorToInt(points[i].y * inv),
                Mathf.FloorToInt(points[i].z * inv)
            );

            if (!grid.ContainsKey(key))
                grid[key] = new List<int>();
            grid[key].Add(i);
        }

        return grid;
    }

    /// <summary>
    /// 找對應點
    /// </summary>
    private (List<Vector3> srcCorr, List<Vector3> tgtCorr, float error, int inliers) FindCorrespondences(
        Vector3[] source, Vector3[] target, Dictionary<Vector3Int, List<int>> targetGrid, float maxDist)
    {
        var srcCorr = new List<Vector3>();
        var tgtCorr = new List<Vector3>();
        float totalError = 0;
        int inliers = 0;

        float inv = 1f / maxDist;
        float maxDistSq = maxDist * maxDist;

        foreach (var srcPoint in source)
        {
            var key = new Vector3Int(
                Mathf.FloorToInt(srcPoint.x * inv),
                Mathf.FloorToInt(srcPoint.y * inv),
                Mathf.FloorToInt(srcPoint.z * inv)
            );

            // 搜索周圍 27 個 cell
            float bestDistSq = float.MaxValue;
            int bestIdx = -1;

            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        var neighborKey = new Vector3Int(key.x + dx, key.y + dy, key.z + dz);
                        if (!targetGrid.TryGetValue(neighborKey, out var indices))
                            continue;

                        foreach (int idx in indices)
                        {
                            float distSq = (srcPoint - target[idx]).sqrMagnitude;
                            if (distSq < bestDistSq)
                            {
                                bestDistSq = distSq;
                                bestIdx = idx;
                            }
                        }
                    }

            if (bestIdx >= 0 && bestDistSq < maxDistSq)
            {
                srcCorr.Add(srcPoint);
                tgtCorr.Add(target[bestIdx]);
                totalError += Mathf.Sqrt(bestDistSq);
                inliers++;
            }
        }

        float meanError = inliers > 0 ? totalError / inliers : float.MaxValue;
        return (srcCorr, tgtCorr, meanError, inliers);
    }

    /// <summary>
    /// 計算剛體變換 (SVD)
    /// </summary>
    private Matrix4x4 ComputeRigidTransform(List<Vector3> source, List<Vector3> target)
    {
        if (source.Count != target.Count || source.Count < 3)
            return Matrix4x4.identity;

        int n = source.Count;

        // 計算質心
        Vector3 srcCentroid = Vector3.zero;
        Vector3 tgtCentroid = Vector3.zero;
        for (int i = 0; i < n; i++)
        {
            srcCentroid += source[i];
            tgtCentroid += target[i];
        }
        srcCentroid /= n;
        tgtCentroid /= n;

        // 中心化
        var srcCentered = new Vector3[n];
        var tgtCentered = new Vector3[n];
        for (int i = 0; i < n; i++)
        {
            srcCentered[i] = source[i] - srcCentroid;
            tgtCentered[i] = target[i] - tgtCentroid;
        }

        // 計算 H = sum(src * tgt^T)
        float[,] H = new float[3, 3];
        for (int i = 0; i < n; i++)
        {
            H[0, 0] += srcCentered[i].x * tgtCentered[i].x;
            H[0, 1] += srcCentered[i].x * tgtCentered[i].y;
            H[0, 2] += srcCentered[i].x * tgtCentered[i].z;
            H[1, 0] += srcCentered[i].y * tgtCentered[i].x;
            H[1, 1] += srcCentered[i].y * tgtCentered[i].y;
            H[1, 2] += srcCentered[i].y * tgtCentered[i].z;
            H[2, 0] += srcCentered[i].z * tgtCentered[i].x;
            H[2, 1] += srcCentered[i].z * tgtCentered[i].y;
            H[2, 2] += srcCentered[i].z * tgtCentered[i].z;
        }

        // SVD 分解 (使用 Jacobi 迭代)
        float[,] U, V;
        float[] S;
        SVD3x3(H, out U, out S, out V);

        // R = V * U^T
        float[,] R = new float[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                R[i, j] = 0;
                for (int k = 0; k < 3; k++)
                    R[i, j] += V[i, k] * U[j, k];
            }

        // 確保是旋轉矩陣 (det = 1)
        float det = R[0, 0] * (R[1, 1] * R[2, 2] - R[1, 2] * R[2, 1])
                  - R[0, 1] * (R[1, 0] * R[2, 2] - R[1, 2] * R[2, 0])
                  + R[0, 2] * (R[1, 0] * R[2, 1] - R[1, 1] * R[2, 0]);

        if (det < 0)
        {
            V[0, 2] = -V[0, 2];
            V[1, 2] = -V[1, 2];
            V[2, 2] = -V[2, 2];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    R[i, j] = 0;
                    for (int k = 0; k < 3; k++)
                        R[i, j] += V[i, k] * U[j, k];
                }
        }

        // t = tgtCentroid - R * srcCentroid
        Vector3 t = tgtCentroid - new Vector3(
            R[0, 0] * srcCentroid.x + R[0, 1] * srcCentroid.y + R[0, 2] * srcCentroid.z,
            R[1, 0] * srcCentroid.x + R[1, 1] * srcCentroid.y + R[1, 2] * srcCentroid.z,
            R[2, 0] * srcCentroid.x + R[2, 1] * srcCentroid.y + R[2, 2] * srcCentroid.z
        );

        // 構建 4x4 矩陣
        Matrix4x4 result = Matrix4x4.identity;
        result.m00 = R[0, 0]; result.m01 = R[0, 1]; result.m02 = R[0, 2]; result.m03 = t.x;
        result.m10 = R[1, 0]; result.m11 = R[1, 1]; result.m12 = R[1, 2]; result.m13 = t.y;
        result.m20 = R[2, 0]; result.m21 = R[2, 1]; result.m22 = R[2, 2]; result.m23 = t.z;

        return result;
    }

    /// <summary>
    /// 3x3 SVD (Jacobi 迭代)
    /// </summary>
    private void SVD3x3(float[,] A, out float[,] U, out float[] S, out float[,] V)
    {
        U = new float[3, 3];
        V = new float[3, 3];
        S = new float[3];

        // 複製 A
        float[,] B = new float[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                B[i, j] = A[i, j];

        // 初始化 U, V 為單位矩陣
        for (int i = 0; i < 3; i++)
        {
            U[i, i] = 1;
            V[i, i] = 1;
        }

        // Jacobi 迭代
        for (int iter = 0; iter < 30; iter++)
        {
            // 對每對 (p, q) 做 Givens 旋轉
            for (int p = 0; p < 2; p++)
                for (int q = p + 1; q < 3; q++)
                {
                    // 計算 2x2 子矩陣的 SVD
                    float app = 0, aqq = 0, apq = 0;
                    for (int k = 0; k < 3; k++)
                    {
                        app += B[k, p] * B[k, p];
                        aqq += B[k, q] * B[k, q];
                        apq += B[k, p] * B[k, q];
                    }

                    if (Mathf.Abs(apq) < 1e-10f) continue;

                    float tau = (aqq - app) / (2 * apq);
                    float t = Mathf.Sign(tau) / (Mathf.Abs(tau) + Mathf.Sqrt(1 + tau * tau));
                    float c = 1 / Mathf.Sqrt(1 + t * t);
                    float s = t * c;

                    // 更新 B
                    for (int k = 0; k < 3; k++)
                    {
                        float bp = B[k, p];
                        float bq = B[k, q];
                        B[k, p] = c * bp - s * bq;
                        B[k, q] = s * bp + c * bq;
                    }

                    // 更新 V
                    for (int k = 0; k < 3; k++)
                    {
                        float vp = V[k, p];
                        float vq = V[k, q];
                        V[k, p] = c * vp - s * vq;
                        V[k, q] = s * vp + c * vq;
                    }
                }
        }

        // 計算 U = A * V * S^-1
        for (int j = 0; j < 3; j++)
        {
            float sigma = 0;
            for (int k = 0; k < 3; k++)
                sigma += B[k, j] * B[k, j];
            S[j] = Mathf.Sqrt(sigma);

            if (S[j] > 1e-10f)
            {
                for (int i = 0; i < 3; i++)
                    U[i, j] = B[i, j] / S[j];
            }
            else
            {
                U[j, j] = 1;
            }
        }
    }

    /// <summary>
    /// 變換點雲
    /// </summary>
    public static Vector3[] TransformPoints(Vector3[] points, Matrix4x4 transform)
    {
        var result = new Vector3[points.Length];
        for (int i = 0; i < points.Length; i++)
            result[i] = transform.MultiplyPoint3x4(points[i]);
        return result;
    }
}
