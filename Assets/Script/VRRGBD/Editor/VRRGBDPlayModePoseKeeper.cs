using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace VRRGBD.Editor
{
    [InitializeOnLoad]
    public static class VRRGBDPlayModePoseKeeper
    {
        private const string PrefKey = "VRRGBD.PlayModeSavedPoses";

        static VRRGBDPlayModePoseKeeper()
        {
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        }

        private static void OnPlayModeStateChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.ExitingPlayMode)
                SaveRuntimePoses();
            else if (state == PlayModeStateChange.EnteredEditMode)
                RestoreRuntimePoses();
        }

        private static void SaveRuntimePoses()
        {
            var saved = new SavedPoseSet();
            foreach (RgbdCameraManager manager in UnityEngine.Object.FindObjectsOfType<RgbdCameraManager>())
            {
                if (manager.opticalFrameBindings == null) continue;

                foreach (CameraOpticalFrameBinding binding in manager.opticalFrameBindings)
                {
                    if (binding == null) continue;
                    Transform target = binding.alignmentRoot != null ? binding.alignmentRoot : binding.opticalFrame;
                    if (target == null || target.gameObject.scene.path == null) continue;

                    saved.poses.Add(SavedPose.FromTransform(target));
                }
            }

            EditorPrefs.SetString(PrefKey, JsonUtility.ToJson(saved));
        }

        private static void RestoreRuntimePoses()
        {
            string json = EditorPrefs.GetString(PrefKey, string.Empty);
            if (string.IsNullOrEmpty(json)) return;
            EditorPrefs.DeleteKey(PrefKey);

            var saved = JsonUtility.FromJson<SavedPoseSet>(json);
            if (saved?.poses == null) return;

            foreach (SavedPose pose in saved.poses)
            {
                Transform target = FindTransform(pose.scenePath, pose.hierarchyPath);
                if (target == null) continue;

                Undo.RecordObject(target, "Restore VR RGBD Play Pose");
                target.localPosition = pose.localPosition;
                target.localRotation = pose.localRotation;
                target.localScale = pose.localScale;
                EditorUtility.SetDirty(target);

                Scene scene = target.gameObject.scene;
                if (scene.IsValid())
                    EditorSceneManager.MarkSceneDirty(scene);
            }
        }

        private static Transform FindTransform(string scenePath, string hierarchyPath)
        {
            for (int i = 0; i < SceneManager.sceneCount; i++)
            {
                Scene scene = SceneManager.GetSceneAt(i);
                if (scene.path != scenePath) continue;

                foreach (GameObject root in scene.GetRootGameObjects())
                {
                    if (hierarchyPath == root.name) return root.transform;
                    if (!hierarchyPath.StartsWith(root.name + "/", StringComparison.Ordinal)) continue;

                    string childPath = hierarchyPath.Substring(root.name.Length + 1);
                    Transform child = root.transform.Find(childPath);
                    if (child != null) return child;
                }
            }

            return null;
        }

        private static string GetHierarchyPath(Transform transform)
        {
            var names = new Stack<string>();
            Transform current = transform;
            while (current != null)
            {
                names.Push(current.name);
                current = current.parent;
            }

            return string.Join("/", names);
        }

        [Serializable]
        private sealed class SavedPoseSet
        {
            public List<SavedPose> poses = new List<SavedPose>();
        }

        [Serializable]
        private sealed class SavedPose
        {
            public string scenePath;
            public string hierarchyPath;
            public Vector3 localPosition;
            public Quaternion localRotation;
            public Vector3 localScale;

            public static SavedPose FromTransform(Transform transform)
            {
                return new SavedPose
                {
                    scenePath = transform.gameObject.scene.path,
                    hierarchyPath = GetHierarchyPath(transform),
                    localPosition = transform.localPosition,
                    localRotation = transform.localRotation,
                    localScale = transform.localScale
                };
            }
        }
    }
}
