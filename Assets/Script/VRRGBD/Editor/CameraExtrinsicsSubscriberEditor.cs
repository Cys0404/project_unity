using UnityEditor;
using UnityEngine;

namespace VRRGBD.Editor
{
    [CustomEditor(typeof(CameraExtrinsicsSubscriber))]
    public sealed class CameraExtrinsicsSubscriberEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            EditorGUILayout.Space();
            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                if (GUILayout.Button("Apply Pending Manual Extrinsics"))
                {
                    foreach (UnityEngine.Object targetObject in targets)
                    {
                        var subscriber = (CameraExtrinsicsSubscriber)targetObject;
                        subscriber.ApplyPendingManualExtrinsics();
                    }
                }
            }

            if (!Application.isPlaying)
                EditorGUILayout.HelpBox("Manual extrinsics can be applied while Play mode is running.", MessageType.Info);
        }
    }
}
