#if UNITY_EDITOR
using UnityEditor;
using UnityEngine.UIElements;
using UnityEngine;
using Unity.DebugDisplay;

namespace Scellecs.Morpeh.Physics.Debug.Editor
{
    public class PhysicsDisplayEditorWindow : EditorWindow
    {
        [MenuItem("Tools/Morpeh/Physics Debug Display")]
        public static void ShowWindow()
        {
            PhysicsDisplayEditorWindow window = GetWindow<PhysicsDisplayEditorWindow>();
            window.titleContent = new GUIContent("Physics Debug Display");
        }

        public void CreateGUI()
        {
            var assetPath = PhysicsDebugDisplayData.ASSET_PATH;
            PhysicsDebugDisplayData asset = AssetDatabase.LoadAssetAtPath<PhysicsDebugDisplayData>(assetPath);

            if (asset == null)
            {
                var createButton = new Button(() => CreateAsset(assetPath))
                {
                    text = "Create PhysicsDisplay Asset"
                };
                rootVisualElement.Add(createButton);
            }
            else
            {
                var drawCollidersToggle = new Toggle("Draw Colliders")
                {
                    value = asset.DrawColliders
                };
                drawCollidersToggle.RegisterValueChangedCallback(evt =>
                {
                    asset.DrawColliders = evt.newValue;
                    EditorUtility.SetDirty(asset);
                });
                rootVisualElement.Add(drawCollidersToggle);

                var drawColliderEdgesToggle = new Toggle("Draw Collider Edges")
                {
                    value = asset.DrawColliderEdges
                };
                drawColliderEdgesToggle.RegisterValueChangedCallback(evt =>
                {
                    asset.DrawColliderEdges = evt.newValue;
                    EditorUtility.SetDirty(asset);
                });
                rootVisualElement.Add(drawColliderEdgesToggle);
            }
        }

        private void CreateAsset(string assetPath)
        {
            PhysicsDebugDisplayData asset = CreateInstance<PhysicsDebugDisplayData>();

            string folderPath = "Assets/Plugins/Scellecs/Morpeh Physics/Assets";
            string[] folders = folderPath.Split('/');

            string currentPath = "";

            foreach (string folder in folders)
            {
                if (string.IsNullOrEmpty(currentPath) == false)
                {
                    currentPath += "/";
                }

                currentPath += folder;

                if (AssetDatabase.IsValidFolder(currentPath) == false)
                {
                    string parentDirectory = System.IO.Path.GetDirectoryName(currentPath);
                    string newFolderName = System.IO.Path.GetFileName(currentPath);

                    AssetDatabase.CreateFolder(parentDirectory, newFolderName);
                }
            }

            AssetDatabase.CreateAsset(asset, assetPath);
            AssetDatabase.SaveAssets();

            rootVisualElement.Clear();
            CreateGUI();
        }
    }
}
#endif
