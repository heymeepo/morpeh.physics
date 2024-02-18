#if UNITY_EDITOR
using Unity.DebugDisplay;
using UnityEditor;
using UnityEngine;

namespace Scellecs.Morpeh.Physics.Debug
{
    public sealed class DebugDisplayInitializer : IInitializer
    {
        public World World { get; set; }

        private DrawComponent drawComponent;

        public void OnAwake()
        {
            DebugDisplay.Reinstantiate();

            if (drawComponent == null)
            {
                drawComponent = new GameObject("DebugDisplayHelper", typeof(DrawComponent)) { hideFlags = HideFlags.DontSave }.GetComponent<DrawComponent>();
            }

            var asset = AssetDatabase.LoadAssetAtPath<PhysicsDebugDisplayData>(PhysicsDebugDisplayData.ASSET_PATH);

            if (asset != null)
            {
                var ent = World.CreateEntity();
                ent.SetComponent(new PhysicsDebugDisplayDataComponent() { data = asset });
            }
        }

        public void Dispose() 
        {
            if (drawComponent != null)
            {
                if (Application.isPlaying)
                    Object.Destroy(drawComponent.gameObject);
                else
                    Object.DestroyImmediate(drawComponent.gameObject);
            }
            drawComponent = null;
        }
    }

    internal class DrawComponent : MonoBehaviour
    {
        public void OnDrawGizmos() => DebugDisplay.Render();
    }
}
#endif
