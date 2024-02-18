#if UNITY_EDITOR
using Unity.DebugDisplay;
using UnityEngine;
#if MORPEH_ELYSIUM
using Scellecs.Morpeh.Elysium;
#endif

namespace Scellecs.Morpeh.Physics.Debug
{
#if MORPEH_ELYSIUM
    public sealed class DebugDisplayDrawSystem : IUpdateSystem
#else
    public sealed class DebugDisplayDrawSystem : ISystem
#endif
    {
        public World World { get; set; }

        private DrawComponent drawComponent;

        public void OnAwake()
        {
            DebugDisplay.Reinstantiate();

            if (drawComponent == null)
            {
                drawComponent = new GameObject("DebugDisplay.DrawComponent", typeof(DrawComponent)) { hideFlags = HideFlags.DontSave }.GetComponent<DrawComponent>();
            }
        }

        public void OnUpdate(float deltaTime)
        {
            DrawMeshUtility.DrawPrimitiveMeshes();
            DrawMeshUtility.ClearTRS();
            AppendMeshColliders.GetMeshes.ClearReferenceMeshes();
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
