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
#if UNITY_EDITOR
        private DrawComponent drawComponent;
#endif
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
#if UNITY_EDITOR
            DrawMeshUtility.DrawPrimitiveMeshes();
            DrawMeshUtility.ClearTRS();
            AppendMeshColliders.GetMeshes.ClearReferenceMeshes();
#endif
        }

        public void Dispose()
        {
#if UNITY_EDITOR
            if (drawComponent != null)
            {
                if (Application.isPlaying)
                    Object.Destroy(drawComponent.gameObject);
                else
                    Object.DestroyImmediate(drawComponent.gameObject);
            }
            drawComponent = null;
#endif
        }
    }

    internal class DrawComponent : MonoBehaviour
    {
        public void OnDrawGizmos()
        {
#if UNITY_EDITOR
            DebugDisplay.Render();
#endif
        }
    }
}
