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

        public void OnAwake() { }

        public void OnUpdate(float deltaTime)
        {
            DrawMeshUtility.DrawPrimitiveMeshes();
            DrawMeshUtility.ClearTRS();
            AppendMeshColliders.GetMeshes.ClearReferenceMeshes();
        }

        public void Dispose() { }
    }
}
#endif
