#if UNITY_EDITOR
using Unity.DebugDisplay;
#if MORPEH_ELYSIUM
using Scellecs.Morpeh.Elysium;
#endif

namespace Scellecs.Morpeh.Physics.Debug
{
#if MORPEH_ELYSIUM
    public sealed class DebugDisplayCleanupSystem : IUpdateSystem
#else
    public sealed class DebugDisplayCleanupSystem : ISystem
#endif
    {
        public World World { get; set; }

        public void OnAwake() { }

        public void OnUpdate(float deltaTime) => DebugDisplay.Clear();

        public void Dispose() { }
    }
}
#endif
