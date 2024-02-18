#if UNITY_EDITOR
using Unity.DebugDisplay;
using UnityEditor;

namespace Scellecs.Morpeh.Physics.Debug
{
    public sealed class DebugDisplayInitializer : IInitializer
    {
        public World World { get; set; }

        public void OnAwake()
        {
            var asset = AssetDatabase.LoadAssetAtPath<PhysicsDebugDisplayData>(PhysicsDebugDisplayData.ASSET_PATH);

            if (asset != null)
            {
                var ent = World.CreateEntity();
                ent.SetComponent(new PhysicsDebugDisplayDataComponent() { data = asset });
            }
        }

        public void Dispose() { }
    }
}
#endif
