using Scellecs.Morpeh.Systems;
using Unity.IL2CPP.CompilerServices;
using UnityEngine;

namespace Scellecs.Morpeh.Physics
{
    [Il2CppSetOption(Option.NullChecks, false)]
    [Il2CppSetOption(Option.ArrayBoundsChecks, false)]
    [Il2CppSetOption(Option.DivideByZeroChecks, false)]
    [CreateAssetMenu(menuName = "ECS/Initializers/PhysicsStateInitializer")]
    public sealed class PhysicsStateInitializerSO : Initializer
    {
        private PhysicsStateInitializer initializer;

        public override void OnAwake()
        {
            initializer = new PhysicsStateInitializer() { World = World };
            initializer.OnAwake();
        }

        public override void Dispose() => initializer.Dispose();
    }
}
