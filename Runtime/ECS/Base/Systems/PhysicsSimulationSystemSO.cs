using Scellecs.Morpeh.Systems;
using Unity.IL2CPP.CompilerServices;
using UnityEngine;

namespace Scellecs.Morpeh.Physics
{
    [Il2CppSetOption(Option.NullChecks, false)]
    [Il2CppSetOption(Option.ArrayBoundsChecks, false)]
    [Il2CppSetOption(Option.DivideByZeroChecks, false)]
    [CreateAssetMenu(menuName = "ECS/Systems/PhysicsSimulationSystem")]
    public sealed class PhysicsSimulationSystemSO : FixedUpdateSystem
    {
        private PhysicsSimulationSystem system;

        public override void OnAwake()
        {
            system = new PhysicsSimulationSystem() { World = World };
            system.OnAwake();
        }

        public override void OnUpdate(float deltaTime) => system.OnUpdate(deltaTime);

        public override void Dispose() => system.Dispose();
    }
}

