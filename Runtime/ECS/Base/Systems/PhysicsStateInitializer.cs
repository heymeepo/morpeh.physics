using Unity.Collections;
using Unity.IL2CPP.CompilerServices;

namespace Scellecs.Morpeh.Physics
{
    [Il2CppSetOption(Option.NullChecks, false)]
    [Il2CppSetOption(Option.ArrayBoundsChecks, false)]
    [Il2CppSetOption(Option.DivideByZeroChecks, false)]
    public sealed class PhysicsStateInitializer : IInitializer
    {
        public World World { get; set; }

        public void OnAwake()
        {
            var state = World.CreateEntity();
            var physicsWorld = new PhysicsWorld(0, 0, 0);
            var physicsStep = PhysicsStep.Default;

            state.SetComponent(new PhysicsStateComponent()
            {
                physicsWorld = physicsWorld,
                physicsStep = physicsStep,
                haveStaticBodiesChanged = new NativeReference<int>(Allocator.Persistent),
                simulation = Simulation.Create()
            });

            World.GetStash<PhysicsStateComponent>().AsDisposable();
        }

        public void Dispose() { }
    }
}
