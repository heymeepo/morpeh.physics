#if MORPEH_ELYSIUM
using Scellecs.Morpeh.Elysium;

namespace Scellecs.Morpeh.Physics
{
    public sealed class PhysicsFeature : IEcsFeature
    {
        public void Configure(EcsStartup.FeatureBuilder builder)
        {
            builder
                .AddInitializer(new PhysicsStateInitializer())
                .AddFixedSystem(new PhysicsSimulationSystem());
        }
    }
}
#endif
