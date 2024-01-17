#if UNITY_DATAFLOWGRAPH_EXISTS
using Unity.Burst;
using Unity.DataFlowGraph;

namespace Unity.Physics
{
    /// <summary>
    /// DataFlowGraph node that performs a Collider Cast query on a CollisionWorld.
    /// </summary>
    public class ColliderCastNode : KernelNodeDefinition<ColliderCastNode.KernelDefs>
    {
        public struct KernelDefs : IKernelPortDefinition
        {
            public DataInput<ColliderCastNode, ColliderCastInput> Input;
            public DataInput<ColliderCastNode, PhysicsWorldSingleton> PhysicsWorld;

            public DataOutput<ColliderCastNode, ColliderCastHit> Hit;
            public DataOutput<ColliderCastNode, bool> HitSuccess;
        }

        public struct KernelData : IKernelData
        {
        }

        [BurstCompile]
        public struct Kernel : IGraphKernel<KernelData, KernelDefs>
        {
            public void Execute(RenderContext ctx, in KernelData data, ref KernelDefs ports)
            {
                var physicsWorld = ctx.Resolve(ports.PhysicsWorld);
                var collisionWorld = physicsWorld.CollisionWorld;

                if (collisionWorld.NumBodies > 0)
                {
                    ctx.Resolve(ref ports.HitSuccess) = collisionWorld.CastCollider(ctx.Resolve(ports.Input), out ColliderCastHit hit);
                    ctx.Resolve(ref ports.Hit) = hit;
                }
                else
                {
                    ctx.Resolve(ref ports.HitSuccess) = false;
                }
            }
        }
    }
}
#endif
