using Scellecs.Morpeh;
using Scellecs.Morpeh.Native;
using Scellecs.Morpeh.Physics.Extensions;
using Scellecs.Morpeh.Transform;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.IL2CPP.CompilerServices;
using Unity.Jobs;
using Unity.Mathematics;

namespace Scellecs.Morpeh.Physics
{
    [Il2CppSetOption(Option.NullChecks, false)]
    [Il2CppSetOption(Option.ArrayBoundsChecks, false)]
    [Il2CppSetOption(Option.DivideByZeroChecks, false)]
    public sealed class PhysicsSimulationSystem : IFixedSystem
    {
        public World World { get; set; }

        private Filter physicsStateFilter;
        private Filter staticBodiesFilter;
        private Filter dynamicBodiesFilter;
        private Filter jointsFilter;

        private Stash<TransformComponent> transformStash;
        private Stash<PhysicsCollider> colliderStash;
        private Stash<PhysicsCustomTags> tagsStash;
        private Stash<PhysicsVelocity> velocityStash;
        private Stash<PhysicsMass> massStash;
        private Stash<PhysicsGravityFactor> gravityFactorStash;
        private Stash<PhysicsDamping> dampingStash;
        private Stash<PhysicsMassOverride> massOverrideStash;
        private Stash<PhysicsJoint> jointsStash;
        private Stash<PhysicsConstrainedBodyPair> bodyPairsStash;

        public void OnAwake()
        {
            physicsStateFilter = World.Filter
                .With<PhysicsStateComponent>()
                .Build();

            staticBodiesFilter = World.Filter
                .With<TransformComponent>()
                .With<PhysicsCollider>()
                .Without<PhysicsVelocity>()
                .Build();

            dynamicBodiesFilter = World.Filter
                .With<TransformComponent>()
                .With<PhysicsCollider>()
                .With<PhysicsVelocity>()
                .Build();

            jointsFilter = World.Filter
                .With<PhysicsJoint>()
                .With<PhysicsConstrainedBodyPair>()
                .Build();

            transformStash = World.GetStash<TransformComponent>();
            colliderStash = World.GetStash<PhysicsCollider>();
            tagsStash = World.GetStash<PhysicsCustomTags>();
            velocityStash = World.GetStash<PhysicsVelocity>();
            massStash = World.GetStash<PhysicsMass>();
            gravityFactorStash = World.GetStash<PhysicsGravityFactor>();
            dampingStash = World.GetStash<PhysicsDamping>();
            massOverrideStash = World.GetStash<PhysicsMassOverride>();
            jointsStash = World.GetStash<PhysicsJoint>();
            bodyPairsStash = World.GetStash<PhysicsConstrainedBodyPair>();
        }

        public unsafe void OnUpdate(float deltaTime)
        {
            ref var physicsState = ref physicsStateFilter.First().GetComponent<PhysicsStateComponent>();
            ref var physicsWorld = ref physicsState.physicsWorld;
            ref var physicsStep = ref physicsState.physicsStep;
            ref var haveStaticBodiesChanged = ref physicsState.haveStaticBodiesChanged;
            ref var simulation = ref physicsState.simulation;

            var dynamicBodiesCount = dynamicBodiesFilter.GetLengthSlow();
            var staticBodiesCount = staticBodiesFilter.GetLengthSlow();
            var jointsCount = jointsFilter.GetLengthSlow();

            var dynamicBodiesFilterNative = dynamicBodiesFilter.AsNative();
            var staticBodiesFilterNative = staticBodiesFilter.AsNative();
            var jointsFilterNative = jointsFilter.AsNative();

            var transformStashNative = transformStash.AsNative();
            var colliderStashNative = colliderStash.AsNative();
            var customTagsStashNative = tagsStash.AsNative();
            var velocityStashNative = velocityStash.AsNative();
            var massStashNative = massStash.AsNative();
            var gravityFactorStashNative = gravityFactorStash.AsNative();
            var dampingStashNative = dampingStash.AsNative();
            var massOverrideStashNative = massOverrideStash.AsNative();
            var jointsStashNative = jointsStash.AsNative();
            var bodyPairsStashNative = bodyPairsStash.AsNative();

            physicsWorld.Reset(staticBodiesCount + 1, dynamicBodiesCount, jointsCount);
            haveStaticBodiesChanged.Value = 1;

            using var handles = new NativeList<JobHandle>(4, Allocator.Temp);

            handles.Add(new CreateDefaultStaticRigidBodyJob()
            {
                bodies = physicsWorld.Bodies,
                bodyIndex = physicsWorld.Bodies.Length - 1,
                entityBodyIndexMap = physicsWorld.CollisionWorld.EntityBodyIndexMap.AsParallelWriter()
            }
            .Schedule());

            if (dynamicBodiesCount > 0)
            {
                handles.Add(new CreateRigidBodiesJob()
                {
                    filter = dynamicBodiesFilterNative,
                    transformStash = transformStashNative,
                    colliderStash = colliderStashNative,
                    customTagsStash = customTagsStashNative,
                    firstBodyIndex = 0,
                    bodies = physicsWorld.Bodies,
                    entityBodyIndexMap = physicsWorld.CollisionWorld.EntityBodyIndexMap.AsParallelWriter()
                }
                .ScheduleParallel(dynamicBodiesCount, 32, default));

                handles.Add(new CreateMotionsJob()
                {
                    filter = dynamicBodiesFilterNative,
                    transformStash = transformStashNative,
                    velocityStash = velocityStashNative,
                    massStash = massStashNative,
                    gravityFactorStash = gravityFactorStashNative,
                    dampingStash = dampingStashNative,
                    massOverrideStash = massOverrideStashNative,
                    motionDatas = physicsWorld.MotionDatas,
                    motionVelocities = physicsWorld.MotionVelocities,
                    defaultPhysicsMass = new PhysicsMass
                    {
                        Transform = RigidTransform.identity,
                        InverseMass = 0.0f,
                        InverseInertia = float3.zero,
                        AngularExpansionFactor = 1.0f,
                    },
                    zeroPhysicsVelocity = new PhysicsVelocity
                    {
                        Linear = float3.zero,
                        Angular = float3.zero
                    },
                    defaultPhysicsDamping = new PhysicsDamping
                    {
                        Linear = 0,
                        Angular = 0,
                    }
                }
                .ScheduleParallel(dynamicBodiesCount, 16, default));
            }

            if (staticBodiesCount > 0)
            {
                handles.Add(new CreateRigidBodiesJob()
                {
                    filter = staticBodiesFilterNative,
                    transformStash = transformStashNative,
                    colliderStash = colliderStashNative,
                    customTagsStash = customTagsStashNative,
                    firstBodyIndex = dynamicBodiesCount,
                    bodies = physicsWorld.Bodies,
                    entityBodyIndexMap = physicsWorld.CollisionWorld.EntityBodyIndexMap.AsParallelWriter()
                }
                .ScheduleParallel(staticBodiesCount, 32, default));
            }

            var handle = JobHandle.CombineDependencies(handles.AsArray());
            handles.Clear();

            if (jointsCount > 0)
            {
                handles.Add(new CreateJointsJob()
                {
                    filter = jointsFilterNative,
                    jointsStash = jointsStashNative,
                    bodyPairsStash = bodyPairsStashNative,
                    numDynamicBodies = dynamicBodiesCount,
                    joints = physicsWorld.Joints,
                    entityBodyIndexMap = physicsWorld.CollisionWorld.EntityBodyIndexMap,
                    entityJointIndexMap = physicsWorld.DynamicsWorld.EntityJointIndexMap.AsParallelWriter(),
                    defaultStaticBodyIndex = physicsWorld.Bodies.Length - 1
                }
                .ScheduleParallel(jointsCount, 32, handle));
            }

            handles.Add(physicsWorld.CollisionWorld.ScheduleBuildBroadphaseJobs(ref physicsWorld, deltaTime, physicsStep.Gravity, haveStaticBodiesChanged.AsReadOnly(), handle));
            handle = JobHandle.CombineDependencies(handles.AsArray());

            var finalResult = simulation.ScheduleStepJobs(new SimulationStepInput()
            {
                World = physicsWorld,
                TimeStep = deltaTime,
                Gravity = physicsStep.Gravity,
                NumSolverIterations = 4,
                SolverStabilizationHeuristicSettings = Solver.StabilizationHeuristicSettings.Default,
                SynchronizeCollisionWorld = physicsStep.SynchronizeCollisionWorld > 0,
            }, handle, physicsStep.MultiThreaded > 0);

            finalResult.FinalExecutionHandle.Complete();

            new ExportDynamicBodiesJob()
            {
                motionDatas = physicsWorld.MotionDatas,
                motionVelocities = physicsWorld.MotionVelocities,
                filter = dynamicBodiesFilterNative,
                transformStash = transformStashNative,
                velocityStash = velocityStashNative
            }
            .ScheduleParallel(dynamicBodiesCount, 16, default).Complete();
        }

        public void Dispose() { }
    }

    [BurstCompile]
    internal struct CreateDefaultStaticRigidBodyJob : IJob
    {
        public int bodyIndex;
        [NativeDisableContainerSafetyRestriction] public NativeArray<RigidBody> bodies;
        [NativeDisableContainerSafetyRestriction] public NativeParallelHashMap<EntityId, int>.ParallelWriter entityBodyIndexMap;

        public void Execute()
        {
            bodies[bodyIndex] = new RigidBody
            {
                WorldFromBody = new RigidTransform(quaternion.identity, float3.zero),
                Scale = 1f,
                Collider = default,
                Entity = EntityId.Invalid,
                CustomTags = 0
            };

            entityBodyIndexMap.TryAdd(EntityId.Invalid, bodyIndex);
        }
    }

    [BurstCompile]
    internal struct CreateRigidBodiesJob : IJobFor
    {
        public NativeFilter filter;
        public NativeStash<TransformComponent> transformStash;
        public NativeStash<PhysicsCollider> colliderStash;
        public NativeStash<PhysicsCustomTags> customTagsStash;

        [ReadOnly] public int firstBodyIndex;
        [NativeDisableContainerSafetyRestriction] public NativeArray<RigidBody> bodies;
        [NativeDisableContainerSafetyRestriction] public NativeParallelHashMap<EntityId, int>.ParallelWriter entityBodyIndexMap;

        public void Execute(int index)
        {
            var bodyIndex = firstBodyIndex + index;
            var entityId = filter[index];
            var transform = transformStash.Get(entityId);
            var collider = colliderStash.Get(entityId);
            var tags = customTagsStash.Get(entityId, out bool hasCustomTags);

            bodies[bodyIndex] = new RigidBody
            {
                WorldFromBody = new RigidTransform(transform.rotation, transform.translation),
                Scale = math.cmax(transform.scale),
                Collider = collider.value,
                Entity = entityId,
                CustomTags = hasCustomTags ? tags.value : (byte)0
            };

            entityBodyIndexMap.TryAdd(entityId, bodyIndex);
        }
    }

    [BurstCompile]
    internal struct CreateMotionsJob : IJobFor
    {
        public NativeFilter filter;
        public NativeStash<TransformComponent> transformStash;
        public NativeStash<PhysicsVelocity> velocityStash;
        public NativeStash<PhysicsMass> massStash;
        public NativeStash<PhysicsGravityFactor> gravityFactorStash;
        public NativeStash<PhysicsDamping> dampingStash;
        public NativeStash<PhysicsMassOverride> massOverrideStash;

        public NativeArray<MotionData> motionDatas;
        public NativeArray<MotionVelocity> motionVelocities;

        public PhysicsMass defaultPhysicsMass;
        public PhysicsVelocity zeroPhysicsVelocity;
        public PhysicsDamping defaultPhysicsDamping;

        public void Execute(int index)
        {
            var entityId = filter[index];

            var transform = transformStash.Get(entityId);
            var scale = math.cmax(transform.scale);
            var velocity = velocityStash.Get(entityId);
            var mass = massStash.Get(entityId, out bool hasPhysicsMassType);
            var gravityFactor = gravityFactorStash.Get(entityId, out bool hasPhysicsGravityFactorType);
            var damping = dampingStash.Get(entityId, out bool hasPhysicsDampingType);
            var massOverride = massOverrideStash.Get(entityId, out bool hasPhysicsMassOverrideType);

            var defaultGravityFactor = hasPhysicsMassType ? 1f : 0f;
            var isKinematic = !hasPhysicsMassType || hasPhysicsMassOverrideType && massOverride.IsKinematic != 0;

            {
                var pmass = isKinematic ? defaultPhysicsMass : mass;
                var setVelocityToZero = isKinematic && hasPhysicsMassOverrideType && massOverride.SetVelocityToZero != 0;
                var pvelocity = setVelocityToZero ? zeroPhysicsVelocity : velocity;
                var hasInfiniteMass = isKinematic || pmass.HasInfiniteMass;
                var pgravityFactor = hasInfiniteMass ? 0f : hasPhysicsGravityFactorType ? gravityFactor.value : defaultGravityFactor;

                pmass = pmass.ApplyScale(scale);

                motionVelocities[index] = new MotionVelocity
                {
                    LinearVelocity = pvelocity.Linear,
                    AngularVelocity = pvelocity.Angular,
                    InverseInertia = pmass.InverseInertia,
                    InverseMass = pmass.InverseMass,
                    AngularExpansionFactor = pmass.AngularExpansionFactor,
                    GravityFactor = pgravityFactor,
                };
            }

            {
                var pmass = hasPhysicsMassType ? mass : defaultPhysicsMass;
                var pdamping = hasPhysicsDampingType ? damping : defaultPhysicsDamping;

                pmass = pmass.ApplyScale(scale);

                motionDatas[index] = new MotionData
                {
                    WorldFromMotion = new RigidTransform(
                        math.mul(transform.rotation, mass.InertiaOrientation),
                        math.rotate(transform.rotation, mass.CenterOfMass) + transform.translation),
                    BodyFromMotion = new RigidTransform(pmass.InertiaOrientation, pmass.CenterOfMass),
                    LinearDamping = pdamping.Linear,
                    AngularDamping = pdamping.Angular,
                };
            }
        }
    }

    [BurstCompile]
    internal struct CreateJointsJob : IJobFor
    {
        public NativeFilter filter;
        public NativeStash<PhysicsJoint> jointsStash;
        public NativeStash<PhysicsConstrainedBodyPair> bodyPairsStash;

        [ReadOnly] public int numDynamicBodies;
        [ReadOnly] public NativeParallelHashMap<EntityId, int> entityBodyIndexMap;

        [NativeDisableParallelForRestriction] public NativeArray<Joint> joints;
        [NativeDisableParallelForRestriction] public NativeParallelHashMap<EntityId, int>.ParallelWriter entityJointIndexMap;

        public int defaultStaticBodyIndex;

        public void Execute(int index)
        {
            var entityId = filter[index];

            var bodyPair = bodyPairsStash.Get(entityId);
            var joint = jointsStash.Get(entityId);

            var entityA = bodyPair.EntityA;
            var entityB = bodyPair.EntityB;

            var pair = new BodyIndexPair
            {
                BodyIndexA = entityA == EntityId.Invalid ? defaultStaticBodyIndex : -1,
                BodyIndexB = entityB == EntityId.Invalid ? defaultStaticBodyIndex : -1,
            };

            pair.BodyIndexA = entityBodyIndexMap.TryGetValue(entityA, out var idxA) ? idxA : -1;
            pair.BodyIndexB = entityBodyIndexMap.TryGetValue(entityB, out var idxB) ? idxB : -1;

            bool isInvalid = false;
            isInvalid |= (pair.BodyIndexA == -1 || pair.BodyIndexB == -1);
            isInvalid |= (pair.BodyIndexA >= numDynamicBodies && pair.BodyIndexB >= numDynamicBodies);

            if (isInvalid)
            {
                pair = BodyIndexPair.Invalid;
            }

            joints[index] = new Joint
            {
                BodyPair = pair,
                Entity = entityId,
                EnableCollision = (byte)bodyPair.EnableCollision,
                AFromJoint = joint.BodyAFromJoint.AsMTransform(),
                BFromJoint = joint.BodyBFromJoint.AsMTransform(),
                Version = joint.Version,
                Constraints = joint.m_Constraints
            };

            entityJointIndexMap.TryAdd(entityId, index);
        }
    }

    [BurstCompile]
    internal struct ExportDynamicBodiesJob : IJobFor
    {
        [ReadOnly] public NativeArray<MotionVelocity> motionVelocities;
        [ReadOnly] public NativeArray<MotionData> motionDatas;

        public NativeFilter filter;
        public NativeStash<TransformComponent> transformStash;
        public NativeStash<PhysicsVelocity> velocityStash;

        public void Execute(int index)
        {
            var entityId = filter[index];

            ref var transform = ref transformStash.Get(entityId);
            ref var velocity = ref velocityStash.Get(entityId);

            MotionData md = motionDatas[index];
            RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));

            transform.translation = worldFromBody.pos;
            transform.rotation = worldFromBody.rot;

            velocity = new PhysicsVelocity
            {
                Linear = motionVelocities[index].LinearVelocity,
                Angular = motionVelocities[index].AngularVelocity
            };
        }
    }
}

