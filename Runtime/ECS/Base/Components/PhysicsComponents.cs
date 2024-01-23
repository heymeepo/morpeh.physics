using System;
using Unity.Collections.Blobs;
using Unity.Collections;
using Unity.Mathematics;
using Scellecs.Morpeh.Physics.Extensions;

namespace Scellecs.Morpeh.Physics
{
    public struct PhysicsStateComponent : IComponent, IDisposable
    {
        public PhysicsWorld physicsWorld;
        public PhysicsStep physicsStep;
        public Simulation simulation;
        public NativeReference<int> haveStaticBodiesChanged;

        public void Dispose()
        {
            physicsWorld.Dispose();
            simulation.Dispose();
            haveStaticBodiesChanged.Dispose();
        }
    }


    /// <summary>
    /// The collision geometry of a rigid body. If not present, the rigid body cannot collide with
    /// anything.
    /// </summary>
    public struct PhysicsCollider : IComponent, IDisposable
    {
        /// <summary>  The collider reference, null is allowed. </summary>
        public BlobAssetReference<Collider> value;

        /// <summary>   Gets a value indicating whether this object is valid. </summary>
        ///
        /// <value> True if this object is valid, false if not. </value>
        public bool IsValid => value.IsCreated;

        /// <summary>   Gets the collider pointer. </summary>
        ///
        /// <value> The collider pointer. </value>
        public unsafe Collider* ColliderPtr => (Collider*)value.GetUnsafePtr();

        /// <summary>   Gets the mass properties. </summary>
        ///
        /// <value> The mass properties. </value>
        public MassProperties MassProperties => value.IsCreated ? value.Value.MassProperties : MassProperties.UnitSphere;

        public void Dispose()
        {
            if (IsValid)
            {
                value.Dispose();
            }
        }
    }

    /// <summary>
    /// The mass properties of a rigid body. If not present, the rigid body has infinite mass and
    /// inertia.
    /// </summary>
    public struct PhysicsMass : IComponent
    {
        /// <summary>   Center of mass and orientation of principal axes. </summary>
        public RigidTransform Transform;
        /// <summary>   Zero is allowed, for infinite mass. </summary>
        public float InverseMass;
        /// <summary>   Zero is allowed, for infinite inertia. </summary>
        public float3 InverseInertia;
        /// <summary>   See MassProperties.AngularExpansionFactor. </summary>
        public float AngularExpansionFactor;

        /// <summary>   Gets or sets the center of mass. </summary>
        ///
        /// <value> The center of mass. </value>
        public float3 CenterOfMass { get => Transform.pos; set => Transform.pos = value; }

        /// <summary>   Gets or sets the inertia orientation. </summary>
        ///
        /// <value> The inertia orientation. </value>
        public quaternion InertiaOrientation { get => Transform.rot; set => Transform.rot = value; }

        /// <summary>   Gets a value indicating whether this object has infinite mass. </summary>
        ///
        /// <value> True if this object has infinite mass, false if not. </value>
        public bool HasInfiniteMass => InverseMass == 0.0f;

        /// <summary>   Gets a value indicating whether this object has infinite inertia. </summary>
        ///
        /// <value> True if this object has infinite inertia, false if not. </value>
        public bool HasInfiniteInertia => !math.any(InverseInertia);

        /// <summary>   Gets a value indicating whether this object is kinematic. </summary>
        ///
        /// <value> True if this object is kinematic, false if not. </value>
        public bool IsKinematic => HasInfiniteMass && HasInfiniteInertia;

        /// <summary>   Creates a dynamic mass. </summary>
        ///
        /// <param name="massProperties">   The mass properties. </param>
        /// <param name="mass">             The mass. </param>
        ///
        /// <returns>   The new dynamic mass. </returns>
        public static PhysicsMass CreateDynamic(MassProperties massProperties, float mass)
        {
            SafetyChecks.CheckFiniteAndPositiveAndThrow(mass, nameof(mass));

            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = math.rcp(mass),
                InverseInertia = math.rcp(massProperties.MassDistribution.InertiaTensor * mass),
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }

        /// <summary>   Creates a kinematic mass. </summary>
        ///
        /// <param name="massProperties">   The mass properties. </param>
        ///
        /// <returns>   The new kinematic mass. </returns>
        public static PhysicsMass CreateKinematic(MassProperties massProperties)
        {
            return new PhysicsMass
            {
                Transform = massProperties.MassDistribution.Transform,
                InverseMass = 0f,
                InverseInertia = float3.zero,
                AngularExpansionFactor = massProperties.AngularExpansionFactor
            };
        }
    }

    /// <summary>
    /// Add this component to a dynamic body if it needs to sometimes switch to being kinematic. This
    /// allows you to retain its dynamic mass properties on its <see cref="PhysicsMass"/> component,
    /// but have the physics solver temporarily treat it as if it were kinematic. Kinematic bodies
    /// will have infinite mass and inertia. They should also not be affected by gravity. Hence, if
    /// IsKinematic is non-zero the value in an associated <see cref="PhysicsGravityFactor"/>
    /// component is also ignored. If SetVelocityToZero is non-zero then the value in an associated <see cref="PhysicsVelocity"/>
    /// component is also ignored.
    /// </summary>
    public struct PhysicsMassOverride : IComponent
    {
        /// <summary>   The is kinematic flag. </summary>
        public byte IsKinematic;
        /// <summary>   The set velocity to zero flag. </summary>
        public byte SetVelocityToZero;
    }

    /// <summary>   The velocity of a rigid body. If absent, the rigid body is static. </summary>
    public struct PhysicsVelocity : IComponent
    {
        /// <summary>   The body's world-space linear velocity in units per second. </summary>
        public float3 Linear;

        /// <summary>
        /// The body's angular velocity in radians per second about each principal axis specified by <see cref="PhysicsMass.Transform"/>
        /// . In order to get or set world-space values, use <see cref="PhysicsComponentExtensions.GetAngularVelocityWorldSpace"/>
        /// and <see cref="PhysicsComponentExtensions.SetAngularVelocityWorldSpace"/>, respectively.
        /// </summary>
        public float3 Angular;

        /// <summary>   Zero Physics Velocity. All fields are initialized to zero. </summary>
        public static readonly PhysicsVelocity Zero = new PhysicsVelocity
        {
            Linear = new float3(0),
            Angular = new float3(0)
        };

        /// <summary>
        /// Create a <see cref="PhysicsVelocity"/> required to move a body to a target position and
        /// orientation. Use this method to control kinematic bodies directly if they need to generate
        /// contact events when moving to their new positions. If you need to teleport kinematic bodies
        /// you can simply set their <see cref="Unity.Transforms.LocalTransform"/> component values directly.
        /// </summary>
        ///
        /// <param name="bodyMass">         The body's <see cref="PhysicsMass"/> component. </param>
        /// <param name="bodyPosition">     The body's world-space position. </param>
        /// <param name="bodyOrientation">  The body's world-space rotation. </param>
        /// <param name="targetTransform">  The desired translation and rotation values the body should
        /// move to in world space. </param>
        /// <param name="stepFrequency">   The step frequency in the simulation where the body's motion
        /// is solved (i.e., 1 / FixedDeltaTime). </param>
        ///
        /// <returns>   The calculated velocity to target. </returns>
        public static PhysicsVelocity CalculateVelocityToTarget(
            in PhysicsMass bodyMass, in float3 bodyPosition, in quaternion bodyOrientation,
            in RigidTransform targetTransform, in float stepFrequency
        )
        {
            var velocity = new PhysicsVelocity();
            var worldFromBody = new RigidTransform(bodyOrientation, bodyPosition);
            var worldFromMotion = math.mul(worldFromBody, bodyMass.Transform);
            PhysicsWorldExtensions.CalculateVelocityToTargetImpl(
                worldFromBody, math.inverse(worldFromMotion.rot), bodyMass.Transform.pos, targetTransform, stepFrequency,
                out velocity.Linear, out velocity.Angular
            );
            return velocity;
        }
    }

    /// <summary>
    /// Optional damping applied to the rigid body velocities during each simulation step. This
    /// scales the velocities using: math.clamp(1 - damping * Timestep, 0, 1)
    /// </summary>
    public struct PhysicsDamping : IComponent
    {
        /// <summary>   Damping applied to the linear velocity. </summary>
        public float Linear;
        /// <summary>   Damping applied to the angular velocity. </summary>
        public float Angular;
    }

    /// <summary>
    /// Optional gravity factor applied to a rigid body during each simulation step. This scales the
    /// gravity vector supplied to the simulation step.
    /// </summary>
    public struct PhysicsGravityFactor : IComponent
    {
        /// <summary>   The value. </summary>
        public float value;
    }

    /// <summary>
    /// Optional custom tags attached to a rigid body. This will be copied to any contacts and
    /// Jacobians involving this rigid body, providing additional context to any user logic operating
    /// on those structures.
    /// </summary>
    public struct PhysicsCustomTags : IComponent
    {
        /// <summary>   The value. </summary>
        public byte value;
    }

    /// <summary>
    /// Parameters describing how to step the physics world. If none is present in the scene, default
    /// values will be used.
    /// </summary>
    public struct PhysicsStep
    {
        /// <summary>   Type of the simulation. </summary>
        public SimulationType SimulationType;
        /// <summary>   The gravity. </summary>
        public float3 Gravity;
        /// <summary>   Number of solver iterations. </summary>
        public int SolverIterationCount;
        /// <summary>   The solver stabilization heuristic settings. </summary>
        public Solver.StabilizationHeuristicSettings SolverStabilizationHeuristicSettings;

        /// <summary>   The multi threaded. </summary>
        public byte MultiThreaded;

        /// <summary>
        /// Whether to synchronize collision world after physics step to enable precise query results.
        /// Note that `BuildPhysicsWorld` will do this work on the following frame anyway, so only use
        /// this option when another system must know about the results of the simulation before the end
        /// of the frame (e.g., to destroy or create some other body that must be present in the
        /// following frame). In most cases, tolerating a frame of latency is easier to work with and is
        /// better for performance.
        /// </summary>
        public byte SynchronizeCollisionWorld;

        /// <summary>   (Immutable) the default. </summary>
        public static readonly PhysicsStep Default = new PhysicsStep
        {
            SimulationType = SimulationType.UnityPhysics,
            Gravity = -9.81f * math.up(),
            SolverIterationCount = 4,
            SolverStabilizationHeuristicSettings = Solver.StabilizationHeuristicSettings.Default,
            MultiThreaded = 1,
            SynchronizeCollisionWorld = 0
        };
    }

    /// <summary>
    /// The physics world singleton. Use it to access the <see cref="PhysicsWorld"/> used by
    /// simulation. If you want read only access to the world, use
    /// (SystemBase|SystemAPI|EntityQuery).GetSingleton&lt;PhysicsWorldSingleton&gt;(). If you want
    /// read write access to the world, use (SystemBase|SystemAPI|EntityQuery).GetSingletonRW&lt;
    /// PhysicsWorldSingleton&gt;().
    /// </summary>
    public struct PhysicsWorldSingleton : IComponent, ICollidable
    {
        /// <summary>   The physics world. </summary>
        public PhysicsWorld PhysicsWorld;

        #region PhysicsWorld API

        /// <summary>   Gets the collision world. </summary>
        ///
        /// <value> The collision world. </value>
        public CollisionWorld CollisionWorld => PhysicsWorld.CollisionWorld;

        /// <summary>   Gets the dynamics world. </summary>
        ///
        /// <value> The dynamics world. </value>
        public DynamicsWorld DynamicsWorld => PhysicsWorld.DynamicsWorld;

        /// <summary>   Gets the number of bodies. </summary>
        ///
        /// <value> The total number of bodies. </value>
        public int NumBodies => PhysicsWorld.NumBodies;

        /// <summary>   Gets the number of static bodies. </summary>
        ///
        /// <value> The total number of static bodies. </value>
        public int NumStaticBodies => PhysicsWorld.NumStaticBodies;

        /// <summary>   Gets the number of dynamic bodies. </summary>
        ///
        /// <value> The total number of dynamic bodies. </value>
        public int NumDynamicBodies => PhysicsWorld.NumDynamicBodies;

        /// <summary>   Gets the number of joints. </summary>
        ///
        /// <value> The total number of joints. </value>
        public int NumJoints => PhysicsWorld.NumJoints;

        /// <summary>   Gets the bodies. </summary>
        ///
        /// <value> The bodies. </value>
        public NativeArray<RigidBody> Bodies => PhysicsWorld.Bodies;

        /// <summary>   Gets the static bodies. </summary>
        ///
        /// <value> The static bodies. </value>
        public NativeArray<RigidBody> StaticBodies => PhysicsWorld.StaticBodies;

        /// <summary>   Gets the dynamic bodies. </summary>
        ///
        /// <value> The dynamic bodies. </value>
        public NativeArray<RigidBody> DynamicBodies => PhysicsWorld.DynamicBodies;

        /// <summary>   Gets the motion datas. </summary>
        ///
        /// <value> The motion datas. </value>
        public NativeArray<MotionData> MotionDatas => PhysicsWorld.MotionDatas;

        /// <summary>   Gets the motion velocities. </summary>
        ///
        /// <value> The motion velocities. </value>
        public NativeArray<MotionVelocity> MotionVelocities => PhysicsWorld.MotionVelocities;

        /// <summary>   Gets the joints. </summary>
        ///
        /// <value> The joints. </value>
        public NativeArray<Joint> Joints => PhysicsWorld.Joints;

        /// <summary>   Calculates the aabb. </summary>
        ///
        /// <returns>   The calculated aabb. </returns>
        public Aabb CalculateAabb() => PhysicsWorld.CalculateAabb();

        /// <summary>   Overlap aabb. </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapAabb(OverlapAabbInput input, ref NativeList<int> allHits) => PhysicsWorld.OverlapAabb(input, ref allHits);

        /// <summary>   Gets the zero-based index of the rigid body. </summary>
        ///
        /// <param name="entity">   The entity. </param>
        ///
        /// <returns>   The rigid body index. </returns>
        public int GetRigidBodyIndex(EntityId entity) => PhysicsWorld.GetRigidBodyIndex(entity);

        /// <summary>   Gets the zero-based index of the joint. </summary>
        ///
        /// <param name="entity">   The entity. </param>
        ///
        /// <returns>   The joint index. </returns>
        public int GetJointIndex(EntityId entity) => PhysicsWorld.GetJointIndex(entity);

        #endregion

        #region DOTS API Queries

        /// <summary>   Cast ray. </summary>
        ///
        /// <param name="input">    The input. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastRay(RaycastInput input)
            => QueryWrappers.RayCast(in this, input);

        /// <summary>   Cast ray. </summary>
        ///
        /// <param name="input">        The input. </param>
        /// <param name="closestHit">   [out] The closest hit. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastRay(RaycastInput input, out RaycastHit closestHit)
            => QueryWrappers.RayCast(in this, input, out closestHit);

        /// <summary>   Cast ray. </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits)
            => QueryWrappers.RayCast(in this, input, ref allHits);

        /// <summary>   Cast ray. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="input">        The input. </param>
        /// <param name="collector">    [in,out] The collector. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastRay<T>(RaycastInput input, ref T collector)
            where T : struct, ICollector<RaycastHit>
            => PhysicsWorld.CastRay(input, ref collector);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">    The input. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(PointDistanceInput input)
            => QueryWrappers.CalculateDistance(in this, input);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">        The input. </param>
        /// <param name="closestHit">   [out] The closest hit. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit)
            => QueryWrappers.CalculateDistance(in this, input, out closestHit);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits)
            => QueryWrappers.CalculateDistance(in this, input, ref allHits);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="input">        The input. </param>
        /// <param name="collector">    [in,out] The collector. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector)
            where T : struct, ICollector<DistanceHit>
            => PhysicsWorld.CalculateDistance(input, ref collector);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">    The input. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(ColliderDistanceInput input)
            => QueryWrappers.CalculateDistance(in this, input);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">        The input. </param>
        /// <param name="closestHit">   [out] The closest hit. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit)
            => QueryWrappers.CalculateDistance(in this, input, out closestHit);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits)
            => QueryWrappers.CalculateDistance(in this, input, ref allHits);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="input">        The input. </param>
        /// <param name="collector">    [in,out] The collector. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector)
            where T : struct, ICollector<DistanceHit>
            => PhysicsWorld.CalculateDistance(input, ref collector);

        /// <summary>   Cast collider. </summary>
        ///
        /// <param name="input">    The input. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider(ColliderCastInput input)
            => QueryWrappers.ColliderCast(in this, input);

        /// <summary>   Cast collider. </summary>
        ///
        /// <param name="input">        The input. </param>
        /// <param name="closestHit">   [out] The closest hit. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit)
            => QueryWrappers.ColliderCast(in this, input, out closestHit);

        /// <summary>   Cast collider. </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits)
            => QueryWrappers.ColliderCast(in this, input, ref allHits);

        /// <summary>   Cast collider. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="input">        The input. </param>
        /// <param name="collector">    [in,out] The collector. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider<T>(ColliderCastInput input, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
            => PhysicsWorld.CastCollider(input, ref collector);

        #endregion

        #region GO API Queries

        /// <summary>   Check capsule. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CheckCapsule(float3 point1, float3 point2, float radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckCapsule(in this, point1, point2, radius, filter, queryInteraction);

        /// <summary>   Overlap capsule. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapCapsule(float3 point1, float3 point2, float radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapCapsule(in this, point1, point2, radius, ref outHits, filter, queryInteraction);

        /// <summary>   Overlap capsule custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapCapsuleCustom<T>(float3 point1, float3 point2, float radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapCapsuleCustom(in this, point1, point2, radius, ref collector, filter, queryInteraction);

        /// <summary>   Check sphere. </summary>
        ///
        /// <param name="position">         The position. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CheckSphere(float3 position, float radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckSphere(in this, position, radius, filter, queryInteraction);

        /// <summary>   Overlap sphere. </summary>
        ///
        /// <param name="position">         The position. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapSphere(float3 position, float radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapSphere(in this, position, radius, ref outHits, filter, queryInteraction);

        /// <summary>   Overlap sphere custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="position">         The position. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapSphereCustom<T>(float3 position, float radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapSphereCustom(in this, position, radius, ref collector, filter, queryInteraction);

        /// <summary>   Check box. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CheckBox(float3 center, quaternion orientation, float3 halfExtents, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckBox(in this, center, orientation, halfExtents, filter, queryInteraction);

        /// <summary>   Overlap box. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapBox(float3 center, quaternion orientation, float3 halfExtents, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapBox(in this, center, orientation, halfExtents, ref outHits, filter, queryInteraction);

        /// <summary>   Overlap box custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapBoxCustom<T>(float3 center, quaternion orientation, float3 halfExtents, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapBoxCustom(in this, center, orientation, halfExtents, ref collector, filter, queryInteraction);

        /// <summary>   Sphere cast. </summary>
        ///
        /// <param name="origin">           The origin. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool SphereCast(float3 origin, float radius, float3 direction, float maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(in this, origin, radius, direction, maxDistance, filter, queryInteraction);

        /// <summary>   Sphere cast. </summary>
        ///
        /// <param name="origin">           The origin. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="hitInfo">          [out] Information describing the hit. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool SphereCast(float3 origin, float radius, float3 direction, float maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(in this, origin, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);

        /// <summary>   Sphere cast all. </summary>
        ///
        /// <param name="origin">           The origin. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool SphereCastAll(float3 origin, float radius, float3 direction, float maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCastAll(in this, origin, radius, direction, maxDistance, ref outHits, filter, queryInteraction);

        /// <summary>   Sphere cast custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="origin">           The origin. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool SphereCastCustom<T>(float3 origin, float radius, float3 direction, float maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.SphereCastCustom(in this, origin, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        /// <summary>   Box cast. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, float maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(in this, center, orientation, halfExtents, direction, maxDistance, filter, queryInteraction);

        /// <summary>   Box cast. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="hitInfo">          [out] Information describing the hit. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, float maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(in this, center, orientation, halfExtents, direction, maxDistance, out hitInfo, filter, queryInteraction);

        /// <summary>   Box cast all. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool BoxCastAll(float3 center, quaternion orientation, float3 halfExtents, float3 direction, float maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCastAll(in this, center, orientation, halfExtents, direction, maxDistance, ref outHits, filter, queryInteraction);

        /// <summary>   Box cast custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool BoxCastCustom<T>(float3 center, quaternion orientation, float3 halfExtents, float3 direction, float maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.BoxCastCustom(in this, center, orientation, halfExtents, direction, maxDistance, ref collector, filter, queryInteraction);

        /// <summary>   Capsule cast. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CapsuleCast(float3 point1, float3 point2, float radius, float3 direction, float maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(in this, point1, point2, radius, direction, maxDistance, filter, queryInteraction);

        /// <summary>   Capsule cast. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="hitInfo">          [out] Information describing the hit. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CapsuleCast(float3 point1, float3 point2, float radius, float3 direction, float maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(in this, point1, point2, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);

        /// <summary>   Capsule cast all. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CapsuleCastAll(float3 point1, float3 point2, float radius, float3 direction, float maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCastAll(in this, point1, point2, radius, direction, maxDistance, ref outHits, filter, queryInteraction);

        /// <summary>   Capsule cast custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CapsuleCastCustom<T>(float3 point1, float3 point2, float radius, float3 direction, float maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.CapsuleCastCustom(in this, point1, point2, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        #endregion
    }
}
