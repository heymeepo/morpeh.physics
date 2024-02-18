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
    [Serializable]
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
    [Serializable]
    public struct PhysicsMassOverride : IComponent
    {
        /// <summary>   The is kinematic flag. </summary>
        public byte IsKinematic;
        /// <summary>   The set velocity to zero flag. </summary>
        public byte SetVelocityToZero;
    }

    /// <summary>   The velocity of a rigid body. If absent, the rigid body is static. </summary>
    [Serializable]
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
    [Serializable]
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
    [Serializable]
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
    [Serializable]
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
}
