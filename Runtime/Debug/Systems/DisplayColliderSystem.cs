#if UNITY_EDITOR
using Unity.Burst;
using Unity.Collections.Blobs;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Scellecs.Morpeh.Native;
using Scellecs.Morpeh.Physics.Baking;
using Scellecs.Morpeh.Transforms;
using Unity.DebugDisplay;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;
using static Scellecs.Morpeh.Physics.Math;
#if MORPEH_ELYSIUM
using Scellecs.Morpeh.Elysium;
#endif

namespace Scellecs.Morpeh.Physics.Debug
{
#if MORPEH_ELYSIUM
    public sealed class DisplayColliderSystem : IUpdateSystem
#else
    public sealed class DisplayColliderSystem : ISystem
#endif
    {
        public World World { get; set; }

        private PrimitiveColliderGeometries defaultGeometries;

        private Filter dynamicFilter;
        private Filter staticFilter;

        private Stash<LocalToWorld> localToWorldStash;
        private Stash<LocalTransform> localTransformStash;
        private Stash<PhysicsCollider> colliderStash;
        private Stash<PhysicsMass> massStash;

        public void OnAwake()
        {
            dynamicFilter = World.Filter
                .With<PhysicsCollider>()
                .With<LocalToWorld>()
                .With<LocalTransform>()
                .With<PhysicsMass>()
                .Build();

            staticFilter = World.Filter
                .With<PhysicsCollider>()
                .With<LocalToWorld>()
                .With<LocalTransform>()
                .Without<PhysicsMass>()
                .Build();

            localToWorldStash = World.GetStash<LocalToWorld>();
            localTransformStash = World.GetStash<LocalTransform>();
            colliderStash = World.GetStash<PhysicsCollider>();
            massStash = World.GetStash<PhysicsMass>();

            DrawColliderUtility.CreateGeometries(out defaultGeometries);
        }

        public void OnUpdate(float deltaTime)
        {
            if (dynamicFilter.IsEmpty() && staticFilter.IsEmpty())
            {
                return;
            }

            var dynamicFilterNative = dynamicFilter.AsNative();
            var staticFilterNative = staticFilter.AsNative();

            var dynamicCount = dynamicFilterNative.length;
            var staticCount = staticFilterNative.length;

            var localToWorldStashNative = localToWorldStash.AsNative();
            var localTransformStashNative = localTransformStash.AsNative();
            var colliderStashNative = colliderStash.AsNative();
            var massStashNative = massStash.AsNative();

            var bodies = new NativeArray<RigidBody>(dynamicCount + staticCount, Allocator.TempJob);
            var motionTypes = new NativeArray<BodyMotionType>(dynamicCount + staticCount, Allocator.TempJob);

            var dynamicHandle = new CreateDynamicBodiesJob()
            {
                filter = dynamicFilterNative,
                localToWorldStash = localToWorldStashNative,
                localTransformStash = localTransformStashNative,
                colliderStash = colliderStashNative,
                massStash = massStashNative,
                bodies = bodies,
                motionTypes = motionTypes
            }
            .ScheduleParallel(dynamicCount, 32, default);

            var staticHandle = new CreateStaticBodiesJob()
            {
                filter = staticFilterNative,
                localToWorldStash = localToWorldStashNative,
                localTransformStash = localTransformStashNative,
                colliderStash = colliderStashNative,
                startIndex = dynamicCount,
                bodies = bodies,
                motionTypes = motionTypes
            }
            .ScheduleParallel(staticCount, 32, default);

            var displayHandle = DisplayColliderFacesJob.ScheduleJob(bodies, motionTypes, 1f, defaultGeometries, JobHandle.CombineDependencies(staticHandle, dynamicHandle));
            var disposeHandle = JobHandle.CombineDependencies(bodies.Dispose(displayHandle), motionTypes.Dispose(displayHandle));

            disposeHandle.Complete();
        }

        public void Dispose()
        {
            defaultGeometries.Dispose();
        }
    }

    [BurstCompile]
    internal struct CreateDynamicBodiesJob : IJobFor
    {
        public NativeFilter filter;
        public NativeStash<LocalToWorld> localToWorldStash;
        public NativeStash<LocalTransform> localTransformStash;
        public NativeStash<PhysicsCollider> colliderStash;
        public NativeStash<PhysicsMass> massStash;

        [NativeDisableParallelForRestriction] public NativeArray<BodyMotionType> motionTypes;
        [NativeDisableParallelForRestriction] public NativeArray<RigidBody> bodies;

        public void Execute(int index)
        {
            var entityId = filter[index];

            ref var localToWorld = ref localToWorldStash.Get(entityId);
            ref var localTransfrom = ref localTransformStash.Get(entityId);
            ref var mass = ref massStash.Get(entityId);
            ref var collider = ref colliderStash.Get(entityId);

            bodies[index] = new RigidBody()
            {
                Collider = collider.value,
                Scale = localTransfrom.scale,
                WorldFromBody = DecomposeRigidBodyTransform(localToWorld.value)
            };

            motionTypes[index] = mass.IsKinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic;
        }
    }

    [BurstCompile]
    internal struct CreateStaticBodiesJob : IJobFor
    {
        public NativeFilter filter;
        public NativeStash<LocalToWorld> localToWorldStash;
        public NativeStash<LocalTransform> localTransformStash;
        public NativeStash<PhysicsCollider> colliderStash;

        public int startIndex;

        [NativeDisableParallelForRestriction] public NativeArray<BodyMotionType> motionTypes;
        [NativeDisableParallelForRestriction] public NativeArray<RigidBody> bodies;

        public void Execute(int index)
        {
            var entityId = filter[index];

            ref var localToWorld = ref localToWorldStash.Get(entityId);
            ref var localTransfrom = ref localTransformStash.Get(entityId);
            ref var collider = ref colliderStash.Get(entityId);

            index = startIndex + index;

            bodies[index] = new RigidBody()
            {
                Collider = collider.value,
                Scale = localTransfrom.scale,
                WorldFromBody = DecomposeRigidBodyTransform(localToWorld.value)
            };

            motionTypes[index] = BodyMotionType.Static;
        }
    }

    [BurstCompile(FloatPrecision.Low, FloatMode.Fast)]
    internal struct DisplayColliderFacesJob : IJobParallelFor
    {
        [ReadOnly] private NativeArray<RigidBody> RigidBodies;
        [ReadOnly] private NativeArray<BodyMotionType> BodiesMotionTypes;
        [ReadOnly] private PrimitiveColliderGeometries Geometries;
        [ReadOnly] private float CollidersFacesScale;

        internal static JobHandle ScheduleJob(in NativeArray<RigidBody> rigidBodies, in NativeArray<BodyMotionType> bodiesMotionTypes, float collidersFacesScale, in PrimitiveColliderGeometries geometries, JobHandle inputDeps)
        {
            return new DisplayColliderFacesJob
            {
                RigidBodies = rigidBodies,
                BodiesMotionTypes = bodiesMotionTypes,
                Geometries = geometries,
                CollidersFacesScale = collidersFacesScale
            }.Schedule(rigidBodies.Length, 16, inputDeps);
        }

        public void Execute(int i)
        {
            var rigidBody = RigidBodies[i];
            var bodyMotionType = BodiesMotionTypes[i];
            var collider = rigidBody.Collider;

            if (collider.IsCreated)
            {
                DrawColliderFaces(collider, rigidBody.WorldFromBody, bodyMotionType, rigidBody.Scale * CollidersFacesScale);
            }
        }

        private unsafe void DrawColliderFaces(BlobAssetReference<Collider> collider, RigidTransform worldFromCollider,
            BodyMotionType bodyMotionType, float uniformScale = 1.0f)
        {
            DrawColliderFaces((Collider*)collider.GetUnsafePtr(), worldFromCollider, bodyMotionType, uniformScale);
        }

        private unsafe void DrawColliderFaces(Collider* collider, RigidTransform worldFromCollider,
            BodyMotionType bodyMotionType, float uniformScale = 1.0f)
        {
            UnityEngine.Quaternion colliderOrientation;
            float3 colliderPosition;
            float radius;
            ColorIndex color = DrawColliderUtility.GetColorIndex(bodyMotionType);

            switch (collider->Type)
            {
                case ColliderType.Cylinder:
                    radius = ((CylinderCollider*)collider)->Radius;
                    var height = ((CylinderCollider*)collider)->Height;
                    colliderPosition = ((CylinderCollider*)collider)->Center;
                    colliderOrientation = ((CylinderCollider*)collider)->Orientation * Quaternion.FromToRotation(Vector3.up, Vector3.back);

                    DrawColliderUtility.DrawPrimitiveCylinderFaces(radius, height, colliderPosition, colliderOrientation, worldFromCollider, ref Geometries.CylinderGeometry, color, uniformScale);
                    break;

                case ColliderType.Box:
                    colliderPosition = ((BoxCollider*)collider)->Center;
                    var size = ((BoxCollider*)collider)->Size;
                    colliderOrientation = ((BoxCollider*)collider)->Orientation;

                    DrawColliderUtility.DrawPrimitiveBoxFaces(size, colliderPosition, colliderOrientation, worldFromCollider, ref Geometries.BoxGeometry, color, uniformScale);
                    break;

                case ColliderType.Triangle:
                case ColliderType.Quad:
                case ColliderType.Convex:
                    DrawConvexFaces(ref ((ConvexCollider*)collider)->ConvexHull, worldFromCollider, color, uniformScale);
                    break;

                case ColliderType.Sphere:
                    radius = ((SphereCollider*)collider)->Radius;
                    colliderPosition = ((SphereCollider*)collider)->Center;

                    DrawColliderUtility.DrawPrimitiveSphereFaces(radius, colliderPosition, worldFromCollider, ref Geometries.SphereGeometry, color, uniformScale);
                    break;

                case ColliderType.Capsule:
                    radius = ((CapsuleCollider*)collider)->Radius;

                    var vertex0 = ((CapsuleCollider*)collider)->Vertex0;
                    var vertex1 = ((CapsuleCollider*)collider)->Vertex1;
                    var axis = vertex1 - vertex0; //axis in wfc-space

                    height = 0.5f * math.length(axis) + radius;

                    colliderPosition = (vertex1 + vertex0) / 2.0f; //axis in wfc-space
                    colliderOrientation = Quaternion.FromToRotation(Vector3.up, -axis);

                    DrawColliderUtility.DrawPrimitiveCapsuleFaces(radius, height, colliderPosition, colliderOrientation, worldFromCollider, ref Geometries.CapsuleGeometry, color, uniformScale);

                    break;

                case ColliderType.Mesh:
                    DrawMeshColliderFaces((MeshCollider*)collider, worldFromCollider, color, uniformScale);
                    break;

                case ColliderType.Compound:
                    DrawCompoundColliderFaces((CompoundCollider*)collider, worldFromCollider, bodyMotionType, uniformScale);
                    break;

                case ColliderType.Terrain:
                    //TODO [#3792]: Terrain should use DebugDraw rather than Gizmos and add uniform scale support.
                    //AppendMeshColliders.GetMeshes.AppendTerrain((TerrainCollider*)collider, worldFromCollider, ref results);
                    break;
            }
        }

        // Covers: collider->Type = Box, Triangle, Quad, Convex, Cylinder(before started using primitives)
        private static void DrawConvexFaces(ref ConvexHull hull, RigidTransform worldFromCollider,
            ColorIndex ci, float uniformScale = 1.0f)
        {
            //for (var f = 0; f < hull.NumFaces; f++)
            //{
            //    var countVert = hull.Faces[f].NumVertices;

            //    if (countVert == 3) // A triangle
            //    {
            //        var vertices = new NativeArray<float3>(3, Allocator.Temp);
            //        for (var fv = 0; fv < countVert; fv++)
            //        {
            //            var origVertexIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + fv];
            //            vertices[fv] = uniformScale * hull.Vertices[origVertexIndex];
            //        }
            //        DrawColliderUtility.DrawTriangle(vertices[0], vertices[1], vertices[2], worldFromCollider, ci);
            //        vertices.Dispose();
            //    }
            //    else if (countVert == 4) // A quad: break into two triangles
            //    {
            //        var vertices = new NativeArray<float3>(4, Allocator.Temp);
            //        for (var fv = 0; fv < countVert; fv++)
            //        {
            //            var origVertexIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + fv];
            //            vertices[fv] = uniformScale * hull.Vertices[origVertexIndex];
            //        }
            //        DrawColliderUtility.DrawTriangle(vertices[0], vertices[1], vertices[2], worldFromCollider, ci);
            //        DrawColliderUtility.DrawTriangle(vertices[2], vertices[3], vertices[0], worldFromCollider, ci);
            //        vertices.Dispose();
            //    }
            //    else // find the average vertex and then use to break into triangles
            //    {
            //        var faceCentroid = float3.zero;
            //        var scaledVertices = new NativeArray<float3>(countVert, Allocator.Temp);
            //        for (var i = 0; i < countVert; i++)
            //        {
            //            var origVertexIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + i];
            //            scaledVertices[i] = uniformScale * hull.Vertices[origVertexIndex];

            //            faceCentroid += scaledVertices[i];
            //        }
            //        faceCentroid /= countVert;

            //        for (var j = 0; j < countVert; j++)
            //        {
            //            var vertices = new NativeArray<float3>(3, Allocator.Temp);
            //            if (j < countVert - 1)
            //            {
            //                vertices[0] = scaledVertices[j];
            //                vertices[1] = scaledVertices[j + 1];
            //            }
            //            else //close the circle of triangles
            //            {
            //                vertices[0] = scaledVertices[j];
            //                vertices[1] = scaledVertices[0];
            //            }
            //            vertices[2] = faceCentroid;
            //            DrawColliderUtility.DrawTriangle(vertices[0], vertices[1], vertices[2], worldFromCollider, ci);
            //            vertices.Dispose();
            //        }
            //        scaledVertices.Dispose();
            //    }
            //}

            var triangleVertices = new NativeList<float3>(Allocator.Temp);
            try
            {
                // set some best guess capacity, assuming we have on average of 3 triangles per face,
                // and given the fact that we need 3 vertices to define a triangle.
                const int kAvgTrianglesPerFace = 3;
                const int kNumVerticesPerTriangle = 3;
                triangleVertices.Capacity = hull.NumFaces * kAvgTrianglesPerFace * kNumVerticesPerTriangle;

                unsafe
                {
                    var vertexBuffer = stackalloc float3[ConvexCollider.k_MaxFaceVertices];
                    for (var f = 0; f < hull.NumFaces; f++)
                    {
                        var countVert = hull.Faces[f].NumVertices;

                        if (countVert == 3) // A triangle
                        {
                            for (var fv = 0; fv < countVert; fv++)
                            {
                                var origVertexIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + fv];
                                triangleVertices.Add(math.transform(worldFromCollider,
                                    uniformScale * hull.Vertices[origVertexIndex]));
                            }
                        }
                        else if (countVert == 4) // A quad: break into two triangles
                        {
                            for (var fv = 0; fv < countVert; fv++)
                            {
                                var origVertexIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + fv];
                                vertexBuffer[fv] = math.transform(worldFromCollider,
                                    uniformScale * hull.Vertices[origVertexIndex]);
                            }

                            // triangle 0, 1, 2
                            triangleVertices.Add(vertexBuffer[0]);
                            triangleVertices.Add(vertexBuffer[1]);
                            triangleVertices.Add(vertexBuffer[2]);
                            // triangle 2, 3, 0
                            triangleVertices.Add(vertexBuffer[2]);
                            triangleVertices.Add(vertexBuffer[3]);
                            triangleVertices.Add(vertexBuffer[0]);
                        }
                        else // find the average vertex and then use to break into triangles
                        {
                            // Todo: we can avoid using the centroid as an extra vertex by simply walking around the face
                            // and producing triangles with the first vertex and every next pair of vertices.

                            var faceCentroid = float3.zero;
                            for (var i = 0; i < countVert; i++)
                            {
                                var origVertexIndex = hull.FaceVertexIndices[hull.Faces[f].FirstIndex + i];
                                var scaledVertex = math.transform(worldFromCollider, uniformScale * hull.Vertices[origVertexIndex]);
                                faceCentroid += scaledVertex;

                                vertexBuffer[i] = scaledVertex;
                            }

                            faceCentroid /= countVert;

                            for (var j = 0; j < countVert; j++)
                            {
                                var vertices = new float3x3();
                                if (j < countVert - 1)
                                {
                                    vertices[0] = vertexBuffer[j];
                                    vertices[1] = vertexBuffer[j + 1];
                                }
                                else //close the circle of triangles
                                {
                                    vertices[0] = vertexBuffer[j];
                                    vertices[1] = vertexBuffer[0];
                                }

                                vertices[2] = faceCentroid;

                                triangleVertices.Add(vertices[0]);
                                triangleVertices.Add(vertices[1]);
                                triangleVertices.Add(vertices[2]);
                            }
                        }
                    }
                }

                PhysicsDebugDisplay.Triangles(triangleVertices, ci);
            }
            finally
            {
                triangleVertices.Dispose();
            }
        }

        private unsafe void DrawCompoundColliderFaces(CompoundCollider* compoundCollider, RigidTransform worldFromCollider,
            BodyMotionType bodyMotionType, float uniformScale = 1.0f)
        {
            for (var i = 0; i < compoundCollider->Children.Length; i++)
            {
                ref CompoundCollider.Child child = ref compoundCollider->Children[i];

                ScaledMTransform mWorldFromCompound = new ScaledMTransform(worldFromCollider, 1.0f);
                ScaledMTransform mWorldFromChild = ScaledMTransform.Mul(mWorldFromCompound, new MTransform(child.CompoundFromChild));
                RigidTransform worldFromChild = new RigidTransform(mWorldFromChild.Rotation, mWorldFromChild.Translation);

                DrawColliderFaces(child.Collider, worldFromChild, bodyMotionType, uniformScale);
            }
        }

        private static unsafe void DrawMeshColliderFaces(MeshCollider* meshCollider, RigidTransform worldFromCollider, ColorIndex ci, float uniformScale = 1.0f)
        {
            ref Mesh mesh = ref meshCollider->Mesh;

            float4x4 worldMatrix = new float4x4(worldFromCollider);
            worldMatrix.c0 *= uniformScale;
            worldMatrix.c1 *= uniformScale;
            worldMatrix.c2 *= uniformScale;

            var nothing = new RigidTransform();
            for (int sectionIndex = 0; sectionIndex < mesh.Sections.Length; sectionIndex++)
            {
                ref Mesh.Section section = ref mesh.Sections[sectionIndex];
                for (int primitiveIndex = 0; primitiveIndex < section.PrimitiveVertexIndices.Length; primitiveIndex++)
                {
                    Mesh.PrimitiveVertexIndices vertexIndices = section.PrimitiveVertexIndices[primitiveIndex];
                    Mesh.PrimitiveFlags flags = section.PrimitiveFlags[primitiveIndex];
                    var numTriangles = 1;
                    if ((flags & Mesh.PrimitiveFlags.IsTrianglePair) != 0)
                    {
                        numTriangles = 2;
                    }

                    float3x4 v = new float3x4(
                        math.transform(worldMatrix, section.Vertices[vertexIndices.A]),
                        math.transform(worldMatrix, section.Vertices[vertexIndices.B]),
                        math.transform(worldMatrix, section.Vertices[vertexIndices.C]),
                        math.transform(worldMatrix, section.Vertices[vertexIndices.D]));

                    for (int triangleIndex = 0; triangleIndex < numTriangles; triangleIndex++)
                    {
                        DrawColliderUtility.DrawTriangle(v[0], v[1 + triangleIndex], v[2 + triangleIndex], nothing, ci);
                    }
                }
            }
        }
    }
}
#endif
