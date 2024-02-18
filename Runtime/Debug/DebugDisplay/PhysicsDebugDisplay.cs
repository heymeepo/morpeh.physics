using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.DebugDisplay
{
    public static class PhysicsDebugDisplay
    {
        /// <summary>
        /// Draws a point.
        /// </summary>
        /// <param name="x"> World space position. </param>
        /// <param name="size"> Extents. </param>
        /// <param name="color"> Color. </param>
        public static void Point(float3 x, float size, Unity.DebugDisplay.ColorIndex color)
        {
            var lines = new Unity.DebugDisplay.Lines(3);

            lines.Draw(x - new float3(size, 0, 0), x + new float3(size, 0, 0), color);
            lines.Draw(x - new float3(0, size, 0), x + new float3(0, size, 0), color);
            lines.Draw(x - new float3(0, 0, size), x + new float3(0, 0, size), color);
        }

        /// <summary>
        /// Draws a line between 2 points.
        /// </summary>
        /// <param name="x0"> Point 0 in world space. </param>
        /// <param name="x1"> Point 1 in world space. </param>
        /// <param name="color"> Color. </param>
        public static void Line(float3 x0, float3 x1, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Line.Draw(x0, x1, color);
        }

        /// <summary>
        /// Draws an arrow.
        /// </summary>
        /// <param name="x"> World space position of the arrow base. </param>
        /// <param name="v"> Arrow direction with length. </param>
        /// <param name="color"> Color. </param>
        public static void Arrow(float3 x, float3 v, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Arrow.Draw(x, v, color);
        }

        /// <summary>
        /// Draws a plane.
        /// </summary>
        /// <param name="x"> Point in world space. </param>
        /// <param name="v"> Normal. </param>
        /// <param name="color"> Color. </param>
        public static void Plane(float3 x, float3 v, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Plane.Draw(x, v, color);
        }

        /// <summary>
        /// Draws an arc.
        /// </summary>
        /// <param name="center"> World space position of the arc center. </param>
        /// <param name="normal"> Arc normal. </param>
        /// <param name="arm"> Arc arm. </param>
        /// <param name="angle"> Arc angle. </param>
        /// <param name="color"> Color. </param>
        public static void Arc(float3 center, float3 normal, float3 arm, float angle,
            Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Arc.Draw(center, normal, arm, angle, color);
        }

        /// <summary>
        /// Draws a cone.
        /// </summary>
        /// <param name="point"> Point in world space. </param>
        /// <param name="axis"> Cone axis. </param>
        /// <param name="angle"> Cone angle. </param>
        /// <param name="color"> Color. </param>
        public static void Cone(float3 point, float3 axis, float angle, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Cone.Draw(point, axis, angle, color);
        }

        /// <summary>
        /// Draws a box.
        /// </summary>
        /// <param name="Size"> Size of the box. </param>
        /// <param name="Center"> Center of the box in world space. </param>
        /// <param name="Orientation"> Orientation of the box in world space. </param>
        /// <param name="color"> Color. </param>
        public static void Box(float3 Size, float3 Center, quaternion Orientation, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Box.Draw(Size, Center, Orientation, color);
        }

        /// <summary>
        /// Draws a triangle.
        /// </summary>
        /// <param name="vertex0"> Vertex 0 in world space. </param>
        /// <param name="vertex1"> Vertex 1 in world space. </param>
        /// <param name="vertex2"> Vertex 2 in world space. </param>
        /// <param name="normal"> Triangle normal. </param>
        /// <param name="color"> Color. </param>
        public static void Triangle(float3 vertex0, float3 vertex1, float3 vertex2, float3 normal, Unity.DebugDisplay.ColorIndex color)
        {
            Unity.DebugDisplay.Triangle.Draw(vertex0, vertex1, vertex2, normal, color);
        }

        /// <summary>
        /// Draws multiple triangles from the provided array of triplets of vertices.
        /// </summary>
        /// <param name="vertices"> An array containing a sequence of vertex triplets. A triangle is drawn from every triplet of vertices. </param>
        /// <param name="numVertices"> Number of vertices. </param>
        /// <param name="color"> Color. </param>
        public static unsafe void Triangles(float3* vertices, int numVertices, ColorIndex color)
        {
            var triangles = new Triangles(numVertices / 3);
            for (int i = 0; i < numVertices; i += 3)
            {
                var v0 = vertices[i];
                var v1 = vertices[i + 1];
                var v2 = vertices[i + 2];

                float3 normal = math.normalize(math.cross(v1 - v0, v2 - v0));
                triangles.Draw(v0, v1, v2, normal, color);
            }
        }


        /// <summary>
        /// Draws multiple triangles from the provided list of triplets of vertices.
        /// </summary>
        /// <param name="vertices"> A list containing a sequence of vertex triplets. A triangle is drawn from every triplet of vertices. </param>
        /// <param name="color"> Color. </param>
        public static void Triangles(in NativeList<float3> vertices, ColorIndex color)
        {
            unsafe
            {
                Triangles(vertices.GetUnsafePtr(), vertices.Length, color);
            }
        }

        /// <summary>
        /// Draws multiple triangles from the provided array of triplets of vertices.
        /// </summary>
        /// <param name="vertices"> An array containing a sequence of vertex triplets. A triangle is drawn from every triplet of vertices. </param>
        /// <param name="color"> Color. </param>
        public static void Triangles(in NativeArray<float3> vertices, ColorIndex color)
        {
            unsafe
            {
                Triangles((float3*)vertices.GetUnsafePtr(), vertices.Length, color);
            }
        }
    }
}
