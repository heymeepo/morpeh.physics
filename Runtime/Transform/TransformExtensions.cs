using System.Runtime.CompilerServices;
using Unity.Mathematics;
using static Unity.Mathematics.math;
using static Unity.Mathematics.quaternion;

namespace Scellecs.Morpeh.Transform
{
    public static class TransformExtensions
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 Forward(this ref TransformComponent trs) => mul(trs.rotation, forward());

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Forward(this ref TransformComponent trs, float3 value) => trs.rotation = LookRotationSafe(value, abs(dot(value, up())) < 0.9999f ? up() : forward());

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 Up(this ref TransformComponent trs) => mul(trs.rotation, up());

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Up(this ref TransformComponent trs, float3 value) => trs.rotation = LookRotationSafe(abs(dot(value, forward())) < 0.9999f ? forward() : up(), value);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float3 Right(this ref TransformComponent trs) => cross(trs.Up(), trs.Forward());

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Right(this ref TransformComponent trs, float3 value)
        {
            trs.Up(normalizesafe(cross(trs.Forward(), value)));
            trs.Forward(normalizesafe(cross(value, trs.Up())));
        }
    }
}