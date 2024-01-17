using System.Runtime.CompilerServices;
using Unity.IL2CPP.CompilerServices;
using Unity.Mathematics;

namespace Scellecs.Morpeh.Transform
{
    [Il2CppSetOption(Option.NullChecks, false)]
    [Il2CppSetOption(Option.ArrayBoundsChecks, false)]
    [Il2CppSetOption(Option.DivideByZeroChecks, false)]
    [System.Serializable]
    public struct TransformComponent : IComponent
    {
        public float3 translation;
        public quaternion rotation;
        public float3 scale;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float4x4(TransformComponent trs) => float4x4.TRS(trs.translation, trs.rotation, trs.scale);
    }
}