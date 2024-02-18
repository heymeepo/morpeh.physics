using Unity.Burst;

namespace Unity.DebugDisplay
{
    internal class DebugDisplay
    {
        internal static void Render()
        {
            Managed.Instance.CopyFromCpuToGpu();
            Managed.Instance.Render();
        }

        internal static void Clear()
        {
            Managed.Instance.Clear();
        }

        [BurstDiscard]
        internal static void Reinstantiate()
        {
            if (Managed.Instance != null)
            {
                Managed.Instance.Dispose();
            }
            Managed.Instance = new Managed();
        }

        [BurstDiscard]
        internal static void Instantiate()
        {
            if (Managed.Instance == null)
                Managed.Instance = new Managed();
        }
    }
}
