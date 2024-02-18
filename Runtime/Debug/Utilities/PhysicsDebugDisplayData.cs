using UnityEngine;

namespace Unity.DebugDisplay
{
    internal sealed class PhysicsDebugDisplayData : ScriptableObject
    {
        internal const string ASSET_PATH = "Assets/Plugins/Scellecs/Morpeh Physics/Assets/PhysicsDisplayDataAsset.asset";
        /// <summary>
        /// Enable or disable collider debug display.
        /// </summary>
        public bool DrawColliders;

        /// <summary>
        /// Enable or disable debug display of collider edges.
        /// </summary>
        public bool DrawColliderEdges;
    }
}
