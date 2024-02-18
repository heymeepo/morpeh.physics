namespace Scellecs.Morpeh.Physics.Baking
{
    public enum BodyMotionType
    {
        /// <summary>
        /// The physics solver will move the rigid body and handle its collision response with other bodies, based on its physical properties.
        /// </summary>
        Dynamic,
        /// <summary>
        /// The physics solver will move the rigid body according to its velocity, but it will be treated as though it has infinite mass.
        /// It will generate a collision response with any rigid bodies that lie in its path of motion, but will not be affected by them.
        /// </summary>
        Kinematic,
        /// <summary>
        /// The physics solver will not move the rigid body.
        /// Any transformations applied to it will be treated as though it is teleporting.
        /// </summary>
        Static
    }
}
