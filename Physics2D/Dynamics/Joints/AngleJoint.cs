/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

/*
* Farseer Physics Engine:
* Copyright (c) 2012 Ian Qvist
*/

using FixedMath.NET;
using System;
using System.Diagnostics;
using tainicom.Aether.Physics2D.Common;
#if XNAAPI
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Dynamics.Joints
{
    /// <summary>
    /// Maintains a fixed angle between two bodies
    /// </summary>
    public class AngleJoint : Joint
    {
        private Fix64 _bias;
        private Fix64 _jointError;
        private Fix64 _massFactor;
        private Fix64 _targetAngle;

        internal AngleJoint()
        {
            JointType = JointType.Angle;
        }

        /// <summary>
        /// Constructor for AngleJoint
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        public AngleJoint(Body bodyA, Body bodyB)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Angle;
            BiasFactor = Fix64Constants.PointTwo;
            MaxImpulse = Fix64.MaxValue;
        }

        public override AetherVector2 WorldAnchorA
        {
            get { return BodyA.Position; }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        public override AetherVector2 WorldAnchorB
        {
            get { return BodyB.Position; }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        /// <summary>
        /// The desired angle between BodyA and BodyB
        /// </summary>
        public Fix64 TargetAngle
        {
            get { return _targetAngle; }
            set
            {
                if (value != _targetAngle)
                {
                    _targetAngle = value;
                    WakeBodies();
                }
            }
        }

        /// <summary>
        /// Gets or sets the bias factor.
        /// Defaults to 0.2
        /// </summary>
        public Fix64 BiasFactor { get; set; }
        
        /// <summary>
        /// Gets or sets the maximum impulse
        /// Defaults to Fix64.MaxValue
        /// </summary>
        public Fix64 MaxImpulse { get; set; }
        
        /// <summary>
        /// Gets or sets the softness of the joint
        /// Defaults to 0
        /// </summary>
        public Fix64 Softness { get; set; }

        public override AetherVector2 GetReactionForce(Fix64 invDt)
        {
            //TODO
            //return _inv_dt * _impulse;
            return AetherVector2.Zero;
        }

        public override Fix64 GetReactionTorque(Fix64 invDt)
        {
            return Fix64.Zero;
        }

        internal override void InitVelocityConstraints(ref SolverData data)
        {
            int indexA = BodyA.IslandIndex;
            int indexB = BodyB.IslandIndex;

            Fix64 aW = data.positions[indexA].a;
            Fix64 bW = data.positions[indexB].a;

            _jointError = (bW - aW - TargetAngle);
            _bias = -BiasFactor * data.step.inv_dt * _jointError;
            _massFactor = (Fix64.One - Softness) / (BodyA._invI + BodyB._invI);
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            int indexA = BodyA.IslandIndex;
            int indexB = BodyB.IslandIndex;

            Fix64 p = (_bias - data.velocities[indexB].w + data.velocities[indexA].w) * _massFactor;

            data.velocities[indexA].w -= BodyA._invI * MathUtils.Sign(p) * MathUtils.Min( Fix64.Abs(p), MaxImpulse);
            data.velocities[indexB].w += BodyB._invI * MathUtils.Sign(p) * MathUtils.Min( Fix64.Abs(p), MaxImpulse);
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            //no position solving for this joint
            return true;
        }
    }
}