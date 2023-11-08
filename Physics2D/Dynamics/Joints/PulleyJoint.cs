/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

/*
* Farseer Physics Engine:
* Copyright (c) 2012 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org 
* 
* This software is provided 'as-is', without any express or implied 
* warranty.  In no event will the authors be held liable for any damages 
* arising from the use of this software. 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it 
* freely, subject to the following restrictions: 
* 1. The origin of this software must not be misrepresented; you must not 
* claim that you wrote the original software. If you use this software 
* in a product, an acknowledgment in the product documentation would be 
* appreciated but is not required. 
* 2. Altered source versions must be plainly marked as such, and must not be 
* misrepresented as being the original software. 
* 3. This notice may not be removed or altered from any source distribution. 
*/

using FixedMath.NET;
using System;
using System.Diagnostics;
using tainicom.Aether.Physics2D.Common;
#if XNAAPI
using Complex = tainicom.Aether.Physics2D.Common.Complex;
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Dynamics.Joints
{
    // Pulley:
    // length1 = norm(p1 - s1)
    // length2 = norm(p2 - s2)
    // C0 = (length1 + ratio * length2)_initial
    // C = C0 - (length1 + ratio * length2)
    // u1 = (p1 - s1) / norm(p1 - s1)
    // u2 = (p2 - s2) / norm(p2 - s2)
    // Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
    // J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
    // K = J * invM * JT
    //   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

    /// <summary>
    /// The pulley joint is connected to two bodies and two fixed world points.
    /// The pulley supports a ratio such that:
    /// <![CDATA[length1 + ratio * length2 <= constant]]>
    /// Yes, the force transmitted is scaled by the ratio.
    /// 
    /// Warning: the pulley joint can get a bit squirrelly by itself. They often
    /// work better when combined with prismatic joints. You should also cover the
    /// the anchor points with static shapes to prevent one side from going to zero length.
    /// </summary>
    public class PulleyJoint : Joint
    {
        // Solver shared
        private Fix64 _impulse;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private AetherVector2 _uA;
        private AetherVector2 _uB;
        private AetherVector2 _rA;
        private AetherVector2 _rB;
        private AetherVector2 _localCenterA;
        private AetherVector2 _localCenterB;
        private Fix64 _invMassA;
        private Fix64 _invMassB;
        private Fix64 _invIA;
        private Fix64 _invIB;
        private Fix64 _mass;

        internal PulleyJoint()
        {
            JointType = JointType.Pulley;
        }

        /// <summary>
        /// Constructor for PulleyJoint.
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="anchorA">The anchor on the first body.</param>
        /// <param name="anchorB">The anchor on the second body.</param>
        /// <param name="worldAnchorA">The world anchor for the first body.</param>
        /// <param name="worldAnchorB">The world anchor for the second body.</param>
        /// <param name="ratio">The ratio.</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public PulleyJoint(Body bodyA, Body bodyB, AetherVector2 anchorA, AetherVector2 anchorB, AetherVector2 worldAnchorA, AetherVector2 worldAnchorB, Fix64 ratio, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Pulley;

            WorldAnchorA = worldAnchorA;
            WorldAnchorB = worldAnchorB;

            if (useWorldCoordinates)
            {
                LocalAnchorA = BodyA.GetLocalPoint(anchorA);
                LocalAnchorB = BodyB.GetLocalPoint(anchorB);

                AetherVector2 dA = anchorA - worldAnchorA;
                LengthA = dA.Length();
                AetherVector2 dB = anchorB - worldAnchorB;
                LengthB = dB.Length();
            }
            else
            {
                LocalAnchorA = anchorA;
                LocalAnchorB = anchorB;

                AetherVector2 dA = anchorA - BodyA.GetLocalPoint(worldAnchorA);
                LengthA = dA.Length();
                AetherVector2 dB = anchorB - BodyB.GetLocalPoint(worldAnchorB);
                LengthB = dB.Length();
            }

            Debug.Assert(ratio != Fix64.Zero);
            Debug.Assert(ratio > Settings.Epsilon);

            Ratio = ratio;
            Constant = LengthA + ratio * LengthB;
            _impulse = Fix64.Zero;
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public AetherVector2 LocalAnchorA { get; set; }

        /// <summary>
        /// The local anchor point on BodyB
        /// </summary>
        public AetherVector2 LocalAnchorB { get; set; }

        /// <summary>
        /// Get the first world anchor.
        /// </summary>
        /// <value></value>
        public override sealed AetherVector2 WorldAnchorA { get; set; }

        /// <summary>
        /// Get the second world anchor.
        /// </summary>
        /// <value></value>
        public override sealed AetherVector2 WorldAnchorB { get; set; }

        /// <summary>
        /// Get the current length of the segment attached to body1.
        /// </summary>
        /// <value></value>
        public Fix64 LengthA { get; set; }

        /// <summary>
        /// Get the current length of the segment attached to body2.
        /// </summary>
        /// <value></value>
        public Fix64 LengthB { get; set; }

        /// <summary>
        /// The current length between the anchor point on BodyA and WorldAnchorA
        /// </summary>
        public Fix64 CurrentLengthA
        {
            get
            {
                AetherVector2 p = BodyA.GetWorldPoint(LocalAnchorA);
                AetherVector2 s = WorldAnchorA;
                AetherVector2 d = p - s;
                return d.Length();
            }
        }

        /// <summary>
        /// The current length between the anchor point on BodyB and WorldAnchorB
        /// </summary>
        public Fix64 CurrentLengthB
        {
            get
            {
                AetherVector2 p = BodyB.GetWorldPoint(LocalAnchorB);
                AetherVector2 s = WorldAnchorB;
                AetherVector2 d = p - s;
                return d.Length();
            }
        }

        /// <summary>
        /// Get the pulley ratio.
        /// </summary>
        /// <value></value>
        public Fix64 Ratio { get; set; }

        //FPE note: Only used for serialization.
        internal Fix64 Constant { get; set; }

        public override AetherVector2 GetReactionForce(Fix64 invDt)
        {
            AetherVector2 P = _impulse * _uB;
            return invDt * P;
        }

        public override Fix64 GetReactionTorque(Fix64 invDt)
        {
            return Fix64.Zero;
        }

        internal override void InitVelocityConstraints(ref SolverData data)
        {
            _indexA = BodyA.IslandIndex;
            _indexB = BodyB.IslandIndex;
            _localCenterA = BodyA._sweep.LocalCenter;
            _localCenterB = BodyB._sweep.LocalCenter;
            _invMassA = BodyA._invMass;
            _invMassB = BodyB._invMass;
            _invIA = BodyA._invI;
            _invIB = BodyB._invI;

            AetherVector2 cA = data.positions[_indexA].c;
            Fix64 aA = data.positions[_indexA].a;
            AetherVector2 vA = data.velocities[_indexA].v;
            Fix64 wA = data.velocities[_indexA].w;

            AetherVector2 cB = data.positions[_indexB].c;
            Fix64 aB = data.positions[_indexB].a;
            AetherVector2 vB = data.velocities[_indexB].v;
            Fix64 wB = data.velocities[_indexB].w;

            Complex qA = Complex.FromAngle(aA);
            Complex qB = Complex.FromAngle(aB);

            _rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
            _rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);

            // Get the pulley axes.
            _uA = cA + _rA - WorldAnchorA;
            _uB = cB + _rB - WorldAnchorB;

            Fix64 lengthA = _uA.Length();
            Fix64 lengthB = _uB.Length();

            if (lengthA > Fix64Constants.Ten * Settings.LinearSlop)
            {
                _uA *= Fix64.One / lengthA;
            }
            else
            {
                _uA = AetherVector2.Zero;
            }

            if (lengthB > Fix64Constants.Ten * Settings.LinearSlop)
            {
                _uB *= Fix64.One / lengthB;
            }
            else
            {
                _uB = AetherVector2.Zero;
            }

            // Compute effective mass.
            Fix64 ruA = MathUtils.Cross(ref _rA, ref _uA);
            Fix64 ruB = MathUtils.Cross(ref _rB, ref _uB);

            Fix64 mA = _invMassA + _invIA * ruA * ruA;
            Fix64 mB = _invMassB + _invIB * ruB * ruB;

            _mass = mA + Ratio * Ratio * mB;

            if (_mass > Fix64.Zero)
            {
                _mass = Fix64.One / _mass;
            }

            if (data.step.warmStarting)
            {
                // Scale impulses to support variable time steps.
                _impulse *= data.step.dtRatio;

                // Warm starting.
                AetherVector2 PA = -(_impulse) * _uA;
                AetherVector2 PB = (-Ratio * _impulse) * _uB;

                vA += _invMassA * PA;
                wA += _invIA * MathUtils.Cross(ref _rA, ref PA);
                vB += _invMassB * PB;
                wB += _invIB * MathUtils.Cross(ref _rB, ref PB);
            }
            else
            {
                _impulse = Fix64.Zero;
            }

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            AetherVector2 vA = data.velocities[_indexA].v;
            Fix64 wA = data.velocities[_indexA].w;
            AetherVector2 vB = data.velocities[_indexB].v;
            Fix64 wB = data.velocities[_indexB].w;

            AetherVector2 vpA = vA + MathUtils.Cross(wA, ref _rA);
            AetherVector2 vpB = vB + MathUtils.Cross(wB, ref _rB);

            Fix64 Cdot = -AetherVector2.Dot(_uA, vpA) - Ratio * AetherVector2.Dot(_uB, vpB);
            Fix64 impulse = -_mass * Cdot;
            _impulse += impulse;

            AetherVector2 PA = -impulse * _uA;
            AetherVector2 PB = -Ratio * impulse * _uB;
            vA += _invMassA * PA;
            wA += _invIA * MathUtils.Cross(ref _rA, ref PA);
            vB += _invMassB * PB;
            wB += _invIB * MathUtils.Cross(ref _rB, ref PB);

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            AetherVector2 cA = data.positions[_indexA].c;
            Fix64 aA = data.positions[_indexA].a;
            AetherVector2 cB = data.positions[_indexB].c;
            Fix64 aB = data.positions[_indexB].a;

            Complex qA = Complex.FromAngle(aA);
            Complex qB = Complex.FromAngle(aB);

            AetherVector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
            AetherVector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);

            // Get the pulley axes.
            AetherVector2 uA = cA + rA - WorldAnchorA;
            AetherVector2 uB = cB + rB - WorldAnchorB;

            Fix64 lengthA = uA.Length();
            Fix64 lengthB = uB.Length();

            if (lengthA > Fix64Constants.Ten * Settings.LinearSlop)
            {
                uA *= Fix64.One / lengthA;
            }
            else
            {
                uA = AetherVector2.Zero;
            }

            if (lengthB > Fix64Constants.Ten * Settings.LinearSlop)
            {
                uB *= Fix64.One / lengthB;
            }
            else
            {
                uB = AetherVector2.Zero;
            }

            // Compute effective mass.
            Fix64 ruA = MathUtils.Cross(ref rA, ref uA);
            Fix64 ruB = MathUtils.Cross(ref rB, ref uB);

            Fix64 mA = _invMassA + _invIA * ruA * ruA;
            Fix64 mB = _invMassB + _invIB * ruB * ruB;

            Fix64 mass = mA + Ratio * Ratio * mB;

            if (mass > Fix64.Zero)
            {
                mass = Fix64.One / mass;
            }

            Fix64 C = Constant - lengthA - Ratio * lengthB;
            Fix64 linearError =  Fix64.Abs(C);

            Fix64 impulse = -mass * C;

            AetherVector2 PA = -impulse * uA;
            AetherVector2 PB = -Ratio * impulse * uB;

            cA += _invMassA * PA;
            aA += _invIA * MathUtils.Cross(ref rA, ref PA);
            cB += _invMassB * PB;
            aB += _invIB * MathUtils.Cross(ref rB, ref PB);

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return linearError < Settings.LinearSlop;
        }
    }
}