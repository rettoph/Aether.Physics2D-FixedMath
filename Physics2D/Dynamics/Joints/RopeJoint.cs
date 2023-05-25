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
using tainicom.Aether.Physics2D.Common;
#if XNAAPI
using Complex = tainicom.Aether.Physics2D.Common.Complex;
using Vector2 = Microsoft.Xna.Framework.Vector2;
using Vector3 = Microsoft.Xna.Framework.Vector3;
#endif

namespace tainicom.Aether.Physics2D.Dynamics.Joints
{
    // Limit:
    // C = norm(pB - pA) - L
    // u = (pB - pA) / norm(pB - pA)
    // Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
    // J = [-u -cross(rA, u) u cross(rB, u)]
    // K = J * invM * JT
    //   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

    /// <summary>
    /// A rope joint enforces a maximum distance between two points on two bodies. It has no other effect.
    /// It can be used on ropes that are made up of several connected bodies, and if there is a need to support a heavy body.
    /// This joint is used for stabiliation of heavy objects on soft constraint joints.
    /// 
    /// Warning: if you attempt to change the maximum length during the simulation you will get some non-physical behavior.
    /// Use the DistanceJoint instead if you want to dynamically control the length.
    /// </summary>
    public class RopeJoint : Joint
    {
        // Solver shared
        private Fix64 _impulse;
        private Fix64 _length;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private AetherVector2 _localCenterA;
        private AetherVector2 _localCenterB;
        private Fix64 _invMassA;
        private Fix64 _invMassB;
        private Fix64 _invIA;
        private Fix64 _invIB;
        private Fix64 _mass;
        private AetherVector2 _rA, _rB;
        private AetherVector2 _u;

        internal RopeJoint()
        {
            JointType = JointType.Rope;
        }

        /// <summary>
        /// Constructor for RopeJoint.
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="anchorA">The anchor on the first body</param>
        /// <param name="anchorB">The anchor on the second body</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public RopeJoint(Body bodyA, Body bodyB, AetherVector2 anchorA, AetherVector2 anchorB, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Rope;

            if (useWorldCoordinates)
            {
                LocalAnchorA = bodyA.GetLocalPoint(anchorA);
                LocalAnchorB = bodyB.GetLocalPoint(anchorB);
            }
            else
            {
                LocalAnchorA = anchorA;
                LocalAnchorB = anchorB;
            }

            //FPE feature: Setting default MaxLength
            AetherVector2 d = WorldAnchorB - WorldAnchorA;
            MaxLength = d.Length();
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public AetherVector2 LocalAnchorA { get; set; }

        /// <summary>
        /// The local anchor point on BodyB
        /// </summary>
        public AetherVector2 LocalAnchorB { get; set; }

        public override sealed AetherVector2 WorldAnchorA
        {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
            set { LocalAnchorA = BodyA.GetLocalPoint(value); }
        }

        public override sealed AetherVector2 WorldAnchorB
        {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { LocalAnchorB = BodyB.GetLocalPoint(value); }
        }

        /// <summary>
        /// Get or set the maximum length of the rope.
        /// By default, it is the distance between the two anchor points.
        /// </summary>
        public Fix64 MaxLength { get; set; }

        /// <summary>
        /// Gets the state of the joint.
        /// </summary>
        public LimitState State { get; private set; }

        public override AetherVector2 GetReactionForce(Fix64 invDt)
        {
            return (invDt * _impulse) * _u;
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
            _u = cB + _rB - cA - _rA;

            _length = _u.Length();

            Fix64 C = _length - MaxLength;
            if (C > Fix64.Zero)
            {
                State = LimitState.AtUpper;
            }
            else
            {
                State = LimitState.Inactive;
            }

            if (_length > Settings.LinearSlop)
            {
                _u *= Fix64.One / _length;
            }
            else
            {
                _u = AetherVector2.Zero;
                _mass = Fix64.Zero;
                _impulse = Fix64.Zero;
                return;
            }

            // Compute effective mass.
            Fix64 crA = MathUtils.Cross(ref _rA, ref _u);
            Fix64 crB = MathUtils.Cross(ref _rB, ref _u);
            Fix64 invMass = _invMassA + _invIA * crA * crA + _invMassB + _invIB * crB * crB;

            _mass = invMass != Fix64.Zero ? Fix64.One / invMass : Fix64.Zero;

            if (data.step.warmStarting)
            {
                // Scale the impulse to support a variable time step.
                _impulse *= data.step.dtRatio;

                AetherVector2 P = _impulse * _u;
                vA -= _invMassA * P;
                wA -= _invIA * MathUtils.Cross(ref _rA, ref P);
                vB += _invMassB * P;
                wB += _invIB * MathUtils.Cross(ref _rB, ref P);
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

            // Cdot = dot(u, v + cross(w, r))
            AetherVector2 vpA = vA + MathUtils.Cross(wA, ref _rA);
            AetherVector2 vpB = vB + MathUtils.Cross(wB, ref _rB);
            Fix64 C = _length - MaxLength;
            Fix64 Cdot = AetherVector2.Dot(_u, vpB - vpA);

            // Predictive constraint.
            if (C < Fix64.Zero)
            {
                Cdot += data.step.inv_dt * C;
            }

            Fix64 impulse = -_mass * Cdot;
            Fix64 oldImpulse = _impulse;
            _impulse = MathUtils.Min(Fix64.Zero, _impulse + impulse);
            impulse = _impulse - oldImpulse;

            AetherVector2 P = impulse * _u;
            vA -= _invMassA * P;
            wA -= _invIA * MathUtils.Cross(ref _rA, ref P);
            vB += _invMassB * P;
            wB += _invIB * MathUtils.Cross(ref _rB, ref P);

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
            AetherVector2 u = cB + rB - cA - rA;

            Fix64 length = u.Length(); u.Normalize();
            Fix64 C = length - MaxLength;

            C = MathUtils.Clamp(C, Fix64.Zero, Settings.MaxLinearCorrection);

            Fix64 impulse = -_mass * C;
            AetherVector2 P = impulse * u;

            cA -= _invMassA * P;
            aA -= _invIA * MathUtils.Cross(ref rA, ref P);
            cB += _invMassB * P;
            aB += _invIB * MathUtils.Cross(ref rB, ref P);

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return length - MaxLength < Settings.LinearSlop;
        }
    }
}