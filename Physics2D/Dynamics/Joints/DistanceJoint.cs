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
    // 1-D rained system
    // m (v2 - v1) = lambda
    // v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
    // x2 = x1 + h * v2

    // 1-D mass-damper-spring system
    // m (v2 - v1) + h * d * v2 + h * k * 

    // C = norm(p2 - p1) - L
    // u = (p2 - p1) / norm(p2 - p1)
    // Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    // J = [-u -cross(r1, u) u cross(r2, u)]
    // K = J * invM * JT
    //   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

    /// <summary>
    /// A distance joint rains two points on two bodies
    /// to remain at a fixed distance from each other. You can view
    /// this as a massless, rigid rod.
    /// </summary>
    public class DistanceJoint : Joint
    {
        // Solver shared
        private Fix64 _bias;
        private Fix64 _gamma;
        private Fix64 _impulse;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private AetherVector2 _u;
        private AetherVector2 _rA;
        private AetherVector2 _rB;
        private AetherVector2 _localCenterA;
        private AetherVector2 _localCenterB;
        private Fix64 _invMassA;
        private Fix64 _invMassB;
        private Fix64 _invIA;
        private Fix64 _invIB;
        private Fix64 _mass;

        internal DistanceJoint()
        {
            JointType = JointType.Distance;
        }

        /// <summary>
        /// This requires defining an
        /// anchor point on both bodies and the non-zero length of the
        /// distance joint. If you don't supply a length, the local anchor points
        /// is used so that the initial configuration can violate the constraint
        /// slightly. This helps when saving and loading a game.
        /// Warning Do not use a zero or short length.
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="anchorA">The first body anchor</param>
        /// <param name="anchorB">The second body anchor</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public DistanceJoint(Body bodyA, Body bodyB, AetherVector2 anchorA, AetherVector2 anchorB, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Distance;

            if (useWorldCoordinates)
            {
                LocalAnchorA = bodyA.GetLocalPoint(ref anchorA);
                LocalAnchorB = bodyB.GetLocalPoint(ref anchorB);
                Length = (anchorB - anchorA).Length();
            }
            else
            {
                LocalAnchorA = anchorA;
                LocalAnchorB = anchorB;
                Length = (BodyB.GetWorldPoint(ref anchorB) - BodyA.GetWorldPoint(ref anchorA)).Length();
            }
        }

        /// <summary>
        /// The local anchor point relative to bodyA's origin.
        /// </summary>
        public AetherVector2 LocalAnchorA { get; set; }

        /// <summary>
        /// The local anchor point relative to bodyB's origin.
        /// </summary>
        public AetherVector2 LocalAnchorB { get; set; }

        public override sealed AetherVector2 WorldAnchorA
        {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        public override sealed AetherVector2 WorldAnchorB
        {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { Debug.Assert(false, "You can't set the world anchor on this joint type."); }
        }

        /// <summary>
        /// The natural length between the anchor points.
        /// Manipulating the length can lead to non-physical behavior when the frequency is zero.
        /// </summary>
        public Fix64 Length { get; set; }

        /// <summary>
        /// The mass-spring-damper frequency in Hertz. A value of 0
        /// disables softness.
        /// </summary>
        public Fix64 Frequency { get; set; }

        /// <summary>
        /// The damping ratio. 0 = no damping, 1 = critical damping.
        /// </summary>
        public Fix64 DampingRatio { get; set; }

        /// <summary>
        /// Get the reaction force given the inverse time step. Unit is N.
        /// </summary>
        /// <param name="invDt"></param>
        /// <returns></returns>
        public override AetherVector2 GetReactionForce(Fix64 invDt)
        {
            AetherVector2 F = (invDt * _impulse) * _u;
            return F;
        }

        /// <summary>
        /// Get the reaction torque given the inverse time step.
        /// Unit is N*m. This is always zero for a distance joint.
        /// </summary>
        /// <param name="invDt"></param>
        /// <returns></returns>
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

            // Handle singularity.
            Fix64 length = _u.Length();
            if (length > Settings.LinearSlop)
            {
                _u *= Fix64.One / length;
            }
            else
            {
                _u = AetherVector2.Zero;
            }

            Fix64 crAu = MathUtils.Cross(ref _rA, ref _u);
            Fix64 crBu = MathUtils.Cross(ref _rB, ref _u);
            Fix64 invMass = _invMassA + _invIA * crAu * crAu + _invMassB + _invIB * crBu * crBu;

            // Compute the effective mass matrix.
            _mass = invMass != Fix64.Zero ? Fix64.One / invMass : Fix64.Zero;

            if (Frequency > Fix64.Zero)
            {
                Fix64 C = length - Length;

                // Frequency
                Fix64 omega = Constant.Tau * Frequency;

                // Damping coefficient
                Fix64 d = Fix64Constants.Two * _mass * DampingRatio * omega;

                // Spring stiffness
                Fix64 k = _mass * omega * omega;

                // magic formulas
                Fix64 h = data.step.dt;
                _gamma = h * (d + h * k);
                _gamma = _gamma != Fix64.Zero ? Fix64.One / _gamma : Fix64.Zero;
                _bias = C * h * k * _gamma;

                invMass += _gamma;
                _mass = invMass != Fix64.Zero ? Fix64.One / invMass : Fix64.Zero;
            }
            else
            {
                _gamma = Fix64.Zero;
                _bias = Fix64.Zero;
            }

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
            Fix64 Cdot = AetherVector2.Dot(_u, vpB - vpA);

            Fix64 impulse = -_mass * (Cdot + _bias + _gamma * _impulse);
            _impulse += impulse;

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
            if (Frequency > Fix64.Zero)
            {
                // There is no position correction for soft distance constraints.
                return true;
            }

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
            Fix64 C = length - Length;
            C = MathUtils.Clamp(C, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);

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

            return  Fix64.Abs(C) < Settings.LinearSlop;
        }
    }
}