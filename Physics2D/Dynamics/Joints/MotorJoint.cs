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
using System.Diagnostics;
using tainicom.Aether.Physics2D.Common;
#if XNAAPI
using Complex = tainicom.Aether.Physics2D.Common.Complex;
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Dynamics.Joints
{
    /// <summary>
    /// A motor joint is used to control the relative motion
    /// between two bodies. A typical usage is to control the movement
    /// of a dynamic body with respect to the ground.
    /// </summary>
    public class MotorJoint : Joint
    {
        // Solver shared
        private AetherVector2 _linearOffset;
        private Fix64 _angularOffset;
        private AetherVector2 _linearImpulse;
        private Fix64 _angularImpulse;
        private Fix64 _maxForce;
        private Fix64 _maxTorque;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private AetherVector2 _rA;
        private AetherVector2 _rB;
        private AetherVector2 _localCenterA;
        private AetherVector2 _localCenterB;
        private AetherVector2 _linearError;
        private Fix64 _angularError;
        private Fix64 _invMassA;
        private Fix64 _invMassB;
        private Fix64 _invIA;
        private Fix64 _invIB;
        private Mat22 _linearMass;
        private Fix64 _angularMass;

        internal MotorJoint()
        {
            JointType = JointType.Motor;
        }

        /// <summary>
        /// Constructor for MotorJoint.
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public MotorJoint(Body bodyA, Body bodyB, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Motor;

            AetherVector2 xB = BodyB.Position;

            if (useWorldCoordinates)
                _linearOffset = BodyA.GetLocalPoint(xB);
            else
                _linearOffset = xB;

            //Defaults
            _angularOffset = Fix64.Zero;
            _maxForce = Fix64.One;
            _maxTorque = Fix64.One;
            CorrectionFactor = Fix64Constants.PointThree;

            _angularOffset = BodyB.Rotation - BodyA.Rotation;
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
        /// The maximum amount of force that can be applied to BodyA
        /// </summary>
        public Fix64 MaxForce
        {
            set
            {
                Debug.Assert(value >= Fix64.Zero);
                _maxForce = value;
            }
            get { return _maxForce; }
        }

        /// <summary>
        /// The maximum amount of torque that can be applied to BodyA
        /// </summary>
        public Fix64 MaxTorque
        {
            set
            {
                Debug.Assert(value >= Fix64.Zero);
                _maxTorque = value;
            }
            get { return _maxTorque; }
        }

        /// <summary>
        /// The linear (translation) offset.
        /// </summary>
        public AetherVector2 LinearOffset
        {
            set
            {
                if (_linearOffset.X != value.X || _linearOffset.Y != value.Y)
                {
                    WakeBodies();
                    _linearOffset = value;
                }
            }
            get { return _linearOffset; }
        }

        /// <summary>
        /// Get or set the angular offset.
        /// </summary>
        public Fix64 AngularOffset
        {
            set
            {
                if (_angularOffset != value)
                {
                    WakeBodies();
                    _angularOffset = value;
                }
            }
            get { return _angularOffset; }
        }

        //FPE note: Used for serialization.
        internal Fix64 CorrectionFactor { get; set; }

        public override AetherVector2 GetReactionForce(Fix64 invDt)
        {
            return invDt * _linearImpulse;
        }

        public override Fix64 GetReactionTorque(Fix64 invDt)
        {
            return invDt * _angularImpulse;
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

            // Compute the effective mass matrix.
            _rA = -Complex.Multiply(ref _localCenterA, ref qA);
            _rB = -Complex.Multiply(ref _localCenterB, ref qB);

            // J = [-I -r1_skew I r2_skew]
            //     [ 0       -1 0       1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
            //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
            //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

            Fix64 mA = _invMassA, mB = _invMassB;
            Fix64 iA = _invIA, iB = _invIB;

            Mat22 K = new Mat22();
            K.ex.X = mA + mB + iA * _rA.Y * _rA.Y + iB * _rB.Y * _rB.Y;
            K.ex.Y = -iA * _rA.X * _rA.Y - iB * _rB.X * _rB.Y;
            K.ey.X = K.ex.Y;
            K.ey.Y = mA + mB + iA * _rA.X * _rA.X + iB * _rB.X * _rB.X;

            _linearMass = K.Inverse;

            _angularMass = iA + iB;
            if (_angularMass > Fix64.Zero)
            {
                _angularMass = Fix64.One / _angularMass;
            }

            _linearError = cB + _rB - cA - _rA - Complex.Multiply(ref _linearOffset, ref qA);
            _angularError = aB - aA - _angularOffset;

            if (data.step.warmStarting)
            {
                // Scale impulses to support a variable time step.
                _linearImpulse *= data.step.dtRatio;
                _angularImpulse *= data.step.dtRatio;

                AetherVector2 P = new AetherVector2(_linearImpulse.X, _linearImpulse.Y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(ref _rA, ref P) + _angularImpulse);
                vB += mB * P;
                wB += iB * (MathUtils.Cross(ref _rB, ref P) + _angularImpulse);
            }
            else
            {
                _linearImpulse = AetherVector2.Zero;
                _angularImpulse = Fix64.Zero;
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

            Fix64 mA = _invMassA, mB = _invMassB;
            Fix64 iA = _invIA, iB = _invIB;

            Fix64 h = data.step.dt;
            Fix64 inv_h = data.step.inv_dt;

            // Solve angular friction
            {
                Fix64 Cdot = wB - wA + inv_h * CorrectionFactor * _angularError;
                Fix64 impulse = -_angularMass * Cdot;

                Fix64 oldImpulse = _angularImpulse;
                Fix64 maxImpulse = h * _maxTorque;
                _angularImpulse = MathUtils.Clamp(_angularImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _angularImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve linear friction
            {
                AetherVector2 Cdot = vB + MathUtils.Cross(wB, ref _rB) - vA - MathUtils.Cross(wA, ref _rA) + inv_h * CorrectionFactor * _linearError;

                AetherVector2 impulse = -MathUtils.Mul(ref _linearMass, ref Cdot);
                AetherVector2 oldImpulse = _linearImpulse;
                _linearImpulse += impulse;

                Fix64 maxImpulse = h * _maxForce;

                if (_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
                {
                    _linearImpulse.Normalize();
                    _linearImpulse *= maxImpulse;
                }

                impulse = _linearImpulse - oldImpulse;

                vA -= mA * impulse;
                wA -= iA * MathUtils.Cross(ref _rA, ref impulse);

                vB += mB * impulse;
                wB += iB * MathUtils.Cross(ref _rB, ref impulse);
            }

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            return true;
        }
    }
}