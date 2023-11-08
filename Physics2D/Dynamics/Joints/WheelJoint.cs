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
    // Linear constraint (point-to-line)
    // d = pB - pA = xB + rB - xA - rA
    // C = dot(ay, d)
    // Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
    //      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
    // J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

    // Spring linear constraint
    // C = dot(ax, d)
    // Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
    // J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

    // Motor rotational constraint
    // Cdot = wB - wA
    // J = [0 0 -1 0 0 1]

    /// <summary>
    /// A wheel joint. This joint provides two degrees of freedom: translation
    /// along an axis fixed in bodyA and rotation in the plane. You can use a
    /// joint limit to restrict the range of motion and a joint motor to drive
    /// the rotation or to model rotational friction.
    /// This joint is designed for vehicle suspensions.
    /// </summary>
    public class WheelJoint : Joint
    {
        // Solver shared
        private AetherVector2 _localXAxis;
        private AetherVector2 _localYAxis;

        private Fix64 _impulse;
        private Fix64 _motorImpulse;
        private Fix64 _springImpulse;

        private Fix64 _maxMotorTorque;
        private Fix64 _motorSpeed;
        private bool _enableMotor;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private AetherVector2 _localCenterA;
        private AetherVector2 _localCenterB;
        private Fix64 _invMassA;
        private Fix64 _invMassB;
        private Fix64 _invIA;
        private Fix64 _invIB;

        private AetherVector2 _ax, _ay;
        private Fix64 _sAx, _sBx;
        private Fix64 _sAy, _sBy;

        private Fix64 _mass;
        private Fix64 _motorMass;
        private Fix64 _springMass;

        private Fix64 _bias;
        private Fix64 _gamma;
        private AetherVector2 _axis;

        internal WheelJoint()
        {
            JointType = JointType.Wheel;
        }

        /// <summary>
        /// Constructor for WheelJoint
        /// </summary>
        /// <param name="bodyA">The first body</param>
        /// <param name="bodyB">The second body</param>
        /// <param name="anchor">The anchor point</param>
        /// <param name="axis">The axis</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public WheelJoint(Body bodyA, Body bodyB, AetherVector2 anchor, AetherVector2 axis, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Wheel;

            if (useWorldCoordinates)
            {
                LocalAnchorA = bodyA.GetLocalPoint(anchor);
                LocalAnchorB = bodyB.GetLocalPoint(anchor);
            }
            else
            {
                LocalAnchorA = bodyA.GetLocalPoint(bodyB.GetWorldPoint(anchor));
                LocalAnchorB = anchor;
            }

            Axis = axis; //FPE only: We maintain the original value as it is supposed to.
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public AetherVector2 LocalAnchorA { get; set; }

        /// <summary>
        /// The local anchor point on BodyB
        /// </summary>
        public AetherVector2 LocalAnchorB { get; set; }

        public override AetherVector2 WorldAnchorA
        {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
            set { LocalAnchorA = BodyA.GetLocalPoint(value); }
        }

        public override AetherVector2 WorldAnchorB
        {
            get { return BodyB.GetWorldPoint(LocalAnchorB); }
            set { LocalAnchorB = BodyB.GetLocalPoint(value); }
        }

        /// <summary>
        /// The axis at which the suspension moves.
        /// </summary>
        public AetherVector2 Axis
        {
            get { return _axis; }
            set
            {
                _axis = value;
                _localXAxis = BodyA.GetLocalVector(_axis);
                _localYAxis = MathUtils.Rot90(ref _localXAxis);
            }
        }

        /// <summary>
        /// The axis in local coordinates relative to BodyA
        /// </summary>
        public AetherVector2 LocalXAxis { get { return _localXAxis; } }

        /// <summary>
        /// The desired motor speed in radians per second.
        /// </summary>
        public Fix64 MotorSpeed
        {
            get { return _motorSpeed; }
            set
            {
                WakeBodies();
                _motorSpeed = value;
            }
        }

        /// <summary>
        /// The maximum motor torque, usually in N-m.
        /// </summary>
        public Fix64 MaxMotorTorque
        {
            get { return _maxMotorTorque; }
            set
            {
                WakeBodies();
                _maxMotorTorque = value;
            }
        }

        /// <summary>
        /// Suspension frequency, zero indicates no suspension
        /// </summary>
        public Fix64 Frequency { get; set; }

        /// <summary>
        /// Suspension damping ratio, one indicates critical damping
        /// </summary>
        public Fix64 DampingRatio { get; set; }

        /// <summary>
        /// Gets the translation along the axis
        /// </summary>
        public Fix64 JointTranslation
        {
            get
            {
                Body bA = BodyA;
                Body bB = BodyB;

                AetherVector2 pA = bA.GetWorldPoint(LocalAnchorA);
                AetherVector2 pB = bB.GetWorldPoint(LocalAnchorB);
                AetherVector2 d = pB - pA;
                AetherVector2 axis = bA.GetWorldVector(ref _localXAxis);

                Fix64 translation = AetherVector2.Dot(d, axis);
                return translation;
            }
        }

        /// <summary>
        /// Gets the angular velocity of the joint
        /// </summary>
        public Fix64 JointSpeed
        {
            get
            {
                Fix64 wA = BodyA.AngularVelocity;
                Fix64 wB = BodyB.AngularVelocity;
                return wB - wA;
            }
        }

        /// <summary>
        /// Enable/disable the joint motor.
        /// </summary>
        public bool MotorEnabled
        {
            get { return _enableMotor; }
            set
            {
                WakeBodies();
                _enableMotor = value;
            }
        }

        /// <summary>
        /// Gets the torque of the motor
        /// </summary>
        /// <param name="invDt">inverse delta time</param>
        public Fix64 GetMotorTorque(Fix64 invDt)
        {
            return invDt * _motorImpulse;
        }

        public override AetherVector2 GetReactionForce(Fix64 invDt)
        {
            return invDt * (_impulse * _ay + _springImpulse * _ax);
        }

        public override Fix64 GetReactionTorque(Fix64 invDt)
        {
            return invDt * _motorImpulse;
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

            Fix64 mA = _invMassA, mB = _invMassB;
            Fix64 iA = _invIA, iB = _invIB;

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

            // Compute the effective masses.
            AetherVector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
            AetherVector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);
            AetherVector2 d1 = cB + rB - cA - rA;

            // Point to line constraint
            {
                _ay = Complex.Multiply(ref _localYAxis, ref qA);
                _sAy = MathUtils.Cross(d1 + rA, _ay);
                _sBy = MathUtils.Cross(ref rB, ref _ay);

                _mass = mA + mB + iA * _sAy * _sAy + iB * _sBy * _sBy;

                if (_mass > Fix64.Zero)
                {
                    _mass = Fix64.One / _mass;
                }
            }

            // Spring constraint
            _springMass = Fix64.Zero;
            _bias = Fix64.Zero;
            _gamma = Fix64.Zero;
            if (Frequency > Fix64.Zero)
            {
                _ax = Complex.Multiply(ref _localXAxis, ref qA);
                _sAx = MathUtils.Cross(d1 + rA, _ax);
                _sBx = MathUtils.Cross(ref rB, ref _ax);

                Fix64 invMass = mA + mB + iA * _sAx * _sAx + iB * _sBx * _sBx;

                if (invMass > Fix64.Zero)
                {
                    _springMass = Fix64.One / invMass;

                    Fix64 C = AetherVector2.Dot(d1, _ax);

                    // Frequency
                    Fix64 omega = Constant.Tau * Frequency;

                    // Damping coefficient
                    Fix64 d = Fix64Constants.Two * _springMass * DampingRatio * omega;

                    // Spring stiffness
                    Fix64 k = _springMass * omega * omega;

                    // magic formulas
                    Fix64 h = data.step.dt;
                    _gamma = h * (d + h * k);
                    if (_gamma > Fix64.Zero)
                    {
                        _gamma = Fix64.One / _gamma;
                    }

                    _bias = C * h * k * _gamma;

                    _springMass = invMass + _gamma;
                    if (_springMass > Fix64.Zero)
                    {
                        _springMass = Fix64.One / _springMass;
                    }
                }
            }
            else
            {
                _springImpulse = Fix64.Zero;
            }

            // Rotational motor
            if (_enableMotor)
            {
                _motorMass = iA + iB;
                if (_motorMass > Fix64.Zero)
                {
                    _motorMass = Fix64.One / _motorMass;
                }
            }
            else
            {
                _motorMass = Fix64.Zero;
                _motorImpulse = Fix64.Zero;
            }

            if (data.step.warmStarting)
            {
                // Account for variable time step.
                _impulse *= data.step.dtRatio;
                _springImpulse *= data.step.dtRatio;
                _motorImpulse *= data.step.dtRatio;

                AetherVector2 P = _impulse * _ay + _springImpulse * _ax;
                Fix64 LA = _impulse * _sAy + _springImpulse * _sAx + _motorImpulse;
                Fix64 LB = _impulse * _sBy + _springImpulse * _sBx + _motorImpulse;

                vA -= _invMassA * P;
                wA -= _invIA * LA;

                vB += _invMassB * P;
                wB += _invIB * LB;
            }
            else
            {
                _impulse = Fix64.Zero;
                _springImpulse = Fix64.Zero;
                _motorImpulse = Fix64.Zero;
            }

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
            data.velocities[_indexB].v = vB;
            data.velocities[_indexB].w = wB;
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            Fix64 mA = _invMassA, mB = _invMassB;
            Fix64 iA = _invIA, iB = _invIB;

            AetherVector2 vA = data.velocities[_indexA].v;
            Fix64 wA = data.velocities[_indexA].w;
            AetherVector2 vB = data.velocities[_indexB].v;
            Fix64 wB = data.velocities[_indexB].w;

            // Solve spring constraint
            {
                Fix64 Cdot = AetherVector2.Dot(_ax, vB - vA) + _sBx * wB - _sAx * wA;
                Fix64 impulse = -_springMass * (Cdot + _bias + _gamma * _springImpulse);
                _springImpulse += impulse;

                AetherVector2 P = impulse * _ax;
                Fix64 LA = impulse * _sAx;
                Fix64 LB = impulse * _sBx;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            // Solve rotational motor constraint
            {
                Fix64 Cdot = wB - wA - _motorSpeed;
                Fix64 impulse = -_motorMass * Cdot;

                Fix64 oldImpulse = _motorImpulse;
                Fix64 maxImpulse = data.step.dt * _maxMotorTorque;
                _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _motorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve point to line constraint
            {
                Fix64 Cdot = AetherVector2.Dot(_ay, vB - vA) + _sBy * wB - _sAy * wA;
                Fix64 impulse = -_mass * Cdot;
                _impulse += impulse;

                AetherVector2 P = impulse * _ay;
                Fix64 LA = impulse * _sAy;
                Fix64 LB = impulse * _sBy;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

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
            AetherVector2 d = (cB - cA) + rB - rA;

            AetherVector2 ay = Complex.Multiply(ref _localYAxis, ref qA);

            Fix64 sAy = MathUtils.Cross(d + rA, ay);
            Fix64 sBy = MathUtils.Cross(ref rB, ref ay);

            Fix64 C = AetherVector2.Dot(d, ay);

            Fix64 k = _invMassA + _invMassB + _invIA * _sAy * _sAy + _invIB * _sBy * _sBy;

            Fix64 impulse;
            if (k != Fix64.Zero)
            {
                impulse = -C / k;
            }
            else
            {
                impulse = Fix64.Zero;
            }

            AetherVector2 P = impulse * ay;
            Fix64 LA = impulse * sAy;
            Fix64 LB = impulse * sBy;

            cA -= _invMassA * P;
            aA -= _invIA * LA;
            cB += _invMassB * P;
            aB += _invIB * LB;

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return  Fix64.Abs(C) <= Settings.LinearSlop;
        }
    }
}