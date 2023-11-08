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
using Vector3 = Microsoft.Xna.Framework.Vector3;
#endif

namespace tainicom.Aether.Physics2D.Dynamics.Joints
{
    // Linear constraint (point-to-line)
    // d = p2 - p1 = x2 + r2 - x1 - r1
    // C = dot(perp, d)
    // Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    //      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
    // J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
    //
    // Angular constraint
    // C = a2 - a1 + a_initial
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    //
    // K = J * invM * JT
    //
    // J = [-a -s1 a s2]
    //     [0  -1  0  1]
    // a = perp
    // s1 = cross(d + r1, a) = cross(p2 - x1, a)
    // s2 = cross(r2, a) = cross(p2 - x2, a)
    // Motor/Limit linear constraint
    // C = dot(ax1, d)
    // Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
    // J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]
    // Block Solver
    // We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
    // when the mass has poor distribution (leading to large torques about the joint anchor points).
    //
    // The Jacobian has 3 rows:
    // J = [-uT -s1 uT s2] // linear
    //     [0   -1   0  1] // angular
    //     [-vT -a1 vT a2] // limit
    //
    // u = perp
    // v = axis
    // s1 = cross(d + r1, u), s2 = cross(r2, u)
    // a1 = cross(d + r1, v), a2 = cross(r2, v)
    // M * (v2 - v1) = JT * df
    // J * v2 = bias
    //
    // v2 = v1 + invM * JT * df
    // J * (v1 + invM * JT * df) = bias
    // K * df = bias - J * v1 = -Cdot
    // K = J * invM * JT
    // Cdot = J * v1 - bias
    //
    // Now solve for f2.
    // df = f2 - f1
    // K * (f2 - f1) = -Cdot
    // f2 = invK * (-Cdot) + f1
    //
    // Clamp accumulated limit impulse.
    // lower: f2(3) = max(f2(3), Fix64.Zero)
    // upper: f2(3) = min(f2(3), Fix64.Zero)
    //
    // Solve for correct f2(1:2)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
    //                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
    // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
    //
    // Now compute impulse to be applied:
    // df = f2 - f1

    /// <summary>
    /// A prismatic joint. This joint provides one degree of freedom: translation
    /// along an axis fixed in bodyA. Relative rotation is prevented. You can
    /// use a joint limit to restrict the range of motion and a joint motor to
    /// drive the motion or to model joint friction.
    /// </summary>
    public class PrismaticJoint : Joint
    {
        private AetherVector2 _localXAxis;
        private AetherVector2 _localYAxisA;
        private AetherVector3 _impulse;
        private Fix64 _lowerTranslation;
        private Fix64 _upperTranslation;
        private Fix64 _maxMotorForce;
        private Fix64 _motorSpeed;
        private bool _enableLimit;
        private bool _enableMotor;
        private LimitState _limitState;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private AetherVector2 _localCenterA;
        private AetherVector2 _localCenterB;
        private Fix64 _invMassA;
        private Fix64 _invMassB;
        private Fix64 _invIA;
        private Fix64 _invIB;
        private AetherVector2 _axis, _perp;
        private Fix64 _s1, _s2;
        private Fix64 _a1, _a2;
        private Mat33 _K;
        private Fix64 _motorMass;
        private AetherVector2 _axis1;

        internal PrismaticJoint()
        {
            JointType = JointType.Prismatic;
        }

        /// <summary>
        /// This requires defining a line of
        /// motion using an axis and an anchor point. The definition uses local
        /// anchor points and a local axis so that the initial configuration
        /// can violate the constraint slightly. The joint translation is zero
        /// when the local anchor points coincide in world space. Using local
        /// anchors and a local axis helps when saving and loading a game.
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="anchorA">The first body anchor.</param>
        /// <param name="anchorB">The second body anchor.</param>
        /// <param name="axis">The axis.</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public PrismaticJoint(Body bodyA, Body bodyB, AetherVector2 anchorA, AetherVector2 anchorB, AetherVector2 axis, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            Initialize(anchorA, anchorB, axis, useWorldCoordinates);
        }

        public PrismaticJoint(Body bodyA, Body bodyB, AetherVector2 anchor, AetherVector2 axis, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            Initialize(anchor, anchor, axis, useWorldCoordinates);
        }

        private void Initialize(AetherVector2 localAnchorA, AetherVector2 localAnchorB, AetherVector2 axis, bool useWorldCoordinates)
        {
            JointType = JointType.Prismatic;

            if (useWorldCoordinates)
            {
                LocalAnchorA = BodyA.GetLocalPoint(localAnchorA);
                LocalAnchorB = BodyB.GetLocalPoint(localAnchorB);
            }
            else
            {
                LocalAnchorA = localAnchorA;
                LocalAnchorB = localAnchorB;
            }

            Axis = axis; //FPE only: store the orignal value for use in Serialization
            ReferenceAngle = BodyB.Rotation - BodyA.Rotation;

            _limitState = LimitState.Inactive;
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
        /// Get the current joint translation, usually in meters.
        /// </summary>
        /// <value></value>
        public Fix64 JointTranslation
        {
            get
            {
                AetherVector2 d = BodyB.GetWorldPoint(LocalAnchorB) - BodyA.GetWorldPoint(LocalAnchorA);
                AetherVector2 axis = BodyA.GetWorldVector(ref _localXAxis);

                return AetherVector2.Dot(d, axis);
            }
        }

        /// <summary>
        /// Get the current joint translation speed, usually in meters per second.
        /// </summary>
        /// <value></value>
        public Fix64 JointSpeed
        {
            get
            {
                Transform xf1 = BodyA.GetTransform();
                Transform xf2 = BodyB.GetTransform();

                AetherVector2 r1 = Complex.Multiply(LocalAnchorA - BodyA.LocalCenter, ref xf1.q);
                AetherVector2 r2 = Complex.Multiply(LocalAnchorB - BodyB.LocalCenter, ref xf2.q);
                AetherVector2 p1 = BodyA._sweep.C + r1;
                AetherVector2 p2 = BodyB._sweep.C + r2;
                AetherVector2 d = p2 - p1;
                AetherVector2 axis = BodyA.GetWorldVector(ref _localXAxis);

                AetherVector2 v1 = BodyA._linearVelocity;
                AetherVector2 v2 = BodyB._linearVelocity;
                Fix64 w1 = BodyA._angularVelocity;
                Fix64 w2 = BodyB._angularVelocity;

                Fix64 speed = AetherVector2.Dot(d, MathUtils.Cross(w1, ref axis)) + AetherVector2.Dot(axis, v2 + MathUtils.Cross(w2, ref r2) - v1 - MathUtils.Cross(w1, ref r1));
                return speed;
            }
        }

        /// <summary>
        /// Is the joint limit enabled?
        /// </summary>
        /// <value><c>true</c> if [limit enabled]; otherwise, <c>false</c>.</value>
        public bool LimitEnabled
        {
            get { return _enableLimit; }
            set
            {
                Debug.Assert(BodyA.FixedRotation == false || BodyB.FixedRotation == false, "Warning: limits does currently not work with fixed rotation");

                if (value != _enableLimit)
                {
                    WakeBodies();
                    _enableLimit = value;
                    _impulse.Z = Fix64.Zero;
                }
            }
        }

        /// <summary>
        /// Get the lower joint limit, usually in meters.
        /// </summary>
        /// <value></value>
        public Fix64 LowerLimit
        {
            get { return _lowerTranslation; }
            set
            {
                if (value != _lowerTranslation)
                {
                    WakeBodies();
                    _lowerTranslation = value;
                    _impulse.Z = Fix64.Zero;
                }
            }
        }

        /// <summary>
        /// Get the upper joint limit, usually in meters.
        /// </summary>
        /// <value></value>
        public Fix64 UpperLimit
        {
            get { return _upperTranslation; }
            set
            {
                if (value != _upperTranslation)
                {
                    WakeBodies();
                    _upperTranslation = value;
                    _impulse.Z = Fix64.Zero;
                }
            }
        }

        /// <summary>
        /// Set the joint limits, usually in meters.
        /// </summary>
        /// <param name="lower">The lower limit</param>
        /// <param name="upper">The upper limit</param>
        public void SetLimits(Fix64 lower, Fix64 upper)
        {
            if (upper != _upperTranslation || lower != _lowerTranslation)
            {
                WakeBodies();
                _upperTranslation = upper;
                _lowerTranslation = lower;
                _impulse.Z = Fix64.Zero;
            }
        }

        /// <summary>
        /// Is the joint motor enabled?
        /// </summary>
        /// <value><c>true</c> if [motor enabled]; otherwise, <c>false</c>.</value>
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
        /// Set the motor speed, usually in meters per second.
        /// </summary>
        /// <value>The speed.</value>
        public Fix64 MotorSpeed
        {
            set
            {
                WakeBodies();
                _motorSpeed = value;
            }
            get { return _motorSpeed; }
        }

        /// <summary>
        /// Set the maximum motor force, usually in N.
        /// </summary>
        /// <value>The force.</value>
        public Fix64 MaxMotorForce
        {
            get { return _maxMotorForce; }
            set
            {
                WakeBodies();
                _maxMotorForce = value;
            }
        }

        /// <summary>
        /// Get the current motor impulse, usually in N.
        /// </summary>
        /// <value></value>
        public Fix64 MotorImpulse { get; set; }

        /// <summary>
        /// Gets the motor force.
        /// </summary>
        /// <param name="invDt">The inverse delta time</param>
        public Fix64 GetMotorForce(Fix64 invDt)
        {
            return invDt * MotorImpulse;
        }

        /// <summary>
        /// The axis at which the joint moves.
        /// </summary>
        public AetherVector2 Axis
        {
            get { return _axis1; }
            set
            {
                _axis1 = value;
                _localXAxis = BodyA.GetLocalVector(_axis1);
                _localXAxis.Normalize();
                _localYAxisA = MathUtils.Cross(Fix64.One, ref _localXAxis);
            }
        }

        /// <summary>
        /// The axis in local coordinates relative to BodyA
        /// </summary>
        public AetherVector2 LocalXAxis { get { return _localXAxis; } }

        /// <summary>
        /// The reference angle.
        /// </summary>
        public Fix64 ReferenceAngle { get; set; }

        public override AetherVector2 GetReactionForce(Fix64 invDt)
        {
            return invDt * (_impulse.X * _perp + (MotorImpulse + _impulse.Z) * _axis);
        }

        public override Fix64 GetReactionTorque(Fix64 invDt)
        {
            return invDt * _impulse.Y;
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

            // Compute the effective masses.
            AetherVector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
            AetherVector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);
            AetherVector2 d = (cB - cA) + rB - rA;

            Fix64 mA = _invMassA, mB = _invMassB;
            Fix64 iA = _invIA, iB = _invIB;

            // Compute motor Jacobian and effective mass.
            {
                _axis = Complex.Multiply(ref _localXAxis, ref qA);
                _a1 = MathUtils.Cross(d + rA, _axis);
                _a2 = MathUtils.Cross(ref rB, ref _axis);

                _motorMass = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
                if (_motorMass > Fix64.Zero)
                {
                    _motorMass = Fix64.One / _motorMass;
                }
            }

            // Prismatic constraint.
            {
                _perp = Complex.Multiply(ref _localYAxisA, ref qA);

                _s1 = MathUtils.Cross(d + rA, _perp);
                _s2 = MathUtils.Cross(ref rB, ref _perp);

                Fix64 k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
                Fix64 k12 = iA * _s1 + iB * _s2;
                Fix64 k13 = iA * _s1 * _a1 + iB * _s2 * _a2;
                Fix64 k22 = iA + iB;
                if (k22 == Fix64.Zero)
                {
                    // For bodies with fixed rotation.
                    k22 = Fix64.One;
                }
                Fix64 k23 = iA * _a1 + iB * _a2;
                Fix64 k33 = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;

                _K.ex = new AetherVector3(k11, k12, k13);
                _K.ey = new AetherVector3(k12, k22, k23);
                _K.ez = new AetherVector3(k13, k23, k33);
            }

            // Compute motor and limit terms.
            if (_enableLimit)
            {
                Fix64 jointTranslation = AetherVector2.Dot(_axis, d);
                if ( Fix64.Abs(_upperTranslation - _lowerTranslation) < Fix64Constants.Two * Settings.LinearSlop)
                {
                    _limitState = LimitState.Equal;
                }
                else if (jointTranslation <= _lowerTranslation)
                {
                    if (_limitState != LimitState.AtLower)
                    {
                        _limitState = LimitState.AtLower;
                        _impulse.Z = Fix64.Zero;
                    }
                }
                else if (jointTranslation >= _upperTranslation)
                {
                    if (_limitState != LimitState.AtUpper)
                    {
                        _limitState = LimitState.AtUpper;
                        _impulse.Z = Fix64.Zero;
                    }
                }
                else
                {
                    _limitState = LimitState.Inactive;
                    _impulse.Z = Fix64.Zero;
                }
            }
            else
            {
                _limitState = LimitState.Inactive;
                _impulse.Z = Fix64.Zero;
            }

            if (_enableMotor == false)
            {
                MotorImpulse = Fix64.Zero;
            }

            if (data.step.warmStarting)
            {
                // Account for variable time step.
                _impulse *= data.step.dtRatio;
                MotorImpulse *= data.step.dtRatio;

                AetherVector2 P = _impulse.X * _perp + (MotorImpulse + _impulse.Z) * _axis;
                Fix64 LA = _impulse.X * _s1 + _impulse.Y + (MotorImpulse + _impulse.Z) * _a1;
                Fix64 LB = _impulse.X * _s2 + _impulse.Y + (MotorImpulse + _impulse.Z) * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else
            {
                _impulse = AetherVector3.Zero;
                MotorImpulse = Fix64.Zero;
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

            // Solve linear motor constraint.
            if (_enableMotor && _limitState != LimitState.Equal)
            {
                Fix64 Cdot = AetherVector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
                Fix64 impulse = _motorMass * (_motorSpeed - Cdot);
                Fix64 oldImpulse = MotorImpulse;
                Fix64 maxImpulse = data.step.dt * _maxMotorForce;
                MotorImpulse = MathUtils.Clamp(MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = MotorImpulse - oldImpulse;

                AetherVector2 P = impulse * _axis;
                Fix64 LA = impulse * _a1;
                Fix64 LB = impulse * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            AetherVector2 Cdot1 = new AetherVector2();
            Cdot1.X = AetherVector2.Dot(_perp, vB - vA) + _s2 * wB - _s1 * wA;
            Cdot1.Y = wB - wA;

            if (_enableLimit && _limitState != LimitState.Inactive)
            {
                // Solve prismatic and limit constraint in block form.
                Fix64 Cdot2;
                Cdot2 = AetherVector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
                AetherVector3 Cdot = new AetherVector3(Cdot1.X, Cdot1.Y, Cdot2);

                AetherVector3 f1 = _impulse;
                AetherVector3 df = _K.Solve33(-Cdot);
                _impulse += df;

                if (_limitState == LimitState.AtLower)
                {
                    _impulse.Z = MathUtils.Max(_impulse.Z, Fix64.Zero);
                }
                else if (_limitState == LimitState.AtUpper)
                {
                    _impulse.Z = MathUtils.Min(_impulse.Z, Fix64.Zero);
                }

                // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
                AetherVector2 b = -Cdot1 - (_impulse.Z - f1.Z) * new AetherVector2(_K.ez.X, _K.ez.Y);
                AetherVector2 f2r = _K.Solve22(b) + new AetherVector2(f1.X, f1.Y);
                _impulse.X = f2r.X;
                _impulse.Y = f2r.Y;

                df = _impulse - f1;

                AetherVector2 P = df.X * _perp + df.Z * _axis;
                Fix64 LA = df.X * _s1 + df.Y + df.Z * _a1;
                Fix64 LB = df.X * _s2 + df.Y + df.Z * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else
            {
                // Limit is inactive, just solve the prismatic constraint in block form.
                AetherVector2 df = _K.Solve22(-Cdot1);
                _impulse.X += df.X;
                _impulse.Y += df.Y;

                AetherVector2 P = df.X * _perp;
                Fix64 LA = df.X * _s1 + df.Y;
                Fix64 LB = df.X * _s2 + df.Y;

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

            Fix64 mA = _invMassA, mB = _invMassB;
            Fix64 iA = _invIA, iB = _invIB;

            // Compute fresh Jacobians
            AetherVector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
            AetherVector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);
            AetherVector2 d = cB + rB - cA - rA;

            AetherVector2 axis = Complex.Multiply(ref _localXAxis, ref qA);
            Fix64 a1 = MathUtils.Cross(d + rA, axis);
            Fix64 a2 = MathUtils.Cross(ref rB, ref axis);
            AetherVector2 perp = Complex.Multiply(ref _localYAxisA, ref qA);

            Fix64 s1 = MathUtils.Cross(d + rA, perp);
            Fix64 s2 = MathUtils.Cross(ref rB, ref perp);

            AetherVector3 impulse;
            AetherVector2 C1 = new AetherVector2();
            C1.X = AetherVector2.Dot(perp, d);
            C1.Y = aB - aA - ReferenceAngle;

            Fix64 linearError =  Fix64.Abs(C1.X);
            Fix64 angularError =  Fix64.Abs(C1.Y);

            bool active = false;
            Fix64 C2 = Fix64.Zero;
            if (_enableLimit)
            {
                Fix64 translation = AetherVector2.Dot(axis, d);
                if ( Fix64.Abs(_upperTranslation - _lowerTranslation) < Fix64Constants.Two * Settings.LinearSlop)
                {
                    // Prevent large angular corrections
                    C2 = MathUtils.Clamp(translation, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);
                    linearError = MathUtils.Max(linearError,  Fix64.Abs(translation));
                    active = true;
                }
                else if (translation <= _lowerTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - _lowerTranslation + Settings.LinearSlop, -Settings.MaxLinearCorrection, Fix64.Zero);
                    linearError = MathUtils.Max(linearError, _lowerTranslation - translation);
                    active = true;
                }
                else if (translation >= _upperTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - _upperTranslation - Settings.LinearSlop, Fix64.Zero, Settings.MaxLinearCorrection);
                    linearError = MathUtils.Max(linearError, translation - _upperTranslation);
                    active = true;
                }
            }

            if (active)
            {
                Fix64 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                Fix64 k12 = iA * s1 + iB * s2;
                Fix64 k13 = iA * s1 * a1 + iB * s2 * a2;
                Fix64 k22 = iA + iB;
                if (k22 == Fix64.Zero)
                {
                    // For fixed rotation
                    k22 = Fix64.One;
                }
                Fix64 k23 = iA * a1 + iB * a2;
                Fix64 k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

                Mat33 K = new Mat33();
                K.ex = new AetherVector3(k11, k12, k13);
                K.ey = new AetherVector3(k12, k22, k23);
                K.ez = new AetherVector3(k13, k23, k33);

                AetherVector3 C = new AetherVector3();
                C.X = C1.X;
                C.Y = C1.Y;
                C.Z = C2;

                impulse = K.Solve33(-C);
            }
            else
            {
                Fix64 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                Fix64 k12 = iA * s1 + iB * s2;
                Fix64 k22 = iA + iB;
                if (k22 == Fix64.Zero)
                {
                    k22 = Fix64.One;
                }

                Mat22 K = new Mat22();
                K.ex = new AetherVector2(k11, k12);
                K.ey = new AetherVector2(k12, k22);

                AetherVector2 impulse1 = K.Solve(-C1);
                impulse = new AetherVector3();
                impulse.X = impulse1.X;
                impulse.Y = impulse1.Y;
                impulse.Z = Fix64.Zero;
            }

            AetherVector2 P = impulse.X * perp + impulse.Z * axis;
            Fix64 LA = impulse.X * s1 + impulse.Y + impulse.Z * a1;
            Fix64 LB = impulse.X * s2 + impulse.Y + impulse.Z * a2;

            cA -= mA * P;
            aA -= iA * LA;
            cB += mB * P;
            aB += iB * LB;

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}