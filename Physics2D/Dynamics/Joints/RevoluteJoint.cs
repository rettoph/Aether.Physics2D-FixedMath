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
    /// <summary>
    /// A revolute joint constrains to bodies to share a common point while they
    /// are free to rotate about the point. The relative rotation about the shared
    /// point is the joint angle. You can limit the relative rotation with
    /// a joint limit that specifies a lower and upper angle. You can use a motor
    /// to drive the relative rotation about the shared point. A maximum motor torque
    /// is provided so that infinite forces are not generated.
    /// </summary>
    public class RevoluteJoint : Joint
    {
        // Solver shared
        private AetherVector3 _impulse;
        private Fix64 _motorImpulse;

        private bool _enableMotor;
        private Fix64 _maxMotorTorque;
        private Fix64 _motorSpeed;

        private bool _enableLimit;
        private Fix64 _referenceAngle;
        private Fix64 _lowerAngle;
        private Fix64 _upperAngle;

        // Solver temp
        private int _indexA;
        private int _indexB;
        private AetherVector2 _rA;
        private AetherVector2 _rB;
        private AetherVector2 _localCenterA;
        private AetherVector2 _localCenterB;
        private Fix64 _invMassA;
        private Fix64 _invMassB;
        private Fix64 _invIA;
        private Fix64 _invIB;
        private Mat33 _mass;			// effective mass for point-to-point constraint.
        private Fix64 _motorMass;	    // effective mass for motor/limit angular constraint.
        private LimitState _limitState;

        internal RevoluteJoint()
        {
            JointType = JointType.Revolute;
        }

        /// <summary>
        /// Constructor of RevoluteJoint. 
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="anchorA">The first body anchor.</param>
        /// <param name="anchorB">The second anchor.</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public RevoluteJoint(Body bodyA, Body bodyB, AetherVector2 anchorA, AetherVector2 anchorB, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            JointType = JointType.Revolute;

            if (useWorldCoordinates)
            {
                LocalAnchorA = BodyA.GetLocalPoint(anchorA);
                LocalAnchorB = BodyB.GetLocalPoint(anchorB);
            }
            else
            {
                LocalAnchorA = anchorA;
                LocalAnchorB = anchorB;
            }

            ReferenceAngle = BodyB.Rotation - BodyA.Rotation;

            _impulse = AetherVector3.Zero;
            _limitState = LimitState.Inactive;
        }

        /// <summary>
        /// Constructor of RevoluteJoint. 
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="anchor">The shared anchor.</param>
        /// <param name="useWorldCoordinates"></param>
        public RevoluteJoint(Body bodyA, Body bodyB, AetherVector2 anchor, bool useWorldCoordinates = false)
            : this(bodyA, bodyB, anchor, anchor, useWorldCoordinates)
        {
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
        /// The referance angle computed as BodyB angle minus BodyA angle.
        /// </summary>
        public Fix64 ReferenceAngle
        {
            get { return _referenceAngle; }
            set
            {
                WakeBodies();
                _referenceAngle = value;
            }
        }

        /// <summary>
        /// Get the current joint angle in radians.
        /// </summary>
        public Fix64 JointAngle
        {
            get { return BodyB._sweep.A - BodyA._sweep.A - ReferenceAngle; }
        }

        /// <summary>
        /// Get the current joint angle speed in radians per second.
        /// </summary>
        public Fix64 JointSpeed
        {
            get { return BodyB._angularVelocity - BodyA._angularVelocity; }
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
                if (_enableLimit != value)
                {
                    WakeBodies();
                    _enableLimit = value;
                    _impulse.Z = Fix64.Zero;
                }
            }
        }

        /// <summary>
        /// Get the lower joint limit in radians.
        /// </summary>
        public Fix64 LowerLimit
        {
            get { return _lowerAngle; }
            set
            {
                if (_lowerAngle != value)
                {
                    WakeBodies();
                    _lowerAngle = value;
                    _impulse.Z = Fix64.Zero;
                }
            }
        }

        /// <summary>
        /// Get the upper joint limit in radians.
        /// </summary>
        public Fix64 UpperLimit
        {
            get { return _upperAngle; }
            set
            {
                if (_upperAngle != value)
                {
                    WakeBodies();
                    _upperAngle = value;
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
            if (lower != _lowerAngle || upper != _upperAngle)
            {
                WakeBodies();
                _upperAngle = upper;
                _lowerAngle = lower;
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
        /// Get or set the motor speed in radians per second.
        /// </summary>
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
        /// Get or set the maximum motor torque, usually in N-m.
        /// </summary>
        public Fix64 MaxMotorTorque
        {
            set
            {
                WakeBodies();
                _maxMotorTorque = value;
            }
            get { return _maxMotorTorque; }
        }

        /// <summary>
        /// Get or set the current motor impulse, usually in N-m.
        /// </summary>
        public Fix64 MotorImpulse
        {
            get { return _motorImpulse; }
            set
            {
                WakeBodies();
                _motorImpulse = value;
            }
        }

        /// <summary>
        /// Gets the motor torque in N-m.
        /// </summary>
        /// <param name="invDt">The inverse delta time</param>
        public Fix64 GetMotorTorque(Fix64 invDt)
        {
            return invDt * _motorImpulse;
        }

        public override AetherVector2 GetReactionForce(Fix64 invDt)
        {
            AetherVector2 p = new AetherVector2(_impulse.X, _impulse.Y);
            return invDt * p;
        }

        public override Fix64 GetReactionTorque(Fix64 invDt)
        {
            return invDt * _impulse.Z;
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

            Fix64 aA = data.positions[_indexA].a;
            AetherVector2 vA = data.velocities[_indexA].v;
            Fix64 wA = data.velocities[_indexA].w;

            Fix64 aB = data.positions[_indexB].a;
            AetherVector2 vB = data.velocities[_indexB].v;
            Fix64 wB = data.velocities[_indexB].w;

            Complex qA = Complex.FromAngle(aA);
            Complex qB = Complex.FromAngle(aB);

            _rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
            _rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);

            // J = [-I -r1_skew I r2_skew]
            //     [ 0       -1 0       1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
            //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
            //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

            Fix64 mA = _invMassA, mB = _invMassB;
            Fix64 iA = _invIA, iB = _invIB;

            bool fixedRotation = (iA + iB == Fix64.Zero);

            _mass.ex.X = mA + mB + _rA.Y * _rA.Y * iA + _rB.Y * _rB.Y * iB;
            _mass.ey.X = -_rA.Y * _rA.X * iA - _rB.Y * _rB.X * iB;
            _mass.ez.X = -_rA.Y * iA - _rB.Y * iB;
            _mass.ex.Y = _mass.ey.X;
            _mass.ey.Y = mA + mB + _rA.X * _rA.X * iA + _rB.X * _rB.X * iB;
            _mass.ez.Y = _rA.X * iA + _rB.X * iB;
            _mass.ex.Z = _mass.ez.X;
            _mass.ey.Z = _mass.ez.Y;
            _mass.ez.Z = iA + iB;

            _motorMass = iA + iB;
            if (_motorMass > Fix64.Zero)
            {
                _motorMass = Fix64.One / _motorMass;
            }

            if (_enableMotor == false || fixedRotation)
            {
                _motorImpulse = Fix64.Zero;
            }

            if (_enableLimit && fixedRotation == false)
            {
                Fix64 jointAngle = aB - aA - ReferenceAngle;
                if ( Fix64.Abs(_upperAngle - _lowerAngle) < Fix64Constants.Two * Settings.AngularSlop)
                {
                    _limitState = LimitState.Equal;
                }
                else if (jointAngle <= _lowerAngle)
                {
                    if (_limitState != LimitState.AtLower)
                    {
                        _impulse.Z = Fix64.Zero;
                    }
                    _limitState = LimitState.AtLower;
                }
                else if (jointAngle >= _upperAngle)
                {
                    if (_limitState != LimitState.AtUpper)
                    {
                        _impulse.Z = Fix64.Zero;
                    }
                    _limitState = LimitState.AtUpper;
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
            }

            if (data.step.warmStarting)
            {
                // Scale impulses to support a variable time step.
                _impulse *= data.step.dtRatio;
                _motorImpulse *= data.step.dtRatio;

                AetherVector2 P = new AetherVector2(_impulse.X, _impulse.Y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(ref _rA, ref P) + MotorImpulse + _impulse.Z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(ref _rB, ref P) + MotorImpulse + _impulse.Z);
            }
            else
            {
                _impulse = AetherVector3.Zero;
                _motorImpulse = Fix64.Zero;
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

            bool fixedRotation = (iA + iB == Fix64.Zero);

            // Solve motor constraint.
            if (_enableMotor && _limitState != LimitState.Equal && fixedRotation == false)
            {
                Fix64 Cdot = wB - wA - _motorSpeed;
                Fix64 impulse = _motorMass * (-Cdot);
                Fix64 oldImpulse = _motorImpulse;
                Fix64 maxImpulse = data.step.dt * _maxMotorTorque;
                _motorImpulse = MathUtils.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _motorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve limit constraint.
            if (_enableLimit && _limitState != LimitState.Inactive && fixedRotation == false)
            {
                AetherVector2 Cdot1 = vB + MathUtils.Cross(wB, ref _rB) - vA - MathUtils.Cross(wA, ref _rA);
                Fix64 Cdot2 = wB - wA;
                AetherVector3 Cdot = new AetherVector3(Cdot1.X, Cdot1.Y, Cdot2);

                AetherVector3 impulse = -_mass.Solve33(Cdot);

                if (_limitState == LimitState.Equal)
                {
                    _impulse += impulse;
                }
                else if (_limitState == LimitState.AtLower)
                {
                    Fix64 newImpulse = _impulse.Z + impulse.Z;
                    if (newImpulse < Fix64.Zero)
                    {
                        AetherVector2 rhs = -Cdot1 + _impulse.Z * new AetherVector2(_mass.ez.X, _mass.ez.Y);
                        AetherVector2 reduced = _mass.Solve22(rhs);
                        impulse.X = reduced.X;
                        impulse.Y = reduced.Y;
                        impulse.Z = -_impulse.Z;
                        _impulse.X += reduced.X;
                        _impulse.Y += reduced.Y;
                        _impulse.Z = Fix64.Zero;
                    }
                    else
                    {
                        _impulse += impulse;
                    }
                }
                else if (_limitState == LimitState.AtUpper)
                {
                    Fix64 newImpulse = _impulse.Z + impulse.Z;
                    if (newImpulse > Fix64.Zero)
                    {
                        AetherVector2 rhs = -Cdot1 + _impulse.Z * new AetherVector2(_mass.ez.X, _mass.ez.Y);
                        AetherVector2 reduced = _mass.Solve22(rhs);
                        impulse.X = reduced.X;
                        impulse.Y = reduced.Y;
                        impulse.Z = -_impulse.Z;
                        _impulse.X += reduced.X;
                        _impulse.Y += reduced.Y;
                        _impulse.Z = Fix64.Zero;
                    }
                    else
                    {
                        _impulse += impulse;
                    }
                }

                AetherVector2 P = new AetherVector2(impulse.X, impulse.Y);

                vA -= mA * P;
                wA -= iA * (MathUtils.Cross(ref _rA, ref P) + impulse.Z);

                vB += mB * P;
                wB += iB * (MathUtils.Cross(ref _rB, ref P) + impulse.Z);
            }
            else
            {
                // Solve point-to-point constraint
                AetherVector2 Cdot = vB + MathUtils.Cross(wB, ref _rB) - vA - MathUtils.Cross(wA, ref _rA);
                AetherVector2 impulse = _mass.Solve22(-Cdot);

                _impulse.X += impulse.X;
                _impulse.Y += impulse.Y;

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
            AetherVector2 cA = data.positions[_indexA].c;
            Fix64 aA = data.positions[_indexA].a;
            AetherVector2 cB = data.positions[_indexB].c;
            Fix64 aB = data.positions[_indexB].a;


            Fix64 angularError = Fix64.Zero;
            Fix64 positionError;

            bool fixedRotation = (_invIA + _invIB == Fix64.Zero);

            // Solve angular limit constraint.
            if (_enableLimit && _limitState != LimitState.Inactive && fixedRotation == false)
            {
                Fix64 angle = aB - aA - ReferenceAngle;
                Fix64 limitImpulse = Fix64.Zero;

                if (_limitState == LimitState.Equal)
                {
                    // Prevent large angular corrections
                    Fix64 C = MathUtils.Clamp(angle - _lowerAngle, -Settings.MaxAngularCorrection, Settings.MaxAngularCorrection);
                    limitImpulse = -_motorMass * C;
                    angularError =  Fix64.Abs(C);
                }
                else if (_limitState == LimitState.AtLower)
                {
                    Fix64 C = angle - _lowerAngle;
                    angularError = -C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(C + Settings.AngularSlop, -Settings.MaxAngularCorrection, Fix64.Zero);
                    limitImpulse = -_motorMass * C;
                }
                else if (_limitState == LimitState.AtUpper)
                {
                    Fix64 C = angle - _upperAngle;
                    angularError = C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(C - Settings.AngularSlop, Fix64.Zero, Settings.MaxAngularCorrection);
                    limitImpulse = -_motorMass * C;
                }

                aA -= _invIA * limitImpulse;
                aB += _invIB * limitImpulse;
            }

            // Solve point-to-point constraint.
            {
                Complex qA = Complex.FromAngle(aA);
                Complex qB = Complex.FromAngle(aB);
                AetherVector2 rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
                AetherVector2 rB = Complex.Multiply(LocalAnchorB - _localCenterB, ref qB);

                AetherVector2 C = cB + rB - cA - rA;
                positionError = C.Length();

                Fix64 mA = _invMassA, mB = _invMassB;
                Fix64 iA = _invIA, iB = _invIB;

                Mat22 K = new Mat22();
                K.ex.X = mA + mB + iA * rA.Y * rA.Y + iB * rB.Y * rB.Y;
                K.ex.Y = -iA * rA.X * rA.Y - iB * rB.X * rB.Y;
                K.ey.X = K.ex.Y;
                K.ey.Y = mA + mB + iA * rA.X * rA.X + iB * rB.X * rB.X;

                AetherVector2 impulse = -K.Solve(C);

                cA -= mA * impulse;
                aA -= iA * MathUtils.Cross(ref rA, ref impulse);

                cB += mB * impulse;
                aB += iB * MathUtils.Cross(ref rB, ref impulse);
            }

            data.positions[_indexA].c = cA;
            data.positions[_indexA].a = aA;
            data.positions[_indexB].c = cB;
            data.positions[_indexB].a = aB;

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}