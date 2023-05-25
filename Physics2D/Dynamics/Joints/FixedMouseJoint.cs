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
    // p = attached point, m = mouse point
    // C = p - m
    // Cdot = v
    //      = v + cross(w, r)
    // J = [I r_skew]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    /// <summary>
    /// A mouse joint is used to make a point on a body track a
    /// specified world point. This a soft constraint with a maximum
    /// force. This allows the constraint to stretch and without
    /// applying huge forces.
    /// NOTE: this joint is not documented in the manual because it was
    /// developed to be used in the testbed. If you want to learn how to
    /// use the mouse joint, look at the testbed.
    /// </summary>
    public class FixedMouseJoint : Joint
    {
        private AetherVector2 _worldAnchor;
        private Fix64 _frequency;
        private Fix64 _dampingRatio;
        private Fix64 _beta;

        // Solver shared
        private AetherVector2 _impulse;
        private Fix64 _maxForce;
        private Fix64 _gamma;

        // Solver temp
        private int _indexA;
        private AetherVector2 _rA;
        private AetherVector2 _localCenterA;
        private Fix64 _invMassA;
        private Fix64 _invIA;
        private Mat22 _mass;
        private AetherVector2 _C;

        /// <summary>
        /// This requires a world target point,
        /// tuning parameters, and the time step.
        /// </summary>
        /// <param name="body">The body.</param>
        /// <param name="worldAnchor">The target.</param>
        public FixedMouseJoint(Body body, AetherVector2 worldAnchor)
            : base(body)
        {
            JointType = JointType.FixedMouse;
            Frequency = Fix64Constants.Five;
            DampingRatio = Fix64Constants.Seven;
            MaxForce = Fix64Constants.OneThousand * body.Mass;

            _worldAnchor = worldAnchor;
            LocalAnchorA = Transform.Divide(ref worldAnchor, ref BodyA._xf);
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public AetherVector2 LocalAnchorA { get; set; }

        public override AetherVector2 WorldAnchorA
        {
            get { return BodyA.GetWorldPoint(LocalAnchorA); }
            set { LocalAnchorA = BodyA.GetLocalPoint(value); }
        }

        public override AetherVector2 WorldAnchorB
        {
            get { return _worldAnchor; }
            set
            {
                WakeBodies();
                _worldAnchor = value;
            }
        }

        /// <summary>
        /// The maximum constraint force that can be exerted
        /// to move the candidate body. Usually you will express
        /// as some multiple of the weight (multiplier * mass * gravity).
        /// </summary>
        public Fix64 MaxForce
        {
            get { return _maxForce; }
            set
            {
                Debug.Assert(value >= Fix64.Zero);
                _maxForce = value;
            }
        }

        /// <summary>
        /// The response speed.
        /// </summary>
        public Fix64 Frequency
        {
            get { return _frequency; }
            set
            {
                Debug.Assert(value >= Fix64.Zero);
                _frequency = value;
            }
        }

        /// <summary>
        /// The damping ratio. 0 = no damping, 1 = critical damping.
        /// </summary>
        public Fix64 DampingRatio
        {
            get { return _dampingRatio; }
            set
            {
                Debug.Assert(value >= Fix64.Zero);
                _dampingRatio = value;
            }
        }

        public override AetherVector2 GetReactionForce(Fix64 invDt)
        {
            return invDt * _impulse;
        }

        public override Fix64 GetReactionTorque(Fix64 invDt)
        {
            return invDt * Fix64.Zero;
        }

        internal override void InitVelocityConstraints(ref SolverData data)
        {
            _indexA = BodyA.IslandIndex;
            _localCenterA = BodyA._sweep.LocalCenter;
            _invMassA = BodyA._invMass;
            _invIA = BodyA._invI;

            AetherVector2 cA = data.positions[_indexA].c;
            Fix64 aA = data.positions[_indexA].a;
            AetherVector2 vA = data.velocities[_indexA].v;
            Fix64 wA = data.velocities[_indexA].w;

            Complex qA = Complex.FromAngle(aA);

            Fix64 mass = BodyA.Mass;

            // Frequency
            Fix64 omega = Constant.Tau * Frequency;

            // Damping coefficient
            Fix64 d = Fix64Constants.Two * mass * DampingRatio * omega;

            // Spring stiffness
            Fix64 k = mass * (omega * omega);

            // magic formulas
            // gamma has units of inverse mass.
            // beta has units of inverse time.
            Fix64 h = data.step.dt;
            Debug.Assert(d + h * k > Settings.Epsilon);
            _gamma = h * (d + h * k);
            if (_gamma != Fix64.Zero)
            {
                _gamma = Fix64.One / _gamma;
            }

            _beta = h * k * _gamma;

            // Compute the effective mass matrix.
            _rA = Complex.Multiply(LocalAnchorA - _localCenterA, ref qA);
            // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
            //      = [1/m1+1/m2     0    ] + invI1 * [r1.Y*r1.Y -r1.X*r1.Y] + invI2 * [r1.Y*r1.Y -r1.X*r1.Y]
            //        [    0     1/m1+1/m2]           [-r1.X*r1.Y r1.X*r1.X]           [-r1.X*r1.Y r1.X*r1.X]
            Mat22 K = new Mat22();
            K.ex.X = _invMassA + _invIA * _rA.Y * _rA.Y + _gamma;
            K.ex.Y = -_invIA * _rA.X * _rA.Y;
            K.ey.X = K.ex.Y;
            K.ey.Y = _invMassA + _invIA * _rA.X * _rA.X + _gamma;

            _mass = K.Inverse;

            _C = cA + _rA - _worldAnchor;
            _C *= _beta;

            // Cheat with some damping
            wA *= Fix64Constants.PointNineEight;

            if (data.step.warmStarting)
            {
                _impulse *= data.step.dtRatio;
                vA += _invMassA * _impulse;
                wA += _invIA * MathUtils.Cross(ref _rA, ref _impulse);
            }
            else
            {
                _impulse = AetherVector2.Zero;
            }

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            AetherVector2 vA = data.velocities[_indexA].v;
            Fix64 wA = data.velocities[_indexA].w;

            // Cdot = v + cross(w, r)
            AetherVector2 Cdot = vA + MathUtils.Cross(wA, ref _rA);
            AetherVector2 impulse = MathUtils.Mul(ref _mass, -(Cdot + _C + _gamma * _impulse));

            AetherVector2 oldImpulse = _impulse;
            _impulse += impulse;
            Fix64 maxImpulse = data.step.dt * MaxForce;
            if (_impulse.LengthSquared() > maxImpulse * maxImpulse)
            {
                _impulse *= maxImpulse / _impulse.Length();
            }
            impulse = _impulse - oldImpulse;

            vA += _invMassA * impulse;
            wA += _invIA * MathUtils.Cross(ref _rA, ref impulse);

            data.velocities[_indexA].v = vA;
            data.velocities[_indexA].w = wA;
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            return true;
        }
    }
}