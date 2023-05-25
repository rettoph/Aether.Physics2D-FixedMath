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

namespace tainicom.Aether.Physics2D.Collision.Shapes
{
    /// <summary>
    /// A circle shape.
    /// </summary>
    public class CircleShape : Shape
    {
        internal AetherVector2 _position;

        /// <summary>
        /// Create a new circle with the desired radius and density.
        /// </summary>
        /// <param name="radius">The radius of the circle.</param>
        /// <param name="density">The density of the circle.</param>
        public CircleShape(Fix64 radius, Fix64 density)
            : base(density)
        {
            Debug.Assert(radius >= Fix64.Zero);
            Debug.Assert(density >= Fix64.Zero);

            ShapeType = ShapeType.Circle;
            _position = AetherVector2.Zero;
            Radius = radius; // The Radius property cache 2radius and calls ComputeProperties(). So no need to call ComputeProperties() here.
        }

        internal CircleShape()
            : base(Fix64.Zero)
        {
            ShapeType = ShapeType.Circle;
            _radius = Fix64.Zero;
            _position = AetherVector2.Zero;
        }

        public override int ChildCount
        {
            get { return 1; }
        }

        /// <summary>
        /// Get or set the position of the circle
        /// </summary>
        public AetherVector2 Position
        {
            get { return _position; }
            set
            {
                _position = value;
                ComputeProperties(); //TODO: Optimize here
            }
        }

        public override bool TestPoint(ref Transform transform, ref AetherVector2 point)
        {
            AetherVector2 center = transform.p + Complex.Multiply(ref _position, ref transform.q);
            AetherVector2 d = point - center;
            return AetherVector2.Dot(d, d) <= _2radius;
        }

        public override bool RayCast(out RayCastOutput output, ref RayCastInput input, ref Transform transform, int childIndex)
        {
            // Collision Detection in Interactive 3D Environments by Gino van den Bergen
            // From Section 3.1.2
            // x = s + a * r
            // norm(x) = radius

            output = new RayCastOutput();

            AetherVector2 position = transform.p + Complex.Multiply(ref _position, ref transform.q);
            AetherVector2 s = input.Point1 - position;
            Fix64 b = AetherVector2.Dot(s, s) - _2radius;

            // Solve quadratic equation.
            AetherVector2 r = input.Point2 - input.Point1;
            Fix64 c = AetherVector2.Dot(s, r);
            Fix64 rr = AetherVector2.Dot(r, r);
            Fix64 sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < Fix64.Zero || rr < Settings.Epsilon)
            {
                return false;
            }

            // Find the point of intersection of the line with the circle.
            Fix64 a = -(c + Fix64.Sqrt(sigma));

            // Is the intersection point on the segment?
            if (Fix64.Zero <= a && a <= input.MaxFraction * rr)
            {
                a /= rr;
                output.Fraction = a;

                //TODO: Check results here
                output.Normal = s + a * r;
                output.Normal.Normalize();
                return true;
            }

            return false;
        }

        public override void ComputeAABB(out AABB aabb, ref Transform transform, int childIndex)
        {
            // OPT: Vector2 p = transform.p + Complex.Multiply(ref _position, ref transform.q);
            var pX = (_position.X * transform.q.R - _position.Y * transform.q.i) + transform.p.X;
            var pY = (_position.Y * transform.q.R + _position.X * transform.q.i) + transform.p.Y;

            // OPT: aabb.LowerBound = new Vector2(p.X - Radius, p.Y - Radius);
            // OPT: aabb.UpperBound = new Vector2(p.X + Radius, p.Y + Radius);
            aabb.LowerBound.X = pX - Radius;
            aabb.LowerBound.Y = pY - Radius;
            aabb.UpperBound.X = pX + Radius;
            aabb.UpperBound.Y = pY + Radius;
        }

        protected override sealed void ComputeProperties()
        {
            Fix64 area = Constant.Pi * _2radius;
            MassData.Area = area;
            MassData.Mass = Density * area;
            MassData.Centroid = Position;

            // inertia about the local origin
            MassData.Inertia = MassData.Mass * (Fix64Constants.PointFive * _2radius + AetherVector2.Dot(Position, Position));
        }

        public override Fix64 ComputeSubmergedArea(ref AetherVector2 normal, Fix64 offset, ref Transform xf, out AetherVector2 sc)
        {
            sc = AetherVector2.Zero;

            AetherVector2 p = Transform.Multiply(ref _position, ref xf);
            Fix64 l = -(AetherVector2.Dot(normal, p) - offset);
            if (l < -Radius + Settings.Epsilon)
            {
                //Completely dry
                return Fix64.Zero;
            }
            if (l > Radius)
            {
                //Completely wet
                sc = p;
                return Constant.Pi * _2radius;
            }

            //Magic
            Fix64 l2 = l * l;
            Fix64 area = _2radius * ((MathUtils.Asin(l / Radius) + Fix64.PiOver2) + l * Fix64.Sqrt(_2radius - l2));
            Fix64 com = -Fix64Constants.Two / Fix64Constants.Three * Fix64.Pow(_2radius - l2, Fix64Constants.OnePointFive) / area;

            sc.X = p.X + normal.X * com;
            sc.Y = p.Y + normal.Y * com;

            return area;
        }

        /// <summary>
        /// Compare the circle to another circle
        /// </summary>
        /// <param name="shape">The other circle</param>
        /// <returns>True if the two circles are the same size and have the same position</returns>
        public bool CompareTo(CircleShape shape)
        {
            return (Radius == shape.Radius && Position == shape.Position);
        }

        public override Shape Clone()
        {
            CircleShape clone = new CircleShape();
            clone.ShapeType = ShapeType;
            clone._radius = Radius;
            clone._2radius = _2radius; //FPE note: We also copy the cache
            clone._density = _density;
            clone._position = _position;
            clone.MassData = MassData;
            return clone;
        }
    }
}