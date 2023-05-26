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
using System.Runtime.InteropServices;
#if XNAAPI
using Complex = tainicom.Aether.Physics2D.Common.Complex;
using Vector2 = Microsoft.Xna.Framework.Vector2;
using Vector3 = Microsoft.Xna.Framework.Vector3;
#endif

namespace tainicom.Aether.Physics2D.Common
{
    public static class MathUtils
    {
        public static Fix64 Cross(ref AetherVector2 a, ref AetherVector2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        public static Fix64 Cross(AetherVector2 a, AetherVector2 b)
        {
            return Cross(ref a, ref b);
        }

        /// Perform the cross product on two vectors.
        public static AetherVector3 Cross(ref AetherVector3 a, ref AetherVector3 b)
        {
            return new AetherVector3( a.Y * b.Z - a.Z * b.Y, 
                                a.Z * b.X - a.X * b.Z, 
                                a.X * b.Y - a.Y * b.X);
        }

        public static AetherVector2 Cross(AetherVector2 a, Fix64 s)
        {
            return new AetherVector2(s * a.Y, -s * a.X);
        }

        public static AetherVector2 Rot270(ref AetherVector2 a)
        {
            return new AetherVector2(a.Y, -a.X);
        }

        public static AetherVector2 Cross(Fix64 s, ref AetherVector2 a)
        {
            return new AetherVector2(-s * a.Y, s * a.X);
        }

        public static AetherVector2 Rot90(ref AetherVector2 a)
        {
            return new AetherVector2(-a.Y, a.X);
        }

        public static AetherVector2 Abs(AetherVector2 v)
        {
            return new AetherVector2( Fix64.Abs(v.X),  Fix64.Abs(v.Y));
        }

        public static AetherVector2 Mul(ref Mat22 A, AetherVector2 v)
        {
            return Mul(ref A, ref v);
        }

        public static AetherVector2 Mul(ref Mat22 A, ref AetherVector2 v)
        {
            return new AetherVector2(A.ex.X * v.X + A.ey.X * v.Y, A.ex.Y * v.X + A.ey.Y * v.Y);
        }
        
        public static AetherVector2 MulT(ref Mat22 A, AetherVector2 v)
        {
            return MulT(ref A, ref v);
        }

        public static AetherVector2 MulT(ref Mat22 A, ref AetherVector2 v)
        {
            return new AetherVector2(v.X * A.ex.X + v.Y * A.ex.Y, v.X * A.ey.X + v.Y * A.ey.Y);
        }


        // A^T * B
        public static void MulT(ref Mat22 A, ref Mat22 B, out Mat22 C)
        {
            C.ex.X = A.ex.X * B.ex.X + A.ex.Y * B.ex.Y;
            C.ex.Y = A.ey.X * B.ex.X + A.ey.Y * B.ex.Y;
            C.ey.X = A.ex.X * B.ey.X + A.ex.Y * B.ey.Y;
            C.ey.Y = A.ey.X * B.ey.X + A.ey.Y * B.ey.Y;
        }

        /// Multiply a matrix times a vector.
        public static AetherVector3 Mul(Mat33 A, AetherVector3 v)
        {
            return v.X * A.ex + v.Y * A.ey + v.Z * A.ez;
        }
        
        public static void Swap<T>(ref T a, ref T b)
        {
            T tmp = a;
            a = b;
            b = tmp;
        }

        /// Multiply a matrix times a vector.
        public static AetherVector2 Mul22(Mat33 A, AetherVector2 v)
        {
            return new AetherVector2(A.ex.X * v.X + A.ey.X * v.Y, A.ex.Y * v.X + A.ey.Y * v.Y);
        }
        
        /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
        public static AetherVector2 Skew(AetherVector2 input)
        {
            return new AetherVector2(-input.Y, input.X);
        }

        public static int Clamp(int a, int low, int high)
        {
            return Math.Max(low, Math.Min(a, high));
        }

        public static Fix64 Clamp(Fix64 a, Fix64 low, Fix64 high)
        {
            return MathUtils.Max(low, MathUtils.Min(a, high));
        }

        public static AetherVector2 Clamp(AetherVector2 a, AetherVector2 low, AetherVector2 high)
        {
            a.X = MathUtils.Max(low.X, MathUtils.Min(a.X, high.X));
            a.Y = MathUtils.Max(low.Y, MathUtils.Min(a.Y, high.Y));
            return a;
        }

        public static void Cross(ref AetherVector2 a, ref AetherVector2 b, out Fix64 c)
        {
            c = a.X * b.Y - a.Y * b.X;
        }

        /// <summary>
        /// Return the angle between two vectors on a plane
        /// The angle is from vector 1 to vector 2, positive anticlockwise
        /// The result is between -pi -> pi
        /// </summary>
        public static Fix64 VectorAngle(ref AetherVector2 p1, ref AetherVector2 p2)
        {
            Fix64 theta1 = Fix64.Atan2(p1.Y, p1.X);
            Fix64 theta2 = Fix64.Atan2(p2.Y, p2.X);
            Fix64 dtheta = theta2 - theta1;
            while (dtheta > Fix64.Pi)
                dtheta -= Fix64.PiTimes2;
            while (dtheta < -Fix64.Pi)
                dtheta += Fix64.PiTimes2;

            return (dtheta);
        }

        /// Perform the dot product on two vectors.
        public static Fix64 Dot(AetherVector3 a, AetherVector3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        /// Perform the dot product on two vectors.
        public static Fix64 Dot(AetherVector2 a, ref AetherVector2 b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        public static Fix64 VectorAngle(AetherVector2 p1, AetherVector2 p2)
        {
            return VectorAngle(ref p1, ref p2);
        }

        /// <summary>
        /// Returns a positive number if c is to the left of the line going from a to b.
        /// </summary>
        /// <returns>Positive number if point is left, negative if point is right, 
        /// and 0 if points are collinear.</returns>
        public static Fix64 Area(AetherVector2 a, AetherVector2 b, AetherVector2 c)
        {
            return Area(ref a, ref b, ref c);
        }

        /// <summary>
        /// Returns a positive number if c is to the left of the line going from a to b.
        /// </summary>
        /// <returns>Positive number if point is left, negative if point is right, 
        /// and 0 if points are collinear.</returns>
        public static Fix64 Area(ref AetherVector2 a, ref AetherVector2 b, ref AetherVector2 c)
        {
            return a.X * (b.Y - c.Y) + b.X * (c.Y - a.Y) + c.X * (a.Y - b.Y);
        }

        /// <summary>
        /// Determines if three vertices are collinear (ie. on a straight line)
        /// </summary>
        /// <param name="a">First vertex</param>
        /// <param name="b">Second vertex</param>
        /// <param name="c">Third vertex</param>
        /// <param name="tolerance">The tolerance</param>
        /// <returns></returns>
        public static bool IsCollinear(ref AetherVector2 a, ref AetherVector2 b, ref AetherVector2 c, Fix64 tolerance)
        {
            return FloatInRange(Area(ref a, ref b, ref c), -tolerance, tolerance);
        }

        /// <summary>
        /// Determines if three vertices are collinear (ie. on a straight line)
        /// </summary>
        /// <param name="a">First vertex</param>
        /// <param name="b">Second vertex</param>
        /// <param name="c">Third vertex</param>
        /// <param name="tolerance">The tolerance</param>
        /// <returns></returns>
        public static bool IsCollinear(ref AetherVector2 a, ref AetherVector2 b, ref AetherVector2 c)
        {
            return IsCollinear(ref a, ref b, ref c, Fix64.Zero);
        }

        public static void Cross(Fix64 s, ref AetherVector2 a, out AetherVector2 b)
        {
            b.X = -s * a.Y;
            b.Y =  s * a.X;
        }

        public static bool FloatEquals(Fix64 value1, Fix64 value2)
        {
            return  Fix64.Abs(value1 - value2) <= Settings.Epsilon;
        }

        /// <summary>
        /// Checks if a floating point Value is equal to another,
        /// within a certain tolerance.
        /// </summary>
        /// <param name="value1">The first floating point Value.</param>
        /// <param name="value2">The second floating point Value.</param>
        /// <param name="delta">The floating point tolerance.</param>
        /// <returns>True if the values are "equal", false otherwise.</returns>
        public static bool FloatEquals(Fix64 value1, Fix64 value2, Fix64 delta)
        {
            return FloatInRange(value1, value2 - delta, value2 + delta);
        }

        /// <summary>
        /// Checks if a floating point Value is within a specified
        /// range of values (inclusive).
        /// </summary>
        /// <param name="value">The Value to check.</param>
        /// <param name="min">The minimum Value.</param>
        /// <param name="max">The maximum Value.</param>
        /// <returns>True if the Value is within the range specified,
        /// false otherwise.</returns>
        public static bool FloatInRange(Fix64 value, Fix64 min, Fix64 max)
        {
            return (value >= min && value <= max);
        }

        public static Fix64 Min(Fix64 value1, Fix64 value2)
        {
            if (value2 < value1)
            {
                return value2;
            }

            return value1;
        }

        public static Fix64 Max(Fix64 value1, Fix64 value2)
        {
            if (value2 > value1)
            {
                return value2;
            }

            return value1;
        }

        public static Fix64 Asin(Fix64 value)
        {
            return Fix64.Atan(value / Fix64.Sqrt(Fix64.One - (value * value)));
        }

        public static Fix64 Sign(Fix64 value)
        {
            if(value > Fix64.Zero)
            {
                return Fix64.One;
            }

            if (value < Fix64.Zero)
            {
                return -Fix64.One;
            }

            return Fix64.Zero;
        }
    }

    /// <summary>
    /// A 2-by-2 matrix. Stored in column-major order.
    /// </summary>
    public struct Mat22
    {
        public AetherVector2 ex, ey;

        /// <summary>
        /// Construct this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        public Mat22(AetherVector2 c1, AetherVector2 c2)
        {
            ex = c1;
            ey = c2;
        }

        /// <summary>
        /// Construct this matrix using scalars.
        /// </summary>
        /// <param name="a11">The a11.</param>
        /// <param name="a12">The a12.</param>
        /// <param name="a21">The a21.</param>
        /// <param name="a22">The a22.</param>
        public Mat22(Fix64 a11, Fix64 a12, Fix64 a21, Fix64 a22)
        {
            ex.X = a11;
            ex.Y = a21;
            ey.X = a12;
            ey.Y = a22;
        }

        public Mat22 Inverse
        {
            get
            {
                Fix64 a = ex.X, b = ey.X, c = ex.Y, d = ey.Y;
                Fix64 det = a * d - b * c;
                if (det != Fix64.Zero)
                {
                    det = Fix64.One / det;
                }

                Mat22 result;
                result.ex.X = det * d;
                result.ex.Y = -det * c;

                result.ey.X = -det * b;
                result.ey.Y = det * a;

                return result;
            }
        }

        /// <summary>
        /// Initialize this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        public void Set(AetherVector2 c1, AetherVector2 c2)
        {
            ex = c1;
            ey = c2;
        }

        /// <summary>
        /// Set this to the identity matrix.
        /// </summary>
        public void SetIdentity()
        {
            ex.X = Fix64.One;
            ey.X = Fix64.Zero;
            ex.Y = Fix64.Zero;
            ey.Y = Fix64.One;
        }

        /// <summary>
        /// Set this matrix to all zeros.
        /// </summary>
        public void SetZero()
        {
            ex.X = Fix64.Zero;
            ey.X = Fix64.Zero;
            ex.Y = Fix64.Zero;
            ey.Y = Fix64.Zero;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public AetherVector2 Solve(AetherVector2 b)
        {
            Fix64 a11 = ex.X, a12 = ey.X, a21 = ex.Y, a22 = ey.Y;
            Fix64 det = a11 * a22 - a12 * a21;
            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            return new AetherVector2(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
        }

        public static void Add(ref Mat22 A, ref Mat22 B, out Mat22 R)
        {
            R.ex = A.ex + B.ex;
            R.ey = A.ey + B.ey;
        }
    }

    /// <summary>
    /// A 3-by-3 matrix. Stored in column-major order.
    /// </summary>
    public struct Mat33
    {
        public AetherVector3 ex, ey, ez;

        /// <summary>
        /// Construct this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        /// <param name="c3">The c3.</param>
        public Mat33(AetherVector3 c1, AetherVector3 c2, AetherVector3 c3)
        {
            ex = c1;
            ey = c2;
            ez = c3;
        }

        /// <summary>
        /// Set this matrix to all zeros.
        /// </summary>
        public void SetZero()
        {
            ex = AetherVector3.Zero;
            ey = AetherVector3.Zero;
            ez = AetherVector3.Zero;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public AetherVector3 Solve33(AetherVector3 b)
        {
            Fix64 det = AetherVector3.Dot(ex, AetherVector3.Cross(ey, ez));
            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            return new AetherVector3(det * AetherVector3.Dot(b, AetherVector3.Cross(ey, ez)), det * AetherVector3.Dot(ex, AetherVector3.Cross(b, ez)), det * AetherVector3.Dot(ex, AetherVector3.Cross(ey, b)));
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases. Solve only the upper
        /// 2-by-2 matrix equation.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public AetherVector2 Solve22(AetherVector2 b)
        {
            Fix64 a11 = ex.X, a12 = ey.X, a21 = ex.Y, a22 = ey.Y;
            Fix64 det = a11 * a22 - a12 * a21;

            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            return new AetherVector2(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
        }

        /// Get the inverse of this matrix as a 2-by-2.
        /// Returns the zero matrix if singular.
        public void GetInverse22(ref Mat33 M)
        {
            Fix64 a = ex.X, b = ey.X, c = ex.Y, d = ey.Y;
            Fix64 det = a * d - b * c;
            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            M.ex.X = det * d; M.ey.X = -det * b; M.ex.Z = Fix64.Zero;
            M.ex.Y = -det * c; M.ey.Y = det * a; M.ey.Z = Fix64.Zero;
            M.ez.X = Fix64.Zero; M.ez.Y = Fix64.Zero; M.ez.Z = Fix64.Zero;
        }

        /// Get the symmetric inverse of this matrix as a 3-by-3.
        /// Returns the zero matrix if singular.
        public void GetSymInverse33(ref Mat33 M)
        {
            Fix64 det = MathUtils.Dot(ex, MathUtils.Cross(ref ey, ref ez));
            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            Fix64 a11 = ex.X, a12 = ey.X, a13 = ez.X;
            Fix64 a22 = ey.Y, a23 = ez.Y;
            Fix64 a33 = ez.Z;

            M.ex.X = det * (a22 * a33 - a23 * a23);
            M.ex.Y = det * (a13 * a23 - a12 * a33);
            M.ex.Z = det * (a12 * a23 - a13 * a22);

            M.ey.X = M.ex.Y;
            M.ey.Y = det * (a11 * a33 - a13 * a13);
            M.ey.Z = det * (a13 * a12 - a11 * a23);

            M.ez.X = M.ex.Z;
            M.ez.Y = M.ey.Z;
            M.ez.Z = det * (a11 * a22 - a12 * a12);
        }
    }

    
    /// <summary>
    /// A transform contains translation and rotation. It is used to represent
    /// the position and orientation of rigid frames.
    /// </summary>
    public struct Transform
    {
        private static readonly Transform _identity = new Transform(AetherVector2.Zero, Complex.One);

        public Complex q;
        public AetherVector2 p;

        public static Transform Identity { get { return _identity; } }

        /// <summary>
        /// Initialize using a position vector and a Complex rotation.
        /// </summary>
        /// <param name="position">The position.</param>
        /// <param name="rotation">The rotation</param>
        public Transform(AetherVector2 position, Complex rotation)
        {
            q = rotation;
            p = position;
        }

        /// <summary>
        /// Initialize using a position vector and a rotation.
        /// </summary>
        /// <param name="position">The position.</param>
        /// <param name="angle">The rotation angle</param>
        public Transform(AetherVector2 position, Fix64 angle)
            : this(position, Complex.FromAngle(angle))
        {
        }
                
        public static AetherVector2 Multiply(AetherVector2 left, ref Transform right)
        {
            return Multiply(ref left, ref right);
        }

        public static AetherVector2 Multiply(ref AetherVector2 left, ref Transform right)
        {
            // Opt: var result = Complex.Multiply(left, right.q) + right.p;
            return new AetherVector2(
                (left.X * right.q.R - left.Y * right.q.i) + right.p.X,
                (left.Y * right.q.R + left.X * right.q.i) + right.p.Y);
        }

        public static AetherVector2 Divide(AetherVector2 left, ref Transform right)
        {
            return Divide(ref left, ref right);
        }

        public static AetherVector2 Divide(ref AetherVector2 left, ref Transform right)
        {
            // Opt: var result = Complex.Divide(left - right.p, right);
            Fix64 px = left.X - right.p.X;
            Fix64 py = left.Y - right.p.Y;
            return new AetherVector2(
                (px * right.q.R + py * right.q.i),
                (py * right.q.R - px * right.q.i));
        }

        public static void Divide(AetherVector2 left, ref Transform right, out AetherVector2 result)
        {
            // Opt: var result = Complex.Divide(left - right.p, right);
            Fix64 px = left.X - right.p.X;
            Fix64 py = left.Y - right.p.Y;
            result.X = (px * right.q.R + py * right.q.i);
            result.Y = (py * right.q.R - px * right.q.i);
        }

        public static Transform Multiply(ref Transform left, ref Transform right)
        {
            return new Transform(
                    Complex.Multiply(ref left.p, ref right.q) + right.p,
                    Complex.Multiply(ref left.q, ref right.q));
        }

        public static Transform Divide(ref Transform left, ref Transform right)
        {
            return new Transform(
                Complex.Divide(left.p - right.p, ref right.q),
                Complex.Divide(ref left.q, ref right.q));
        }
        
        public static void Divide(ref Transform left, ref Transform right, out Transform result)
        {
            Complex.Divide(left.p - right.p, ref right.q, out result.p);
            Complex.Divide(ref left.q, ref right.q, out result.q);
        }
            
        public static void Multiply(ref Transform left, Complex right, out Transform result)
        {
            result.p = Complex.Multiply(ref left.p, ref right);
            result.q = Complex.Multiply(ref left.q, ref right);
        }
        
        public static void Divide(ref Transform left, Complex right, out Transform result)
        {
            result.p = Complex.Divide(ref left.p, ref right);
            result.q = Complex.Divide(ref left.q, ref right);
        }
    }

    /// <summary>
    /// This describes the motion of a body/shape for TOI computation.
    /// Shapes are defined with respect to the body origin, which may
    /// no coincide with the center of mass. However, to support dynamics
    /// we must interpolate the center of mass position.
    /// </summary>
    public struct Sweep
    {
        /// <summary>
        /// World angles
        /// </summary>
        public Fix64 A;

        public Fix64 A0;

        /// <summary>
        /// Fraction of the current time step in the range [0,1]
        /// c0 and a0 are the positions at alpha0.
        /// </summary>
        public Fix64 Alpha0;

        /// <summary>
        /// Center world positions
        /// </summary>
        public AetherVector2 C;

        public AetherVector2 C0;

        /// <summary>
        /// Local center of mass position
        /// </summary>
        public AetherVector2 LocalCenter;

        /// <summary>
        /// Get the interpolated transform at a specific time.
        /// </summary>
        /// <param name="xfb">The transform.</param>
        /// <param name="beta">beta is a factor in [0,1], where 0 indicates alpha0.</param>
        public void GetTransform(out Transform xfb, Fix64 beta)
        {
            xfb.p.X = (Fix64.One - beta) * C0.X + beta * C.X;
            xfb.p.Y = (Fix64.One - beta) * C0.Y + beta * C.Y;
            Fix64 angle = (Fix64.One - beta) * A0 + beta * A;
            xfb.q = Complex.FromAngle(angle);

            // Shift to origin
            xfb.p -= Complex.Multiply(ref LocalCenter, ref xfb.q);
        }

        /// <summary>
        /// Advance the sweep forward, yielding a new initial state.
        /// </summary>
        /// <param name="alpha">new initial time..</param>
        public void Advance(Fix64 alpha)
        {
            Debug.Assert(Alpha0 < Fix64.One);
            Fix64 beta = (alpha - Alpha0) / (Fix64.One - Alpha0);
            C0 += beta * (C - C0);
            A0 += beta * (A - A0);
            Alpha0 = alpha;
        }

        /// <summary>
        /// Normalize the angles.
        /// </summary>
        public void Normalize()
        {
            Fix64 d = Constant.Tau * Fix64.Floor(A0 / Constant.Tau);
            A0 -= d;
            A -= d;
        }
    }
}