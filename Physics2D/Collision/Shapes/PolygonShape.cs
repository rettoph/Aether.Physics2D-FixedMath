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
using tainicom.Aether.Physics2D.Common.ConvexHull;
#if XNAAPI
using Complex = tainicom.Aether.Physics2D.Common.Complex;
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Collision.Shapes
{
    /// <summary>
    /// Represents a simple non-selfintersecting convex polygon.
    /// Create a convex hull from the given array of points.
    /// </summary>
    public class PolygonShape : Shape
    {
        private Vertices _vertices;
        private Vertices _normals;

        /// <summary>
        /// Initializes a new instance of the <see cref="PolygonShape"/> class.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="density">The density.</param>
        public PolygonShape(Vertices vertices, Fix64 density)
            : base(density)
        {
            ShapeType = ShapeType.Polygon;
            _radius = Settings.PolygonRadius;

            Vertices = vertices;
        }

        /// <summary>
        /// Create a new PolygonShape with the specified density.
        /// </summary>
        /// <param name="density">The density.</param>
        public PolygonShape(Fix64 density)
            : base(density)
        {
            Debug.Assert(density >= Fix64.Zero);

            ShapeType = ShapeType.Polygon;
            _radius = Settings.PolygonRadius;
            _vertices = new Vertices(Settings.MaxPolygonVertices);
            _normals = new Vertices(Settings.MaxPolygonVertices);
        }

        internal PolygonShape()
            : base(Fix64.Zero)
        {
            ShapeType = ShapeType.Polygon;
            _radius = Settings.PolygonRadius;
            _vertices = new Vertices(Settings.MaxPolygonVertices);
            _normals = new Vertices(Settings.MaxPolygonVertices);
        }

        /// <summary>
        /// Create a convex hull from the given array of local points.
        /// The number of vertices must be in the range [3, Settings.MaxPolygonVertices].
        /// Warning: the points may be re-ordered, even if they form a convex polygon
        /// Warning: collinear points are handled but not removed. Collinear points may lead to poor stacking behavior.
        /// </summary>
        public Vertices Vertices
        {
            get { return _vertices; }
            set
            {
                _vertices = new Vertices(value);

                Debug.Assert(_vertices.Count >= 3 && _vertices.Count <= Settings.MaxPolygonVertices);

                if (Settings.UseConvexHullPolygons)
                {
                    //FPE note: This check is required as the GiftWrap algorithm early exits on triangles
                    //So instead of giftwrapping a triangle, we just force it to be clock wise.
                    if (_vertices.Count <= 3)
                        _vertices.ForceCounterClockWise();
                    else
                        _vertices = GiftWrap.GetConvexHull(_vertices);
                }

                _normals = new Vertices(_vertices.Count);

                // Compute normals. Ensure the edges have non-zero length.
                for (int i = 0; i < _vertices.Count; ++i)
                {
                    int next = i + 1 < _vertices.Count ? i + 1 : 0;
                    AetherVector2 edge = _vertices[next] - _vertices[i];
                    Debug.Assert(edge.LengthSquared() > Settings.Epsilon * Settings.Epsilon);

                    //FPE optimization: Normals.Add(MathUtils.Cross(edge, Fix64.One));
                    AetherVector2 temp = new AetherVector2(edge.Y, -edge.X);
                    temp.Normalize();
                    _normals.Add(temp);
                }

                // Compute the polygon mass data
                ComputeProperties();
            }
        }

        public Vertices Normals { get { return _normals; } }

        public override int ChildCount { get { return 1; } }

        protected override void ComputeProperties()
        {
            // Polygon mass, centroid, and inertia.
            // Let rho be the polygon density in mass per unit area.
            // Then:
            // mass = rho * int(dA)
            // centroid.X = (1/mass) * rho * int(x * dA)
            // centroid.Y = (1/mass) * rho * int(y * dA)
            // I = rho * int((x*x + y*y) * dA)
            //
            // We can compute these integrals by summing all the integrals
            // for each triangle of the polygon. To evaluate the integral
            // for a single triangle, we make a change of variables to
            // the (u,v) coordinates of the triangle:
            // x = x0 + e1x * u + e2x * v
            // y = y0 + e1y * u + e2y * v
            // where 0 <= u && 0 <= v && u + v <= 1.
            //
            // We integrate u from [0,1-v] and then v from [0,1].
            // We also need to use the Jacobian of the transformation:
            // D = cross(e1, e2)
            //
            // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
            //
            // The rest of the derivation is handled by computer algebra.

            Debug.Assert(Vertices.Count >= 3);

            //FPE optimization: Early exit as polygons with 0 density does not have any properties.
            if (_density <= Fix64.Zero)
                return;

            //FPE optimization: Consolidated the calculate centroid and mass code to a single method.
            AetherVector2 center = AetherVector2.Zero;
            Fix64 area = Fix64.Zero;
            Fix64 I = Fix64.Zero;

            // pRef is the reference point for forming triangles.
            // It's location doesn't change the result (except for rounding error).
            AetherVector2 s = AetherVector2.Zero;

            // This code would put the reference point inside the polygon.
            for (int i = 0; i < Vertices.Count; ++i)
            {
                s += Vertices[i];
            }
            s *= Fix64.One / (Fix64)Vertices.Count;

            for (int i = 0; i < Vertices.Count; ++i)
            {
                // Triangle vertices.
                AetherVector2 e1 = Vertices[i] - s;
                AetherVector2 e2 = i + 1 < Vertices.Count ? Vertices[i + 1] - s : Vertices[0] - s;

                Fix64 D = MathUtils.Cross(ref e1, ref e2);

                Fix64 triangleArea = Fix64Constants.PointFive * D;
                area += triangleArea;

                // Area weighted centroid
                center += triangleArea * Fix64Constants.k_inv3 * (e1 + e2);

                Fix64 ex1 = e1.X, ey1 = e1.Y;
                Fix64 ex2 = e2.X, ey2 = e2.Y;

                Fix64 intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
                Fix64 inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

                I += (Fix64Constants.PointTwoFive * Fix64Constants.k_inv3 * D) * (intx2 + inty2);
            }

            //The area is too small for the engine to handle.
            Debug.Assert(area > Settings.Epsilon);

            // We save the area
            MassData.Area = area;

            // Total mass
            MassData.Mass = _density * area;

            // Center of mass
            center *= Fix64.One / area;
            MassData.Centroid = center + s;

            // Inertia tensor relative to the local origin (point s).
            MassData.Inertia = _density * I;

            // Shift to center of mass then to original body origin.
            MassData.Inertia += MassData.Mass * (AetherVector2.Dot(MassData.Centroid, MassData.Centroid) - AetherVector2.Dot(center, center));
        }

        public override bool TestPoint(ref Transform transform, ref AetherVector2 point)
        {
            AetherVector2 pLocal = Complex.Divide(point - transform.p, ref transform.q);

            for (int i = 0; i < Vertices.Count; ++i)
            {
                Fix64 dot = AetherVector2.Dot(Normals[i], pLocal - Vertices[i]);
                if (dot > Fix64.Zero)
                {
                    return false;
                }
            }

            return true;
        }

        public override bool RayCast(out RayCastOutput output, ref RayCastInput input, ref Transform transform, int childIndex)
        {
            output = new RayCastOutput();

            // Put the ray into the polygon's frame of reference.
            AetherVector2 p1 = Complex.Divide(input.Point1 - transform.p, ref transform.q);
            AetherVector2 p2 = Complex.Divide(input.Point2 - transform.p, ref transform.q);
            AetherVector2 d = p2 - p1;

            Fix64 lower = Fix64.Zero, upper = input.MaxFraction;

            int index = -1;

            for (int i = 0; i < Vertices.Count; ++i)
            {
                // p = p1 + a * d
                // dot(normal, p - v) = 0
                // dot(normal, p1 - v) + a * dot(normal, d) = 0
                Fix64 numerator = AetherVector2.Dot(Normals[i], Vertices[i] - p1);
                Fix64 denominator = AetherVector2.Dot(Normals[i], d);

                if (denominator == Fix64.Zero)
                {
                    if (numerator < Fix64.Zero)
                    {
                        return false;
                    }
                }
                else
                {
                    // Note: we want this predicate without division:
                    // lower < numerator / denominator, where denominator < 0
                    // Since denominator < 0, we have to flip the inequality:
                    // lower < numerator / denominator <==> denominator * lower > numerator.
                    if (denominator < Fix64.Zero && numerator < lower * denominator)
                    {
                        // Increase lower.
                        // The segment enters this half-space.
                        lower = numerator / denominator;
                        index = i;
                    }
                    else if (denominator > Fix64.Zero && numerator < upper * denominator)
                    {
                        // Decrease upper.
                        // The segment exits this half-space.
                        upper = numerator / denominator;
                    }
                }

                // The use of epsilon here causes the assert on lower to trip
                // in some cases. Apparently the use of epsilon was to make edge
                // shapes work, but now those are handled separately.
                //if (upper < lower - b2_epsilon)
                if (upper < lower)
                {
                    return false;
                }
            }

            Debug.Assert(Fix64.Zero <= lower && lower <= input.MaxFraction);

            if (index >= 0)
            {
                output.Fraction = lower;
                output.Normal = Complex.Multiply(Normals[index], ref transform.q);
                return true;
            }

            return false;
        }

        /// <summary>
        /// Given a transform, compute the associated axis aligned bounding box for a child shape.
        /// </summary>
        /// <param name="aabb">The aabb results.</param>
        /// <param name="transform">The world transform of the shape.</param>
        /// <param name="childIndex">The child shape index.</param>
        public override void ComputeAABB(out AABB aabb, ref Transform transform, int childIndex)
        {
            // OPT: aabb.LowerBound = Transform.Multiply(Vertices[0], ref transform);
            var vert = Vertices[0];
            aabb.LowerBound.X = (vert.X * transform.q.R - vert.Y * transform.q.i) + transform.p.X;
            aabb.LowerBound.Y = (vert.Y * transform.q.R + vert.X * transform.q.i) + transform.p.Y;
            aabb.UpperBound = aabb.LowerBound;

            for (int i = 1; i < Vertices.Count; ++i)
            {
                // OPT: Vector2 v = Transform.Multiply(Vertices[i], ref transform);
                vert = Vertices[i];
                Fix64 vX = (vert.X * transform.q.R - vert.Y * transform.q.i) + transform.p.X;
                Fix64 vY = (vert.Y * transform.q.R + vert.X * transform.q.i) + transform.p.Y;

                // OPT: Vector2.Min(ref aabb.LowerBound, ref v, out aabb.LowerBound);
                // OPT: Vector2.Max(ref aabb.UpperBound, ref v, out aabb.UpperBound);
                Debug.Assert(aabb.LowerBound.X <= aabb.UpperBound.X);
                if (vX < aabb.LowerBound.X) aabb.LowerBound.X = vX;
                else if (vX > aabb.UpperBound.X) aabb.UpperBound.X = vX;
                Debug.Assert(aabb.LowerBound.Y <= aabb.UpperBound.Y);
                if (vY < aabb.LowerBound.Y) aabb.LowerBound.Y = vY;
                else if (vY > aabb.UpperBound.Y) aabb.UpperBound.Y = vY;
            }

            // OPT: Vector2 r = new Vector2(Radius, Radius);
            // OPT: aabb.LowerBound = aabb.LowerBound - r;
            // OPT: aabb.UpperBound = aabb.UpperBound + r;
            aabb.LowerBound.X -= Radius;
            aabb.LowerBound.Y -= Radius;
            aabb.UpperBound.X += Radius;
            aabb.UpperBound.Y += Radius;
        }

        public override Fix64 ComputeSubmergedArea(ref AetherVector2 normal, Fix64 offset, ref Transform xf, out AetherVector2 sc)
        {
            sc = AetherVector2.Zero;

            //Transform plane into shape co-ordinates
            AetherVector2 normalL = Complex.Divide(ref normal, ref xf.q);
            Fix64 offsetL = offset - AetherVector2.Dot(normal, xf.p);

            Fix64[] depths = new Fix64[Settings.MaxPolygonVertices];
            int diveCount = 0;
            int intoIndex = -1;
            int outoIndex = -1;

            bool lastSubmerged = false;
            int i;
            for (i = 0; i < Vertices.Count; i++)
            {
                depths[i] = AetherVector2.Dot(normalL, Vertices[i]) - offsetL;
                bool isSubmerged = depths[i] < -Settings.Epsilon;
                if (i > 0)
                {
                    if (isSubmerged)
                    {
                        if (!lastSubmerged)
                        {
                            intoIndex = i - 1;
                            diveCount++;
                        }
                    }
                    else
                    {
                        if (lastSubmerged)
                        {
                            outoIndex = i - 1;
                            diveCount++;
                        }
                    }
                }
                lastSubmerged = isSubmerged;
            }
            switch (diveCount)
            {
                case 0:
                    if (lastSubmerged)
                    {
                        //Completely submerged
                        sc = Transform.Multiply(MassData.Centroid, ref xf);
                        return MassData.Mass / Density;
                    }

                    //Completely dry
                    return Fix64.Zero;
                case 1:
                    if (intoIndex == -1)
                    {
                        intoIndex = Vertices.Count - 1;
                    }
                    else
                    {
                        outoIndex = Vertices.Count - 1;
                    }
                    break;
            }

            int intoIndex2 = (intoIndex + 1) % Vertices.Count;
            int outoIndex2 = (outoIndex + 1) % Vertices.Count;

            Fix64 intoLambda = (Fix64.Zero - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
            Fix64 outoLambda = (Fix64.Zero - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);

            AetherVector2 intoVec = new AetherVector2(Vertices[intoIndex].X * (Fix64.One - intoLambda) + Vertices[intoIndex2].X * intoLambda, Vertices[intoIndex].Y * (Fix64.One - intoLambda) + Vertices[intoIndex2].Y * intoLambda);
            AetherVector2 outoVec = new AetherVector2(Vertices[outoIndex].X * (Fix64.One - outoLambda) + Vertices[outoIndex2].X * outoLambda, Vertices[outoIndex].Y * (Fix64.One - outoLambda) + Vertices[outoIndex2].Y * outoLambda);

            //Initialize accumulator
            Fix64 area = Fix64.Zero;
            AetherVector2 center = new AetherVector2(Fix64.Zero, Fix64.Zero);
            AetherVector2 p2 = Vertices[intoIndex2];

            //An awkward loop from intoIndex2+1 to outIndex2
            i = intoIndex2;
            while (i != outoIndex2)
            {
                i = (i + 1) % Vertices.Count;
                AetherVector2 p3;
                if (i == outoIndex2)
                    p3 = outoVec;
                else
                    p3 = Vertices[i];
                //Add the triangle formed by intoVec,p2,p3
                {
                    AetherVector2 e1 = p2 - intoVec;
                    AetherVector2 e2 = p3 - intoVec;

                    Fix64 D = MathUtils.Cross(ref e1, ref e2);

                    Fix64 triangleArea = Fix64Constants.PointFive * D;

                    area += triangleArea;

                    // Area weighted centroid
                    center += triangleArea * Fix64Constants.k_inv3 * (intoVec + p2 + p3);
                }

                p2 = p3;
            }

            //Normalize and transform centroid
            center *= Fix64.One / area;

            sc = Transform.Multiply(ref center, ref xf);

            return area;
        }

        public bool CompareTo(PolygonShape shape)
        {
            if (Vertices.Count != shape.Vertices.Count)
                return false;

            for (int i = 0; i < Vertices.Count; i++)
            {
                if (Vertices[i] != shape.Vertices[i])
                    return false;
            }

            return (Radius == shape.Radius && MassData == shape.MassData);
        }

        public override Shape Clone()
        {
            PolygonShape clone = new PolygonShape();
            clone.ShapeType = ShapeType;
            clone._radius = _radius;
            clone._density = _density;
            clone._vertices = new Vertices(_vertices);
            clone._normals = new Vertices(_normals);
            clone.MassData = MassData;
            return clone;
        }
    }
}