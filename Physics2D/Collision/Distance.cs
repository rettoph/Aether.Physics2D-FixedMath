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
using tainicom.Aether.Physics2D.Collision.Shapes;
using tainicom.Aether.Physics2D.Common;
#if XNAAPI
using Complex = tainicom.Aether.Physics2D.Common.Complex;
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Collision
{
    /// <summary>
    /// A distance proxy is used by the GJK algorithm.
    /// It encapsulates any shape.
    /// </summary>
    public struct DistanceProxy
    {
        internal Fix64 Radius;
        internal Vertices Vertices;

        // GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.

        /// <summary>
        /// Initialize the proxy using the given shape. The shape
        /// must remain in scope while the proxy is in use.
        /// </summary>
        /// <param name="shape">The shape.</param>
        /// <param name="index">The index.</param>
        public DistanceProxy(Shape shape, int index)
        {
            Vertices = new Vertices();

            switch (shape.ShapeType)
            {
                case ShapeType.Circle:
                    {
                        CircleShape circle = (CircleShape)shape;
                        Vertices.Clear();
                        Vertices.Add(circle.Position);
                        Radius = circle.Radius;
                    }
                    break;

                case ShapeType.Polygon:
                    {
                        PolygonShape polygon = (PolygonShape)shape;
                        Vertices.Clear();
                        for (int i = 0; i < polygon.Vertices.Count; i++)
                        {
                            Vertices.Add(polygon.Vertices[i]);
                        }
                        Radius = polygon.Radius;
                    }
                    break;

                case ShapeType.Chain:
                    {
                        ChainShape chain = (ChainShape)shape;
                        Debug.Assert(0 <= index && index < chain.Vertices.Count);
                        Vertices.Clear();
                        Vertices.Add(chain.Vertices[index]);
                        Vertices.Add(index + 1 < chain.Vertices.Count ? chain.Vertices[index + 1] : chain.Vertices[0]);

                        Radius = chain.Radius;
                    }
                    break;

                case ShapeType.Edge:
                    {
                        EdgeShape edge = (EdgeShape)shape;
                        Vertices.Clear();
                        Vertices.Add(edge.Vertex1);
                        Vertices.Add(edge.Vertex2);
                        Radius = edge.Radius;
                    }
                    break;

                default:
                    Radius = Fix64.Zero;
                    Debug.Assert(false);
                    break;
            }
        }

        /// <summary>
        /// Get the supporting vertex index in the given direction.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <returns></returns>
        public int GetSupport(AetherVector2 direction)
        {
            int bestIndex = 0;
            Fix64 bestValue = AetherVector2.Dot(Vertices[0], direction);
            for (int i = 1; i < Vertices.Count; ++i)
            {
                Fix64 value = AetherVector2.Dot(Vertices[i], direction);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return bestIndex;
        }

        /// <summary>
        /// Get the supporting vertex in the given direction.
        /// </summary>
        /// <param name="direction">The direction.</param>
        /// <returns></returns>
        public AetherVector2 GetSupportVertex(AetherVector2 direction)
        {
            int bestIndex = 0;
            Fix64 bestValue = AetherVector2.Dot(Vertices[0], direction);
            for (int i = 1; i < Vertices.Count; ++i)
            {
                Fix64 value = AetherVector2.Dot(Vertices[i], direction);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return Vertices[bestIndex];
        }
    }

    /// <summary>
    /// Used to warm start ComputeDistance.
    /// Set count to zero on first call.
    /// </summary>
    public struct SimplexCache
    {
        /// <summary>
        /// Length or area
        /// </summary>
        public ushort Count;

        /// <summary>
        /// Vertices on shape A
        /// </summary>
        public FixedArray3<byte> IndexA;

        /// <summary>
        /// Vertices on shape B
        /// </summary>
        public FixedArray3<byte> IndexB;

        public Fix64 Metric;
    }

    /// <summary>
    /// Input for Distance.ComputeDistance().
    /// You have to option to use the shape radii in the computation. 
    /// </summary>
    public struct DistanceInput
    {
        public DistanceProxy ProxyA;
        public DistanceProxy ProxyB;
        public Transform TransformA;
        public Transform TransformB;
        public bool UseRadii;
    }

    /// <summary>
    /// Output for Distance.ComputeDistance().
    /// </summary>
    public struct DistanceOutput
    {
        public Fix64 Distance;

        /// <summary>
        /// Number of GJK iterations used
        /// </summary>
        public int Iterations;

        /// <summary>
        /// Closest point on shapeA
        /// </summary>
        public AetherVector2 PointA;

        /// <summary>
        /// Closest point on shapeB
        /// </summary>
        public AetherVector2 PointB;
    }

    internal struct SimplexVertex
    {
        /// <summary>
        /// Barycentric coordinate for closest point 
        /// </summary>
        public Fix64 A;

        /// <summary>
        /// wA index
        /// </summary>
        public int IndexA;

        /// <summary>
        /// wB index
        /// </summary>
        public int IndexB;

        /// <summary>
        /// wB - wA
        /// </summary>
        public AetherVector2 W;

        /// <summary>
        /// Support point in proxyA
        /// </summary>
        public AetherVector2 WA;

        /// <summary>
        /// Support point in proxyB
        /// </summary>
        public AetherVector2 WB;
    }

    internal struct Simplex
    {
        internal int Count;
        internal FixedArray3<SimplexVertex> V;

        internal void ReadCache(ref SimplexCache cache, ref DistanceProxy proxyA, ref Transform transformA, ref DistanceProxy proxyB, ref Transform transformB)
        {
            Debug.Assert(cache.Count <= 3);

            // Copy data from cache.
            Count = cache.Count;
            for (int i = 0; i < Count; ++i)
            {
                SimplexVertex v = V[i];
                v.IndexA = cache.IndexA[i];
                v.IndexB = cache.IndexB[i];
                AetherVector2 wALocal = proxyA.Vertices[v.IndexA];
                AetherVector2 wBLocal = proxyB.Vertices[v.IndexB];
                v.WA = Transform.Multiply(ref wALocal, ref transformA);
                v.WB = Transform.Multiply(ref wBLocal, ref transformB);
                v.W = v.WB - v.WA;
                v.A = Fix64.Zero;
                V[i] = v;
            }

            // Compute the new simplex metric, if it is substantially different than
            // old metric then flush the simplex.
            if (Count > 1)
            {
                Fix64 metric1 = cache.Metric;
                Fix64 metric2 = GetMetric();
                if (metric2 < Fix64Constants.PointFive * metric1 || Fix64Constants.Two * metric1 < metric2 || metric2 < Settings.Epsilon)
                {
                    // Reset the simplex.
                    Count = 0;
                }
            }

            // If the cache is empty or invalid ...
            if (Count == 0)
            {
                SimplexVertex v = V[0];
                v.IndexA = 0;
                v.IndexB = 0;
                AetherVector2 wALocal = proxyA.Vertices[0];
                AetherVector2 wBLocal = proxyB.Vertices[0];
                v.WA = Transform.Multiply(ref wALocal, ref transformA);
                v.WB = Transform.Multiply(ref wBLocal, ref transformB);
                v.W = v.WB - v.WA;
                v.A = Fix64.One;
                V[0] = v;
                Count = 1;
            }
        }

        internal void WriteCache(ref SimplexCache cache)
        {
            cache.Metric = GetMetric();
            cache.Count = (UInt16)Count;
            for (int i = 0; i < Count; ++i)
            {
                cache.IndexA[i] = (byte)(V[i].IndexA);
                cache.IndexB[i] = (byte)(V[i].IndexB);
            }
        }

        internal AetherVector2 GetSearchDirection()
        {
            switch (Count)
            {
                case 1:
                    return -V[0].W;

                case 2:
                    {
                        AetherVector2 e12 = V[1].W - V[0].W;
                        Fix64 sgn = MathUtils.Cross(e12, -V[0].W);
                        if (sgn > Fix64.Zero)
                        {
                            // Origin is left of e12.
                            return new AetherVector2(-e12.Y, e12.X);
                        }
                        else
                        {
                            // Origin is right of e12.
                            return new AetherVector2(e12.Y, -e12.X);
                        }
                    }

                default:
                    Debug.Assert(false);
                    return AetherVector2.Zero;
            }
        }

        internal AetherVector2 GetClosestPoint()
        {
            switch (Count)
            {
                case 0:
                    Debug.Assert(false);
                    return AetherVector2.Zero;

                case 1:
                    return V[0].W;

                case 2:
                    return V[0].A * V[0].W + V[1].A * V[1].W;

                case 3:
                    return AetherVector2.Zero;

                default:
                    Debug.Assert(false);
                    return AetherVector2.Zero;
            }
        }

        internal void GetWitnessPoints(out AetherVector2 pA, out AetherVector2 pB)
        {
            switch (Count)
            {
                case 0:
                    pA = AetherVector2.Zero;
                    pB = AetherVector2.Zero;
                    Debug.Assert(false);
                    break;

                case 1:
                    pA = V[0].WA;
                    pB = V[0].WB;
                    break;

                case 2:
                    pA = V[0].A * V[0].WA + V[1].A * V[1].WA;
                    pB = V[0].A * V[0].WB + V[1].A * V[1].WB;
                    break;

                case 3:
                    pA = V[0].A * V[0].WA + V[1].A * V[1].WA + V[2].A * V[2].WA;
                    pB = pA;
                    break;

                default:
                    throw new Exception();
            }
        }

        internal Fix64 GetMetric()
        {
            switch (Count)
            {
                case 0:
                    Debug.Assert(false);
                    return Fix64.Zero;
                case 1:
                    return Fix64.Zero;

                case 2:
                    return (V[0].W - V[1].W).Length();

                case 3:
                    return MathUtils.Cross(V[1].W - V[0].W, V[2].W - V[0].W);

                default:
                    Debug.Assert(false);
                    return Fix64.Zero;
            }
        }

        // Solve a line segment using barycentric coordinates.
        //
        // p = a1 * w1 + a2 * w2
        // a1 + a2 = 1
        //
        // The vector from the origin to the closest point on the line is
        // perpendicular to the line.
        // e12 = w2 - w1
        // dot(p, e) = 0
        // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
        //
        // 2-by-2 linear system
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        //
        // Define
        // d12_1 =  dot(w2, e12)
        // d12_2 = -dot(w1, e12)
        // d12 = d12_1 + d12_2
        //
        // Solution
        // a1 = d12_1 / d12
        // a2 = d12_2 / d12

        internal void Solve2()
        {
            AetherVector2 w1 = V[0].W;
            AetherVector2 w2 = V[1].W;
            AetherVector2 e12 = w2 - w1;

            // w1 region
            Fix64 d12_2 = -AetherVector2.Dot(w1, e12);
            if (d12_2 <= Fix64.Zero)
            {
                // a2 <= 0, so we clamp it to 0
                SimplexVertex v0 = V[0];
                v0.A = Fix64.One;
                V[0] = v0;
                Count = 1;
                return;
            }

            // w2 region
            Fix64 d12_1 = AetherVector2.Dot(w2, e12);
            if (d12_1 <= Fix64.Zero)
            {
                // a1 <= 0, so we clamp it to 0
                SimplexVertex v1 = V[1];
                v1.A = Fix64.One;
                V[1] = v1;
                Count = 1;
                V[0] = V[1];
                return;
            }

            // Must be in e12 region.
            Fix64 inv_d12 = Fix64.One / (d12_1 + d12_2);
            SimplexVertex v0_2 = V[0];
            SimplexVertex v1_2 = V[1];
            v0_2.A = d12_1 * inv_d12;
            v1_2.A = d12_2 * inv_d12;
            V[0] = v0_2;
            V[1] = v1_2;
            Count = 2;
        }

        // Possible regions:
        // - points[2]
        // - edge points[0]-points[2]
        // - edge points[1]-points[2]
        // - inside the triangle
        internal void Solve3()
        {
            AetherVector2 w1 = V[0].W;
            AetherVector2 w2 = V[1].W;
            AetherVector2 w3 = V[2].W;

            // Edge12
            // [1      1     ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            // a3 = 0
            AetherVector2 e12 = w2 - w1;
            Fix64 w1e12 = AetherVector2.Dot(w1, e12);
            Fix64 w2e12 = AetherVector2.Dot(w2, e12);
            Fix64 d12_1 = w2e12;
            Fix64 d12_2 = -w1e12;

            // Edge13
            // [1      1     ][a1] = [1]
            // [w1.e13 w3.e13][a3] = [0]
            // a2 = 0
            AetherVector2 e13 = w3 - w1;
            Fix64 w1e13 = AetherVector2.Dot(w1, e13);
            Fix64 w3e13 = AetherVector2.Dot(w3, e13);
            Fix64 d13_1 = w3e13;
            Fix64 d13_2 = -w1e13;

            // Edge23
            // [1      1     ][a2] = [1]
            // [w2.e23 w3.e23][a3] = [0]
            // a1 = 0
            AetherVector2 e23 = w3 - w2;
            Fix64 w2e23 = AetherVector2.Dot(w2, e23);
            Fix64 w3e23 = AetherVector2.Dot(w3, e23);
            Fix64 d23_1 = w3e23;
            Fix64 d23_2 = -w2e23;

            // Triangle123
            Fix64 n123 = MathUtils.Cross(ref e12, ref e13);

            Fix64 d123_1 = n123 * MathUtils.Cross(ref w2, ref w3);
            Fix64 d123_2 = n123 * MathUtils.Cross(ref w3, ref w1);
            Fix64 d123_3 = n123 * MathUtils.Cross(ref w1, ref w2);

            // w1 region
            if (d12_2 <= Fix64.Zero && d13_2 <= Fix64.Zero)
            {
                SimplexVertex v0_1 = V[0];
                v0_1.A = Fix64.One;
                V[0] = v0_1;
                Count = 1;
                return;
            }

            // e12
            if (d12_1 > Fix64.Zero && d12_2 > Fix64.Zero && d123_3 <= Fix64.Zero)
            {
                Fix64 inv_d12 = Fix64.One / (d12_1 + d12_2);
                SimplexVertex v0_2 = V[0];
                SimplexVertex v1_2 = V[1];
                v0_2.A = d12_1 * inv_d12;
                v1_2.A = d12_2 * inv_d12;
                V[0] = v0_2;
                V[1] = v1_2;
                Count = 2;
                return;
            }

            // e13
            if (d13_1 > Fix64.Zero && d13_2 > Fix64.Zero && d123_2 <= Fix64.Zero)
            {
                Fix64 inv_d13 = Fix64.One / (d13_1 + d13_2);
                SimplexVertex v0_3 = V[0];
                SimplexVertex v2_3 = V[2];
                v0_3.A = d13_1 * inv_d13;
                v2_3.A = d13_2 * inv_d13;
                V[0] = v0_3;
                V[2] = v2_3;
                Count = 2;
                V[1] = V[2];
                return;
            }

            // w2 region
            if (d12_1 <= Fix64.Zero && d23_2 <= Fix64.Zero)
            {
                SimplexVertex v1_4 = V[1];
                v1_4.A = Fix64.One;
                V[1] = v1_4;
                Count = 1;
                V[0] = V[1];
                return;
            }

            // w3 region
            if (d13_1 <= Fix64.Zero && d23_1 <= Fix64.Zero)
            {
                SimplexVertex v2_5 = V[2];
                v2_5.A = Fix64.One;
                V[2] = v2_5;
                Count = 1;
                V[0] = V[2];
                return;
            }

            // e23
            if (d23_1 > Fix64.Zero && d23_2 > Fix64.Zero && d123_1 <= Fix64.Zero)
            {
                Fix64 inv_d23 = Fix64.One / (d23_1 + d23_2);
                SimplexVertex v1_6 = V[1];
                SimplexVertex v2_6 = V[2];
                v1_6.A = d23_1 * inv_d23;
                v2_6.A = d23_2 * inv_d23;
                V[1] = v1_6;
                V[2] = v2_6;
                Count = 2;
                V[0] = V[2];
                return;
            }

            // Must be in triangle123
            Fix64 inv_d123 = Fix64.One / (d123_1 + d123_2 + d123_3);
            SimplexVertex v0_7 = V[0];
            SimplexVertex v1_7 = V[1];
            SimplexVertex v2_7 = V[2];
            v0_7.A = d123_1 * inv_d123;
            v1_7.A = d123_2 * inv_d123;
            v2_7.A = d123_3 * inv_d123;
            V[0] = v0_7;
            V[1] = v1_7;
            V[2] = v2_7;
            Count = 3;
        }
    }

    /// <summary>
    /// The Gilbert–Johnson–Keerthi distance algorithm that provides the distance between shapes.
    /// </summary>
    public static class Distance
    {
        /// <summary>
        /// The number of calls made to the ComputeDistance() function.
        /// Note: This is only activated when Settings.EnableDiagnostics = true
        /// </summary>
        [ThreadStatic]
        public static int GJKCalls;

        /// <summary>
        /// The number of iterations that was made on the last call to ComputeDistance().
        /// Note: This is only activated when Settings.EnableDiagnostics = true
        /// </summary>
        [ThreadStatic]
        public static int GJKIters;

        /// <summary>
        /// The maximum numer of iterations ever mae with calls to the CompteDistance() funtion.
        /// Note: This is only activated when Settings.EnableDiagnostics = true
        /// </summary>
        [ThreadStatic]
        public static int GJKMaxIters;

        public static void ComputeDistance(out DistanceOutput output, out SimplexCache cache, DistanceInput input)
        {
            cache = new SimplexCache();

            if (Settings.EnableDiagnostics) //FPE: We only gather diagnostics when enabled
                ++GJKCalls;

            // Initialize the simplex.
            Simplex simplex = new Simplex();
            simplex.ReadCache(ref cache, ref input.ProxyA, ref input.TransformA, ref input.ProxyB, ref input.TransformB);

            // These store the vertices of the last simplex so that we
            // can check for duplicates and prevent cycling.
            FixedArray3<int> saveA = new FixedArray3<int>();
            FixedArray3<int> saveB = new FixedArray3<int>();

            //Fix64 distanceSqr1 = Settings.MaxFloat;

            // Main iteration loop.
            int iter = 0;
            while (iter < Settings.MaxGJKIterations)
            {
                // Copy simplex so we can identify duplicates.
                int saveCount = simplex.Count;
                for (int i = 0; i < saveCount; ++i)
                {
                    saveA[i] = simplex.V[i].IndexA;
                    saveB[i] = simplex.V[i].IndexB;
                }

                switch (simplex.Count)
                {
                    case 1:
                        break;
                    case 2:
                        simplex.Solve2();
                        break;
                    case 3:
                        simplex.Solve3();
                        break;
                    default:
                        Debug.Assert(false);
                        break;
                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                {
                    break;
                }

                //FPE: This code was not used anyway.
                // Compute closest point.
                //Vector2 p = simplex.GetClosestPoint();
                //Fix64 distanceSqr2 = p.LengthSquared();

                // Ensure progress
                //if (distanceSqr2 >= distanceSqr1)
                //{
                //break;
                //}
                //distanceSqr1 = distanceSqr2;

                // Get search direction.
                AetherVector2 d = simplex.GetSearchDirection();

                // Ensure the search direction is numerically fit.
                if (d.LengthSquared() < Settings.Epsilon * Settings.Epsilon)
                {
                    // The origin is probably contained by a line segment
                    // or triangle. Thus the shapes are overlapped.

                    // We can't return zero here even though there may be overlap.
                    // In case the simplex is a point, segment, or triangle it is difficult
                    // to determine if the origin is contained in the CSO or very close to it.
                    break;
                }

                // Compute a tentative new simplex vertex using support points.
                SimplexVertex vertex = simplex.V[simplex.Count];
                vertex.IndexA = input.ProxyA.GetSupport(-Complex.Divide(ref d, ref input.TransformA.q));
                vertex.WA = Transform.Multiply(input.ProxyA.Vertices[vertex.IndexA], ref input.TransformA);

                vertex.IndexB = input.ProxyB.GetSupport( Complex.Divide(ref d, ref input.TransformB.q));
                vertex.WB = Transform.Multiply(input.ProxyB.Vertices[vertex.IndexB], ref input.TransformB);
                vertex.W = vertex.WB - vertex.WA;
                simplex.V[simplex.Count] = vertex;

                // Iteration count is equated to the number of support point calls.
                ++iter;

                if (Settings.EnableDiagnostics) //FPE: We only gather diagnostics when enabled
                    ++GJKIters;

                // Check for duplicate support points. This is the main termination criteria.
                bool duplicate = false;
                for (int i = 0; i < saveCount; ++i)
                {
                    if (vertex.IndexA == saveA[i] && vertex.IndexB == saveB[i])
                    {
                        duplicate = true;
                        break;
                    }
                }

                // If we found a duplicate support point we must exit to avoid cycling.
                if (duplicate)
                {
                    break;
                }

                // New vertex is ok and needed.
                ++simplex.Count;
            }

            if (Settings.EnableDiagnostics) //FPE: We only gather diagnostics when enabled
                GJKMaxIters = Math.Max(GJKMaxIters, iter);

            // Prepare output.
            simplex.GetWitnessPoints(out output.PointA, out output.PointB);
            output.Distance = (output.PointA - output.PointB).Length();
            output.Iterations = iter;

            // Cache the simplex.
            simplex.WriteCache(ref cache);

            // Apply radii if requested.
            if (input.UseRadii)
            {
                Fix64 rA = input.ProxyA.Radius;
                Fix64 rB = input.ProxyB.Radius;

                if (output.Distance > rA + rB && output.Distance > Settings.Epsilon)
                {
                    // Shapes are still no overlapped.
                    // Move the witness points to the outer surface.
                    output.Distance -= rA + rB;
                    AetherVector2 normal = output.PointB - output.PointA;
                    normal.Normalize();
                    output.PointA += rA * normal;
                    output.PointB -= rB * normal;
                }
                else
                {
                    // Shapes are overlapped when radii are considered.
                    // Move the witness points to the middle.
                    AetherVector2 p = Fix64Constants.PointFive * (output.PointA + output.PointB);
                    output.PointA = p;
                    output.PointB = p;
                    output.Distance = Fix64.Zero;
                }
            }
        }
    }
}