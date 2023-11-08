// Copyright (c) 2017 Kastellanos Nikolaos

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
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using tainicom.Aether.Physics2D.Collision.Shapes;
using tainicom.Aether.Physics2D.Common;
#if XNAAPI
using Complex = tainicom.Aether.Physics2D.Common.Complex;
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Collision
{
    internal enum ContactFeatureType : byte
    {
        Vertex = 0,
        Face = 1,
    }

    /// <summary>
    /// The features that intersect to form the contact point
    /// This must be 4 bytes or less.
    /// </summary>
    public struct ContactFeature
    {
        /// <summary>
        /// Feature index on ShapeA
        /// </summary>
        public byte IndexA;

        /// <summary>
        /// Feature index on ShapeB
        /// </summary>
        public byte IndexB;

        /// <summary>
        /// The feature type on ShapeA
        /// </summary>
        public byte TypeA;

        /// <summary>
        /// The feature type on ShapeB
        /// </summary>
        public byte TypeB;
    }

    /// <summary>
    /// Contact ids to facilitate warm starting.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public struct ContactID
    {
        /// <summary>
        /// The features that intersect to form the contact point
        /// </summary>
        [FieldOffset(0)]
        public ContactFeature Features;

        /// <summary>
        /// Used to quickly compare contact ids.
        /// </summary>
        [FieldOffset(0)]
        public uint Key;
    }

    /// <summary>
    /// A manifold point is a contact point belonging to a contact
    /// manifold. It holds details related to the geometry and dynamics
    /// of the contact points.
    /// The local point usage depends on the manifold type:
    /// -ShapeType.Circles: the local center of circleB
    /// -SeparationFunction.FaceA: the local center of cirlceB or the clip point of polygonB
    /// -SeparationFunction.FaceB: the clip point of polygonA
    /// This structure is stored across time steps, so we keep it small.
    /// Note: the impulses are used for internal caching and may not
    /// provide reliable contact forces, especially for high speed collisions.
    /// </summary>
    public struct ManifoldPoint
    {
        /// <summary>
        /// Uniquely identifies a contact point between two Shapes
        /// </summary>
        public ContactID Id;

        /// <summary>
        /// Usage depends on manifold type
        /// </summary>
        public AetherVector2 LocalPoint;

        /// <summary>
        /// The non-penetration impulse
        /// </summary>
        public Fix64 NormalImpulse;

        /// <summary>
        /// The friction impulse
        /// </summary>
        public Fix64 TangentImpulse;
    }

    public enum ManifoldType
    {
        Circles,
        FaceA,
        FaceB
    }

    /// <summary>
    /// A manifold for two touching convex Shapes.
    /// Box2D supports multiple types of contact:
    /// - Clip point versus plane with radius
    /// - Point versus point with radius (circles)
    /// The local point usage depends on the manifold type:
    /// - ShapeType.Circles: the local center of circleA
    /// - SeparationFunction.FaceA: the center of faceA
    /// - SeparationFunction.FaceB: the center of faceB
    /// Similarly the local normal usage:
    /// - ShapeType.Circles: not used
    /// - SeparationFunction.FaceA: the normal on polygonA
    /// - SeparationFunction.FaceB: the normal on polygonB
    /// We store contacts in this way so that position correction can
    /// account for movement, which is critical for continuous physics.
    /// All contact scenarios must be expressed in one of these types.
    /// This structure is stored across time steps, so we keep it small.
    /// </summary>
    public struct Manifold
    {
        /// <summary>
        /// Not use for Type.SeparationFunction.Points
        /// </summary>
        public AetherVector2 LocalNormal;

        /// <summary>
        /// Usage depends on manifold type
        /// </summary>
        public AetherVector2 LocalPoint;

        /// <summary>
        /// The number of manifold points
        /// </summary>
        public int PointCount;

        /// <summary>
        /// The points of contact
        /// </summary>
        public FixedArray2<ManifoldPoint> Points;

        public ManifoldType Type;
    }

    /// <summary>
    /// This is used for determining the state of contact points.
    /// </summary>
    public enum PointState
    {
        /// <summary>
        /// Point does not exist
        /// </summary>
        Null,

        /// <summary>
        /// Point was added in the update
        /// </summary>
        Add,

        /// <summary>
        /// Point persisted across the update
        /// </summary>
        Persist,

        /// <summary>
        /// Point was removed in the update
        /// </summary>
        Remove,
    }

    /// <summary>
    /// Used for computing contact manifolds.
    /// </summary>
    public struct ClipVertex
    {
        public ContactID ID;
        public AetherVector2 V;
    }

    /// <summary>
    /// Ray-cast input data.
    /// </summary>
    public struct RayCastInput
    {
        /// <summary>
        /// The ray extends from p1 to p1 + maxFraction * (p2 - p1).
        /// If you supply a max fraction of 1, the ray extends from p1 to p2.
        /// A max fraction of 0.5 makes the ray go from p1 and half way to p2.
        /// </summary>
        public Fix64 MaxFraction;

        /// <summary>
        /// The starting point of the ray.
        /// </summary>
        public AetherVector2 Point1;

        /// <summary>
        /// The ending point of the ray.
        /// </summary>
        public AetherVector2 Point2;
    }

    /// <summary>
    /// Ray-cast output data. 
    /// </summary>
    public struct RayCastOutput
    {
        /// <summary>
        /// The ray hits at p1 + fraction * (p2 - p1), where p1 and p2 come from RayCastInput.
        /// Contains the actual fraction of the ray where it has the intersection point.
        /// </summary>
        public Fix64 Fraction;

        /// <summary>
        /// The normal of the face of the shape the ray has hit.
        /// </summary>
        public AetherVector2 Normal;
    }

    /// <summary>
    /// An axis aligned bounding box.
    /// </summary>
    public struct AABB
    {
        /// <summary>
        /// The lower vertex
        /// </summary>
        public AetherVector2 LowerBound;

        /// <summary>
        /// The upper vertex
        /// </summary>
        public AetherVector2 UpperBound;

        public AABB(AetherVector2 min, AetherVector2 max)
            : this(ref min, ref max)
        {
        }

        public AABB(ref AetherVector2 min, ref AetherVector2 max)
        {
            LowerBound = min;
            UpperBound = max;
        }

        public AABB(AetherVector2 center, Fix64 width, Fix64 height)
        {
            LowerBound = center - new AetherVector2(width / Fix64Constants.Two, height / Fix64Constants.Two);
            UpperBound = center + new AetherVector2(width / Fix64Constants.Two, height / Fix64Constants.Two);
        }

        public Fix64 Width
        {
            get { return UpperBound.X - LowerBound.X; }
        }

        public Fix64 Height
        {
            get { return UpperBound.Y - LowerBound.Y; }
        }

        /// <summary>
        /// Get the center of the AABB.
        /// </summary>
        public AetherVector2 Center
        {
            get { return Fix64Constants.PointFive * (LowerBound + UpperBound); }
        }

        /// <summary>
        /// Get the extents of the AABB (half-widths).
        /// </summary>
        public AetherVector2 Extents
        {
            get { return Fix64Constants.PointFive * (UpperBound - LowerBound); }
        }

        /// <summary>
        /// Get the perimeter length
        /// </summary>
        public Fix64 Perimeter
        {
            get
            {
                Fix64 wx = UpperBound.X - LowerBound.X;
                Fix64 wy = UpperBound.Y - LowerBound.Y;
                return Fix64Constants.Two * (wx + wy);
            }
        }

        /// <summary>
        /// Gets the vertices of the AABB.
        /// </summary>
        /// <value>The corners of the AABB</value>
        public Vertices Vertices
        {
            get
            {
                Vertices vertices = new Vertices(4);
                vertices.Add(UpperBound);
                vertices.Add(new AetherVector2(UpperBound.X, LowerBound.Y));
                vertices.Add(LowerBound);
                vertices.Add(new AetherVector2(LowerBound.X, UpperBound.Y));
                return vertices;
            }
        }

        /// <summary>
        /// First quadrant
        /// </summary>
        public AABB Q1
        {
            get { return new AABB(Center, UpperBound); }
        }

        /// <summary>
        /// Second quadrant
        /// </summary>
        public AABB Q2
        {
            get { return new AABB(new AetherVector2(LowerBound.X, Center.Y), new AetherVector2(Center.X, UpperBound.Y)); }
        }

        /// <summary>
        /// Third quadrant
        /// </summary>
        public AABB Q3
        {
            get { return new AABB(LowerBound, Center); }
        }

        /// <summary>
        /// Forth quadrant
        /// </summary>
        public AABB Q4
        {
            get { return new AABB(new AetherVector2(Center.X, LowerBound.Y), new AetherVector2(UpperBound.X, Center.Y)); }
        }

        /// <summary>
        /// Verify that the bounds are sorted. And the bounds are valid numbers (not NaN).
        /// </summary>
        /// <returns>
        /// 	<c>true</c> if this instance is valid; otherwise, <c>false</c>.
        /// </returns>
        public bool IsValid()
        {
            AetherVector2 d = UpperBound - LowerBound;
            bool valid = d.X >= Fix64.Zero && d.Y >= Fix64.Zero;
            return valid;
        }

        /// <summary>
        /// Combine an AABB into this one.
        /// </summary>
        /// <param name="aabb">The aabb.</param>
        public void Combine(ref AABB aabb)
        {
            AetherVector2.Min(ref LowerBound, ref aabb.LowerBound, out LowerBound);
            AetherVector2.Max(ref UpperBound, ref aabb.UpperBound, out UpperBound);
        }

        /// <summary>
        /// Combine two AABBs into this one.
        /// </summary>
        /// <param name="aabb1">The aabb1.</param>
        /// <param name="aabb2">The aabb2.</param>
        public void Combine(ref AABB aabb1, ref AABB aabb2)
        {
            AetherVector2.Min(ref aabb1.LowerBound, ref aabb2.LowerBound, out LowerBound);
            AetherVector2.Max(ref aabb1.UpperBound, ref aabb2.UpperBound, out UpperBound);
        }

        /// <summary>
        /// Does this aabb contain the provided AABB.
        /// </summary>
        /// <param name="aabb">The aabb.</param>
        /// <returns>
        /// 	<c>true</c> if it contains the specified aabb; otherwise, <c>false</c>.
        /// </returns>
        public bool Contains(ref AABB aabb)
        {
            bool result = true;
            result = result && LowerBound.X <= aabb.LowerBound.X;
            result = result && LowerBound.Y <= aabb.LowerBound.Y;
            result = result && aabb.UpperBound.X <= UpperBound.X;
            result = result && aabb.UpperBound.Y <= UpperBound.Y;
            return result;
        }

        /// <summary>
        /// Determines whether the AAABB contains the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns>
        /// 	<c>true</c> if it contains the specified point; otherwise, <c>false</c>.
        /// </returns>
        public bool Contains(ref AetherVector2 point)
        {
            //using epsilon to try and gaurd against Fix64 rounding errors.
            return (point.X > (LowerBound.X + Settings.Epsilon) && point.X < (UpperBound.X - Settings.Epsilon) &&
                   (point.Y > (LowerBound.Y + Settings.Epsilon) && point.Y < (UpperBound.Y - Settings.Epsilon)));
        }

        /// <summary>
        /// Test if the two AABBs overlap.
        /// </summary>
        /// <param name="a">The first AABB.</param>
        /// <param name="b">The second AABB.</param>
        /// <returns>True if they are overlapping.</returns>
        public static bool TestOverlap(ref AABB a, ref AABB b)
        {
            if (b.LowerBound.X > a.UpperBound.X || b.LowerBound.Y > a.UpperBound.Y)
                return false;

            if (a.LowerBound.X > b.UpperBound.X || a.LowerBound.Y > b.UpperBound.Y)
                return false;

            return true;
        }

        /// <summary>
        /// Raycast against this AABB using the specificed points and maxfraction (found in input)
        /// </summary>
        /// <param name="output">The results of the raycast.</param>
        /// <param name="input">The parameters for the raycast.</param>
        /// <returns>True if the ray intersects the AABB</returns>
        public bool RayCast(out RayCastOutput output, ref RayCastInput input, bool doInteriorCheck = true)
        {
            // From Real-time Collision Detection, p179.

            output = new RayCastOutput();

            Fix64 tmin = -Settings.MaxFloat;
            Fix64 tmax = Settings.MaxFloat;

            AetherVector2 p = input.Point1;
            AetherVector2 d = input.Point2 - input.Point1;
            AetherVector2 absD = MathUtils.Abs(d);

            AetherVector2 normal = AetherVector2.Zero;

            for (int i = 0; i < 2; ++i)
            {
                Fix64 absD_i = i == 0 ? absD.X : absD.Y;
                Fix64 lowerBound_i = i == 0 ? LowerBound.X : LowerBound.Y;
                Fix64 upperBound_i = i == 0 ? UpperBound.X : UpperBound.Y;
                Fix64 p_i = i == 0 ? p.X : p.Y;

                if (absD_i < Settings.Epsilon)
                {
                    // Parallel.
                    if (p_i < lowerBound_i || upperBound_i < p_i)
                    {
                        return false;
                    }
                }
                else
                {
                    Fix64 d_i = i == 0 ? d.X : d.Y;

                    Fix64 inv_d = Fix64.One / d_i;
                    Fix64 t1 = (lowerBound_i - p_i) * inv_d;
                    Fix64 t2 = (upperBound_i - p_i) * inv_d;

                    // Sign of the normal vector.
                    Fix64 s = -Fix64.One;

                    if (t1 > t2)
                    {
                        MathUtils.Swap(ref t1, ref t2);
                        s = Fix64.One;
                    }

                    // Push the min up
                    if (t1 > tmin)
                    {
                        if (i == 0)
                        {
                            normal.X = s;
                        }
                        else
                        {
                            normal.Y = s;
                        }

                        tmin = t1;
                    }

                    // Pull the max down
                    tmax = MathUtils.Min(tmax, t2);

                    if (tmin > tmax)
                    {
                        return false;
                    }
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (doInteriorCheck && (tmin < Fix64.Zero || input.MaxFraction < tmin))
            {
                return false;
            }

            // Intersection.
            output.Fraction = tmin;
            output.Normal = normal;
            return true;
        }
    }

    /// <summary>
    /// This structure is used to keep track of the best separating axis.
    /// </summary>
    public struct EPAxis
    {
        public int Index;
        public Fix64 Separation;
        public EPAxisType Type;
    }

    /// <summary>
    /// Reference face used for clipping
    /// </summary>
    public struct ReferenceFace
    {
        public int i1, i2;

        public AetherVector2 v1, v2;

        public AetherVector2 normal;

        public AetherVector2 sideNormal1;
        public Fix64 sideOffset1;

        public AetherVector2 sideNormal2;
        public Fix64 sideOffset2;
    }

    public enum EPAxisType
    {
        Unknown,
        EdgeA,
        EdgeB,
    }

    /// <summary>
    /// Collision methods
    /// </summary>
    public static class Collision
    {
        /// <summary>
        /// Test overlap between the two shapes.
        /// </summary>
        /// <param name="shapeA">The first shape.</param>
        /// <param name="indexA">The index for the first shape.</param>
        /// <param name="shapeB">The second shape.</param>
        /// <param name="indexB">The index for the second shape.</param>
        /// <param name="xfA">The transform for the first shape.</param>
        /// <param name="xfB">The transform for the seconds shape.</param>
        /// <returns></returns>
        public static bool TestOverlap(Shape shapeA, int indexA, Shape shapeB, int indexB, ref Transform xfA, ref Transform xfB)
        {
            DistanceInput _input = new DistanceInput();
            _input.ProxyA = new DistanceProxy(shapeA, indexA);
            _input.ProxyB = new DistanceProxy(shapeB, indexB);
            _input.TransformA = xfA;
            _input.TransformB = xfB;
            _input.UseRadii = true;

            SimplexCache cache;
            DistanceOutput output;
            Distance.ComputeDistance(out output, out cache, _input);

            return output.Distance < Fix64Constants.Ten * Settings.Epsilon;
        }

        public static void GetPointStates(out FixedArray2<PointState> state1, out FixedArray2<PointState> state2, ref Manifold manifold1, ref Manifold manifold2)
        {
            state1 = new FixedArray2<PointState>();
            state2 = new FixedArray2<PointState>();

            // Detect persists and removes.
            for (int i = 0; i < manifold1.PointCount; ++i)
            {
                ContactID id = manifold1.Points[i].Id;

                state1[i] = PointState.Remove;

                for (int j = 0; j < manifold2.PointCount; ++j)
                {
                    if (manifold2.Points[j].Id.Key == id.Key)
                    {
                        state1[i] = PointState.Persist;
                        break;
                    }
                }
            }

            // Detect persists and adds.
            for (int i = 0; i < manifold2.PointCount; ++i)
            {
                ContactID id = manifold2.Points[i].Id;

                state2[i] = PointState.Add;

                for (int j = 0; j < manifold1.PointCount; ++j)
                {
                    if (manifold1.Points[j].Id.Key == id.Key)
                    {
                        state2[i] = PointState.Persist;
                        break;
                    }
                }
            }
        }

        /// <summary>
        /// Compute the collision manifold between two circles.
        /// </summary>
        public static void CollideCircles(ref Manifold manifold, CircleShape circleA, ref Transform xfA, CircleShape circleB, ref Transform xfB)
        {
            manifold.PointCount = 0;

            AetherVector2 pA = Transform.Multiply(ref circleA._position, ref xfA);
            AetherVector2 pB = Transform.Multiply(ref circleB._position, ref xfB);

            AetherVector2 d = pB - pA;
            Fix64 distSqr = AetherVector2.Dot(d, d);
            Fix64 radius = circleA.Radius + circleB.Radius;
            if (distSqr > radius * radius)
            {
                return;
            }

            manifold.Type = ManifoldType.Circles;
            manifold.LocalPoint = circleA.Position;
            manifold.LocalNormal = AetherVector2.Zero;
            manifold.PointCount = 1;

            ManifoldPoint p0 = manifold.Points[0];

            p0.LocalPoint = circleB.Position;
            p0.Id.Key = 0;

            manifold.Points[0] = p0;
        }

        /// <summary>
        /// Compute the collision manifold between a polygon and a circle.
        /// </summary>
        /// <param name="manifold">The manifold.</param>
        /// <param name="polygonA">The polygon A.</param>
        /// <param name="xfA">The transform of A.</param>
        /// <param name="circleB">The circle B.</param>
        /// <param name="xfB">The transform of B.</param>
        public static void CollidePolygonAndCircle(ref Manifold manifold, PolygonShape polygonA, ref Transform xfA, CircleShape circleB, ref Transform xfB)
        {
            manifold.PointCount = 0;

            // Compute circle position in the frame of the polygon.
            AetherVector2 c = Transform.Multiply(ref circleB._position, ref xfB);
            AetherVector2 cLocal = Transform.Divide(ref c, ref xfA);

            // Find the min separating edge.
            int normalIndex = 0;
            Fix64 separation = -Settings.MaxFloat;
            Fix64 radius = polygonA.Radius + circleB.Radius;
            int vertexCount = polygonA.Vertices.Count;

            for (int i = 0; i < vertexCount; ++i)
            {
                AetherVector2 value1 = polygonA.Normals[i];
                AetherVector2 value2 = cLocal - polygonA.Vertices[i];
                Fix64 s = value1.X * value2.X + value1.Y * value2.Y;

                if (s > radius)
                {
                    // Early out.
                    return;
                }

                if (s > separation)
                {
                    separation = s;
                    normalIndex = i;
                }
            }

            // Vertices that subtend the incident face.
            int vertIndex1 = normalIndex;
            int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
            AetherVector2 v1 = polygonA.Vertices[vertIndex1];
            AetherVector2 v2 = polygonA.Vertices[vertIndex2];

            // If the center is inside the polygon ...
            if (separation < Settings.Epsilon)
            {
                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = polygonA.Normals[normalIndex];
                manifold.LocalPoint = Fix64Constants.PointFive * (v1 + v2);

                ManifoldPoint p0 = manifold.Points[0];

                p0.LocalPoint = circleB.Position;
                p0.Id.Key = 0;

                manifold.Points[0] = p0;

                return;
            }

            // Compute barycentric coordinates
            Fix64 u1 = (cLocal.X - v1.X) * (v2.X - v1.X) + (cLocal.Y - v1.Y) * (v2.Y - v1.Y);
            Fix64 u2 = (cLocal.X - v2.X) * (v1.X - v2.X) + (cLocal.Y - v2.Y) * (v1.Y - v2.Y);

            if (u1 <= Fix64.Zero)
            {
                Fix64 r = (cLocal.X - v1.X) * (cLocal.X - v1.X) + (cLocal.Y - v1.Y) * (cLocal.Y - v1.Y);
                if (r > radius * radius)
                {
                    return;
                }

                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = cLocal - v1;
                Fix64 factor = Fix64.One /
                               Fix64.Sqrt(manifold.LocalNormal.X * manifold.LocalNormal.X +
                                         manifold.LocalNormal.Y * manifold.LocalNormal.Y);
                manifold.LocalNormal.X = manifold.LocalNormal.X * factor;
                manifold.LocalNormal.Y = manifold.LocalNormal.Y * factor;
                manifold.LocalPoint = v1;

                ManifoldPoint p0b = manifold.Points[0];

                p0b.LocalPoint = circleB.Position;
                p0b.Id.Key = 0;

                manifold.Points[0] = p0b;
            }
            else if (u2 <= Fix64.Zero)
            {
                Fix64 r = (cLocal.X - v2.X) * (cLocal.X - v2.X) + (cLocal.Y - v2.Y) * (cLocal.Y - v2.Y);
                if (r > radius * radius)
                {
                    return;
                }

                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = cLocal - v2;
                Fix64 factor = Fix64.One /
                               Fix64.Sqrt(manifold.LocalNormal.X * manifold.LocalNormal.X +
                                         manifold.LocalNormal.Y * manifold.LocalNormal.Y);
                manifold.LocalNormal.X = manifold.LocalNormal.X * factor;
                manifold.LocalNormal.Y = manifold.LocalNormal.Y * factor;
                manifold.LocalPoint = v2;

                ManifoldPoint p0c = manifold.Points[0];

                p0c.LocalPoint = circleB.Position;
                p0c.Id.Key = 0;

                manifold.Points[0] = p0c;
            }
            else
            {
                AetherVector2 faceCenter = Fix64Constants.PointFive * (v1 + v2);
                AetherVector2 value1 = cLocal - faceCenter;
                AetherVector2 value2 = polygonA.Normals[vertIndex1];
                Fix64 separation2 = value1.X * value2.X + value1.Y * value2.Y;
                if (separation2 > radius)
                {
                    return;
                }

                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = polygonA.Normals[vertIndex1];
                manifold.LocalPoint = faceCenter;

                ManifoldPoint p0d = manifold.Points[0];

                p0d.LocalPoint = circleB.Position;
                p0d.Id.Key = 0;

                manifold.Points[0] = p0d;
            }
        }

        /// <summary>
        /// Compute the collision manifold between two polygons.
        /// </summary>
        /// <param name="manifold">The manifold.</param>
        /// <param name="polyA">The poly A.</param>
        /// <param name="transformA">The transform A.</param>
        /// <param name="polyB">The poly B.</param>
        /// <param name="transformB">The transform B.</param>
        public static void CollidePolygons(ref Manifold manifold, PolygonShape polyA, ref Transform transformA, PolygonShape polyB, ref Transform transformB)
        {
            manifold.PointCount = 0;
            Fix64 totalRadius = polyA.Radius + polyB.Radius;

            int edgeA = 0;
            Fix64 separationA = FindMaxSeparation(out edgeA, polyA, ref transformA, polyB, ref transformB);
            if (separationA > totalRadius)
                return;

            int edgeB = 0;
            Fix64 separationB = FindMaxSeparation(out edgeB, polyB, ref transformB, polyA, ref transformA);
            if (separationB > totalRadius)
                return;

            PolygonShape poly1; // reference polygon
            PolygonShape poly2; // incident polygon
            Transform xf1, xf2;
            int edge1; // reference edge
            bool flip;

            if (separationB > Fix64Constants.k_relativeTol * separationA + Fix64Constants.k_absoluteTol)
            {
                poly1 = polyB;
                poly2 = polyA;
                xf1 = transformB;
                xf2 = transformA;
                edge1 = edgeB;
                manifold.Type = ManifoldType.FaceB;
                flip = true;
            }
            else
            {
                poly1 = polyA;
                poly2 = polyB;
                xf1 = transformA;
                xf2 = transformB;
                edge1 = edgeA;
                manifold.Type = ManifoldType.FaceA;
                flip = false;
            }

            FixedArray2<ClipVertex> incidentEdge;
            FindIncidentEdge(out incidentEdge, poly1, ref xf1, edge1, poly2, ref xf2);

            int count1 = poly1.Vertices.Count;

            int iv1 = edge1;
            int iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

            AetherVector2 v11 = poly1.Vertices[iv1];
            AetherVector2 v12 = poly1.Vertices[iv2];

            AetherVector2 localTangent = v12 - v11;
            localTangent.Normalize();

            AetherVector2 localNormal = new AetherVector2(localTangent.Y, -localTangent.X);
            AetherVector2 planePoint = Fix64Constants.PointFive * (v11 + v12);

            AetherVector2 tangent = Complex.Multiply(ref localTangent, ref xf1.q);

            Fix64 normalx = tangent.Y;
            Fix64 normaly = -tangent.X;

            v11 = Transform.Multiply(ref v11, ref xf1);
            v12 = Transform.Multiply(ref v12, ref xf1);

            // Face offset.
            Fix64 frontOffset = normalx * v11.X + normaly * v11.Y;

            // Side offsets, extended by polytope skin thickness.
            Fix64 sideOffset1 = -(tangent.X * v11.X + tangent.Y * v11.Y) + totalRadius;
            Fix64 sideOffset2 = tangent.X * v12.X + tangent.Y * v12.Y + totalRadius;

            // Clip incident edge against extruded edge1 side edges.
            FixedArray2<ClipVertex> clipPoints1;
            FixedArray2<ClipVertex> clipPoints2;

            // Clip to box side 1
            int np = ClipSegmentToLine(out clipPoints1, ref incidentEdge, -tangent, sideOffset1, iv1);

            if (np < 2)
                return;

            // Clip to negative box side 1
            np = ClipSegmentToLine(out clipPoints2, ref clipPoints1, tangent, sideOffset2, iv2);

            if (np < 2)
            {
                return;
            }

            // Now clipPoints2 contains the clipped points.
            manifold.LocalNormal = localNormal;
            manifold.LocalPoint = planePoint;

            int pointCount = 0;
            for (int i = 0; i < Settings.MaxManifoldPoints; ++i)
            {
                AetherVector2 value = clipPoints2[i].V;
                Fix64 separation = normalx * value.X + normaly * value.Y - frontOffset;

                if (separation <= totalRadius)
                {
                    ManifoldPoint cp = manifold.Points[pointCount];
                    Transform.Divide(clipPoints2[i].V, ref xf2, out cp.LocalPoint);
                    cp.Id = clipPoints2[i].ID;

                    if (flip)
                    {
                        // Swap features
                        ContactFeature cf = cp.Id.Features;
                        cp.Id.Features.IndexA = cf.IndexB;
                        cp.Id.Features.IndexB = cf.IndexA;
                        cp.Id.Features.TypeA = cf.TypeB;
                        cp.Id.Features.TypeB = cf.TypeA;
                    }

                    manifold.Points[pointCount] = cp;

                    ++pointCount;
                }
            }

            manifold.PointCount = pointCount;
        }

        /// <summary>
        /// Compute contact points for edge versus circle.
        /// This accounts for edge connectivity.
        /// </summary>
        /// <param name="manifold">The manifold.</param>
        /// <param name="edgeA">The edge A.</param>
        /// <param name="transformA">The transform A.</param>
        /// <param name="circleB">The circle B.</param>
        /// <param name="transformB">The transform B.</param>
        public static void CollideEdgeAndCircle(ref Manifold manifold, EdgeShape edgeA, ref Transform transformA, CircleShape circleB, ref Transform transformB)
        {
            manifold.PointCount = 0;

            // Compute circle in frame of edge
            AetherVector2 Q = Transform.Divide(Transform.Multiply(ref circleB._position, ref transformB), ref transformA);

            AetherVector2 A = edgeA.Vertex1, B = edgeA.Vertex2;
            AetherVector2 e = B - A;

            // Barycentric coordinates
            Fix64 u = AetherVector2.Dot(e, B - Q);
            Fix64 v = AetherVector2.Dot(e, Q - A);

            Fix64 radius = edgeA.Radius + circleB.Radius;

            ContactFeature cf;
            cf.IndexB = 0;
            cf.TypeB = (byte)ContactFeatureType.Vertex;

            AetherVector2 P, d;

            // Region A
            if (v <= Fix64.Zero)
            {
                P = A;
                d = Q - P;
                Fix64 dd;
                AetherVector2.Dot(ref d, ref d, out dd);
                if (dd > radius * radius)
                {
                    return;
                }

                // Is there an edge connected to A?
                if (edgeA.HasVertex0)
                {
                    AetherVector2 A1 = edgeA.Vertex0;
                    AetherVector2 B1 = A;
                    AetherVector2 e1 = B1 - A1;
                    Fix64 u1 = AetherVector2.Dot(e1, B1 - Q);

                    // Is the circle in Region AB of the previous edge?
                    if (u1 > Fix64.Zero)
                    {
                        return;
                    }
                }

                cf.IndexA = 0;
                cf.TypeA = (byte)ContactFeatureType.Vertex;
                manifold.PointCount = 1;
                manifold.Type = ManifoldType.Circles;
                manifold.LocalNormal = AetherVector2.Zero;
                manifold.LocalPoint = P;
                ManifoldPoint mp = new ManifoldPoint();
                mp.Id.Key = 0;
                mp.Id.Features = cf;
                mp.LocalPoint = circleB.Position;
                manifold.Points[0] = mp;
                return;
            }

            // Region B
            if (u <= Fix64.Zero)
            {
                P = B;
                d = Q - P;
                Fix64 dd;
                AetherVector2.Dot(ref d, ref d, out dd);
                if (dd > radius * radius)
                {
                    return;
                }

                // Is there an edge connected to B?
                if (edgeA.HasVertex3)
                {
                    AetherVector2 B2 = edgeA.Vertex3;
                    AetherVector2 A2 = B;
                    AetherVector2 e2 = B2 - A2;
                    Fix64 v2 = AetherVector2.Dot(e2, Q - A2);

                    // Is the circle in Region AB of the next edge?
                    if (v2 > Fix64.Zero)
                    {
                        return;
                    }
                }

                cf.IndexA = 1;
                cf.TypeA = (byte)ContactFeatureType.Vertex;
                manifold.PointCount = 1;
                manifold.Type = ManifoldType.Circles;
                manifold.LocalNormal = AetherVector2.Zero;
                manifold.LocalPoint = P;
                ManifoldPoint mp = new ManifoldPoint();
                mp.Id.Key = 0;
                mp.Id.Features = cf;
                mp.LocalPoint = circleB.Position;
                manifold.Points[0] = mp;
                return;
            }

            // Region AB
            Fix64 den;
            AetherVector2.Dot(ref e, ref e, out den);
            Debug.Assert(den > Fix64.Zero);
            P = (Fix64.One / den) * (u * A + v * B);
            d = Q - P;
            Fix64 dd2;
            AetherVector2.Dot(ref d, ref d, out dd2);
            if (dd2 > radius * radius)
            {
                return;
            }

            AetherVector2 n = new AetherVector2(-e.Y, e.X);
            if (AetherVector2.Dot(n, Q - A) < Fix64.Zero)
            {
                n = new AetherVector2(-n.X, -n.Y);
            }
            n.Normalize();

            cf.IndexA = 0;
            cf.TypeA = (byte)ContactFeatureType.Face;
            manifold.PointCount = 1;
            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = n;
            manifold.LocalPoint = A;
            ManifoldPoint mp2 = new ManifoldPoint();
            mp2.Id.Key = 0;
            mp2.Id.Features = cf;
            mp2.LocalPoint = circleB.Position;
            manifold.Points[0] = mp2;
        }

        /// <summary>
        /// Collides and edge and a polygon, taking into account edge adjacency.
        /// </summary>
        /// <param name="manifold">The manifold.</param>
        /// <param name="edgeA">The edge A.</param>
        /// <param name="xfA">The xf A.</param>
        /// <param name="polygonB">The polygon B.</param>
        /// <param name="xfB">The xf B.</param>
        public static void CollideEdgeAndPolygon(ref Manifold manifold, EdgeShape edgeA, ref Transform xfA, PolygonShape polygonB, ref Transform xfB)
        {
            EPCollider.Collide(ref manifold, edgeA, ref xfA, polygonB, ref xfB);
        }

        private static class EPCollider
        {
            /// <summary>
            /// This holds polygon B expressed in frame A.
            /// </summary>
            internal struct TempPolygon
            {
                public AetherVector2[] Vertices;
                public AetherVector2[] Normals;
                public int Count;

                internal TempPolygon(int maxPolygonVertices)
                {
                    Vertices = new AetherVector2[maxPolygonVertices];
                    Normals = new AetherVector2[maxPolygonVertices];
                    Count = 0;
                }
            }

            public static void Collide(ref Manifold manifold, EdgeShape edgeA, ref Transform xfA, PolygonShape polygonB, ref Transform xfB)
            {
                // Algorithm:
                // 1. Classify v1 and v2
                // 2. Classify polygon centroid as front or back
                // 3. Flip normal if necessary
                // 4. Initialize normal range to [-pi, pi] about face normal
                // 5. Adjust normal range according to adjacent edges
                // 6. Visit each separating axes, only accept axes within the range
                // 7. Return if _any_ axis indicates separation
                // 8. Clip

                TempPolygon tempPolygonB = new TempPolygon(Settings.MaxPolygonVertices);
                Transform xf;
                AetherVector2 centroidB;
                AetherVector2 normal0 = new AetherVector2();
                AetherVector2 normal1;
                AetherVector2 normal2 = new AetherVector2();
                AetherVector2 normal;
                AetherVector2 lowerLimit, upperLimit;
                Fix64 radius;
                bool front;

                Transform.Divide(ref xfB, ref xfA, out xf);

                centroidB = Transform.Multiply(polygonB.MassData.Centroid, ref xf);

                AetherVector2 v0 = edgeA.Vertex0;
                AetherVector2 v1 = edgeA._vertex1;
                AetherVector2 v2 = edgeA._vertex2;
                AetherVector2 v3 = edgeA.Vertex3;

                bool hasVertex0 = edgeA.HasVertex0;
                bool hasVertex3 = edgeA.HasVertex3;

                AetherVector2 edge1 = v2 - v1;
                edge1.Normalize();
                normal1 = new AetherVector2(edge1.Y, -edge1.X);
                Fix64 offset1 = AetherVector2.Dot(normal1, centroidB - v1);
                Fix64 offset0 = Fix64.Zero, offset2 = Fix64.Zero;
                bool convex1 = false, convex2 = false;

                // Is there a preceding edge?
                if (hasVertex0)
                {
                    AetherVector2 edge0 = v1 - v0;
                    edge0.Normalize();
                    normal0 = new AetherVector2(edge0.Y, -edge0.X);
                    convex1 = MathUtils.Cross(ref edge0, ref edge1) >= Fix64.Zero;
                    offset0 = AetherVector2.Dot(normal0, centroidB - v0);
                }

                // Is there a following edge?
                if (hasVertex3)
                {
                    AetherVector2 edge2 = v3 - v2;
                    edge2.Normalize();
                    normal2 = new AetherVector2(edge2.Y, -edge2.X);
                    convex2 = MathUtils.Cross(ref edge1, ref edge2) > Fix64.Zero;
                    offset2 = AetherVector2.Dot(normal2, centroidB - v2);
                }

                // Determine front or back collision. Determine collision normal limits.
                if (hasVertex0 && hasVertex3)
                {
                    if (convex1 && convex2)
                    {
                        front = offset0 >= Fix64.Zero || offset1 >= Fix64.Zero || offset2 >= Fix64.Zero;
                        if (front)
                        {
                            normal = normal1;
                            lowerLimit = normal0;
                            upperLimit = normal2;
                        }
                        else
                        {
                            normal = -normal1;
                            lowerLimit = -normal1;
                            upperLimit = -normal1;
                        }
                    }
                    else if (convex1)
                    {
                        front = offset0 >= Fix64.Zero || (offset1 >= Fix64.Zero && offset2 >= Fix64.Zero);
                        if (front)
                        {
                            normal = normal1;
                            lowerLimit = normal0;
                            upperLimit = normal1;
                        }
                        else
                        {
                            normal = -normal1;
                            lowerLimit = -normal2;
                            upperLimit = -normal1;
                        }
                    }
                    else if (convex2)
                    {
                        front = offset2 >= Fix64.Zero || (offset0 >= Fix64.Zero && offset1 >= Fix64.Zero);
                        if (front)
                        {
                            normal = normal1;
                            lowerLimit = normal1;
                            upperLimit = normal2;
                        }
                        else
                        {
                            normal = -normal1;
                            lowerLimit = -normal1;
                            upperLimit = -normal0;
                        }
                    }
                    else
                    {
                        front = offset0 >= Fix64.Zero && offset1 >= Fix64.Zero && offset2 >= Fix64.Zero;
                        if (front)
                        {
                            normal = normal1;
                            lowerLimit = normal1;
                            upperLimit = normal1;
                        }
                        else
                        {
                            normal = -normal1;
                            lowerLimit = -normal2;
                            upperLimit = -normal0;
                        }
                    }
                }
                else if (hasVertex0)
                {
                    if (convex1)
                    {
                        front = offset0 >= Fix64.Zero || offset1 >= Fix64.Zero;
                        if (front)
                        {
                            normal = normal1;
                            lowerLimit = normal0;
                            upperLimit = -normal1;
                        }
                        else
                        {
                            normal = -normal1;
                            lowerLimit = normal1;
                            upperLimit = -normal1;
                        }
                    }
                    else
                    {
                        front = offset0 >= Fix64.Zero && offset1 >= Fix64.Zero;
                        if (front)
                        {
                            normal = normal1;
                            lowerLimit = normal1;
                            upperLimit = -normal1;
                        }
                        else
                        {
                            normal = -normal1;
                            lowerLimit = normal1;
                            upperLimit = -normal0;
                        }
                    }
                }
                else if (hasVertex3)
                {
                    if (convex2)
                    {
                        front = offset1 >= Fix64.Zero || offset2 >= Fix64.Zero;
                        if (front)
                        {
                            normal = normal1;
                            lowerLimit = -normal1;
                            upperLimit = normal2;
                        }
                        else
                        {
                            normal = -normal1;
                            lowerLimit = -normal1;
                            upperLimit = normal1;
                        }
                    }
                    else
                    {
                        front = offset1 >= Fix64.Zero && offset2 >= Fix64.Zero;
                        if (front)
                        {
                            normal = normal1;
                            lowerLimit = -normal1;
                            upperLimit = normal1;
                        }
                        else
                        {
                            normal = -normal1;
                            lowerLimit = -normal2;
                            upperLimit = normal1;
                        }
                    }
                }
                else
                {
                    front = offset1 >= Fix64.Zero;
                    if (front)
                    {
                        normal = normal1;
                        lowerLimit = -normal1;
                        upperLimit = -normal1;
                    }
                    else
                    {
                        normal = -normal1;
                        lowerLimit = normal1;
                        upperLimit = normal1;
                    }
                }

                // Get polygonB in frameA
                tempPolygonB.Count = polygonB.Vertices.Count;
                for (int i = 0; i < polygonB.Vertices.Count; ++i)
                {
                    tempPolygonB.Vertices[i] = Transform.Multiply(polygonB.Vertices[i], ref xf);
                    tempPolygonB.Normals[i] = Complex.Multiply(polygonB.Normals[i], ref xf.q);
                }

                radius = Fix64Constants.Two * Settings.PolygonRadius;

                manifold.PointCount = 0;

                EPAxis edgeAxis = ComputeEdgeSeparation(ref tempPolygonB, ref normal, ref v1, front);

                // If no valid normal can be found than this edge should not collide.
                if (edgeAxis.Type == EPAxisType.Unknown)
                {
                    return;
                }

                if (edgeAxis.Separation > radius)
                {
                    return;
                }

                EPAxis polygonAxis = ComputePolygonSeparation(ref tempPolygonB, ref normal, ref v1, ref v2, ref lowerLimit, ref upperLimit, radius);
                if (polygonAxis.Type != EPAxisType.Unknown && polygonAxis.Separation > radius)
                {
                    return;
                }

                // Use hysteresis for jitter reduction.
                EPAxis primaryAxis;
                if (polygonAxis.Type == EPAxisType.Unknown)
                {
                    primaryAxis = edgeAxis;
                }
                else if (polygonAxis.Separation > Fix64Constants.k_relativeTol * edgeAxis.Separation + Fix64Constants.k_absoluteTol)
                {
                    primaryAxis = polygonAxis;
                }
                else
                {
                    primaryAxis = edgeAxis;
                }

                FixedArray2<ClipVertex> ie = new FixedArray2<ClipVertex>();
                ReferenceFace rf;
                if (primaryAxis.Type == EPAxisType.EdgeA)
                {
                    manifold.Type = ManifoldType.FaceA;

                    // Search for the polygon normal that is most anti-parallel to the edge normal.
                    int bestIndex = 0;
                    Fix64 bestValue = AetherVector2.Dot(normal, tempPolygonB.Normals[0]);
                    for (int i = 1; i < tempPolygonB.Count; ++i)
                    {
                        Fix64 value = AetherVector2.Dot(normal, tempPolygonB.Normals[i]);
                        if (value < bestValue)
                        {
                            bestValue = value;
                            bestIndex = i;
                        }
                    }

                    int i1 = bestIndex;
                    int i2 = i1 + 1 < tempPolygonB.Count ? i1 + 1 : 0;

                    ClipVertex c0 = ie[0];
                    c0.V = tempPolygonB.Vertices[i1];
                    c0.ID.Features.IndexA = 0;
                    c0.ID.Features.IndexB = (byte)i1;
                    c0.ID.Features.TypeA = (byte)ContactFeatureType.Face;
                    c0.ID.Features.TypeB = (byte)ContactFeatureType.Vertex;
                    ie[0] = c0;

                    ClipVertex c1 = ie[1];
                    c1.V = tempPolygonB.Vertices[i2];
                    c1.ID.Features.IndexA = 0;
                    c1.ID.Features.IndexB = (byte)i2;
                    c1.ID.Features.TypeA = (byte)ContactFeatureType.Face;
                    c1.ID.Features.TypeB = (byte)ContactFeatureType.Vertex;
                    ie[1] = c1;

                    if (front)
                    {
                        rf.i1 = 0;
                        rf.i2 = 1;
                        rf.v1 = v1;
                        rf.v2 = v2;
                        rf.normal = normal1;
                    }
                    else
                    {
                        rf.i1 = 1;
                        rf.i2 = 0;
                        rf.v1 = v2;
                        rf.v2 = v1;
                        rf.normal = -normal1;
                    }
                }
                else
                {
                    manifold.Type = ManifoldType.FaceB;
                    ClipVertex c0 = ie[0];
                    c0.V = v1;
                    c0.ID.Features.IndexA = 0;
                    c0.ID.Features.IndexB = (byte)primaryAxis.Index;
                    c0.ID.Features.TypeA = (byte)ContactFeatureType.Vertex;
                    c0.ID.Features.TypeB = (byte)ContactFeatureType.Face;
                    ie[0] = c0;

                    ClipVertex c1 = ie[1];
                    c1.V = v2;
                    c1.ID.Features.IndexA = 0;
                    c1.ID.Features.IndexB = (byte)primaryAxis.Index;
                    c1.ID.Features.TypeA = (byte)ContactFeatureType.Vertex;
                    c1.ID.Features.TypeB = (byte)ContactFeatureType.Face;
                    ie[1] = c1;

                    rf.i1 = primaryAxis.Index;
                    rf.i2 = rf.i1 + 1 < tempPolygonB.Count ? rf.i1 + 1 : 0;
                    rf.v1 = tempPolygonB.Vertices[rf.i1];
                    rf.v2 = tempPolygonB.Vertices[rf.i2];
                    rf.normal = tempPolygonB.Normals[rf.i1];
                }

                rf.sideNormal1 = new AetherVector2(rf.normal.Y, -rf.normal.X);
                rf.sideNormal2 = -rf.sideNormal1;
                rf.sideOffset1 = AetherVector2.Dot(rf.sideNormal1, rf.v1);
                rf.sideOffset2 = AetherVector2.Dot(rf.sideNormal2, rf.v2);

                // Clip incident edge against extruded edge1 side edges.
                FixedArray2<ClipVertex> clipPoints1;
                FixedArray2<ClipVertex> clipPoints2;
                int np;

                // Clip to box side 1
                np = ClipSegmentToLine(out clipPoints1, ref ie, rf.sideNormal1, rf.sideOffset1, rf.i1);

                if (np < Settings.MaxManifoldPoints)
                {
                    return;
                }

                // Clip to negative box side 1
                np = ClipSegmentToLine(out clipPoints2, ref clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);

                if (np < Settings.MaxManifoldPoints)
                {
                    return;
                }

                // Now clipPoints2 contains the clipped points.
                if (primaryAxis.Type == EPAxisType.EdgeA)
                {
                    manifold.LocalNormal = rf.normal;
                    manifold.LocalPoint = rf.v1;
                }
                else
                {
                    manifold.LocalNormal = polygonB.Normals[rf.i1];
                    manifold.LocalPoint = polygonB.Vertices[rf.i1];
                }

                int pointCount = 0;
                for (int i = 0; i < Settings.MaxManifoldPoints; ++i)
                {
                    Fix64 separation = AetherVector2.Dot(rf.normal, clipPoints2[i].V - rf.v1);

                    if (separation <= radius)
                    {
                        ManifoldPoint cp = manifold.Points[pointCount];

                        if (primaryAxis.Type == EPAxisType.EdgeA)
                        {
                            Transform.Divide(clipPoints2[i].V, ref xf, out cp.LocalPoint);
                            cp.Id = clipPoints2[i].ID;
                        }
                        else
                        {
                            cp.LocalPoint = clipPoints2[i].V;
                            cp.Id.Features.TypeA = clipPoints2[i].ID.Features.TypeB;
                            cp.Id.Features.TypeB = clipPoints2[i].ID.Features.TypeA;
                            cp.Id.Features.IndexA = clipPoints2[i].ID.Features.IndexB;
                            cp.Id.Features.IndexB = clipPoints2[i].ID.Features.IndexA;
                        }

                        manifold.Points[pointCount] = cp;
                        ++pointCount;
                    }
                }

                manifold.PointCount = pointCount;
            }

            private static EPAxis ComputeEdgeSeparation(ref TempPolygon polygonB, ref AetherVector2 normal, ref AetherVector2 v1, bool front)
            {
                EPAxis axis;
                axis.Type = EPAxisType.EdgeA;
                axis.Index = front ? 0 : 1;
                axis.Separation = Settings.MaxFloat;

                for (int i = 0; i < polygonB.Count; ++i)
                {
                    Fix64 s = AetherVector2.Dot(normal, polygonB.Vertices[i] - v1);
                    if (s < axis.Separation)
                    {
                        axis.Separation = s;
                    }
                }

                return axis;
            }

            private static EPAxis ComputePolygonSeparation(ref TempPolygon polygonB, ref AetherVector2 normal, ref AetherVector2 v1, ref AetherVector2 v2, ref AetherVector2 lowerLimit, ref AetherVector2 upperLimit, Fix64 radius)
            {
                EPAxis axis;
                axis.Type = EPAxisType.Unknown;
                axis.Index = -1;
                axis.Separation = -Settings.MaxFloat;

                AetherVector2 perp = new AetherVector2(-normal.Y, normal.X);

                for (int i = 0; i < polygonB.Count; ++i)
                {
                    AetherVector2 n = -polygonB.Normals[i];

                    Fix64 s1 = AetherVector2.Dot(n, polygonB.Vertices[i] - v1);
                    Fix64 s2 = AetherVector2.Dot(n, polygonB.Vertices[i] - v2);
                    Fix64 s = MathUtils.Min(s1, s2);

                    if (s > radius)
                    {
                        // No collision
                        axis.Type = EPAxisType.EdgeB;
                        axis.Index = i;
                        axis.Separation = s;
                        return axis;
                    }

                    // Adjacency
                    if (AetherVector2.Dot(n, perp) >= Fix64.Zero)
                    {
                        if (AetherVector2.Dot(n - upperLimit, normal) < -Settings.AngularSlop)
                        {
                            continue;
                        }
                    }
                    else
                    {
                        if (AetherVector2.Dot(n - lowerLimit, normal) < -Settings.AngularSlop)
                        {
                            continue;
                        }
                    }

                    if (s > axis.Separation)
                    {
                        axis.Type = EPAxisType.EdgeB;
                        axis.Index = i;
                        axis.Separation = s;
                    }
                }

                return axis;
            }
        }

        /// <summary>
        /// Clipping for contact manifolds.
        /// </summary>
        /// <param name="vOut">The v out.</param>
        /// <param name="vIn">The v in.</param>
        /// <param name="normal">The normal.</param>
        /// <param name="offset">The offset.</param>
        /// <param name="vertexIndexA">The vertex index A.</param>
        /// <returns></returns>
        private static int ClipSegmentToLine(out FixedArray2<ClipVertex> vOut, ref FixedArray2<ClipVertex> vIn, AetherVector2 normal, Fix64 offset, int vertexIndexA)
        {
            vOut = new FixedArray2<ClipVertex>();

            ClipVertex v0 = vIn[0];
            ClipVertex v1 = vIn[1];

            // Start with no output points
            int numOut = 0;

            // Calculate the distance of end points to the line
            Fix64 distance0 = normal.X * v0.V.X + normal.Y * v0.V.Y - offset;
            Fix64 distance1 = normal.X * v1.V.X + normal.Y * v1.V.Y - offset;

            // If the points are behind the plane
            if (distance0 <= Fix64.Zero) vOut[numOut++] = v0;
            if (distance1 <= Fix64.Zero) vOut[numOut++] = v1;

            // If the points are on different sides of the plane
            if (distance0 * distance1 < Fix64.Zero)
            {
                // Find intersection point of edge and plane
                Fix64 interp = distance0 / (distance0 - distance1);

                ClipVertex cv = vOut[numOut];

                cv.V.X = v0.V.X + interp * (v1.V.X - v0.V.X);
                cv.V.Y = v0.V.Y + interp * (v1.V.Y - v0.V.Y);

                // VertexA is hitting edgeB.
                cv.ID.Features.IndexA = (byte)vertexIndexA;
                cv.ID.Features.IndexB = v0.ID.Features.IndexB;
                cv.ID.Features.TypeA = (byte)ContactFeatureType.Vertex;
                cv.ID.Features.TypeB = (byte)ContactFeatureType.Face;

                vOut[numOut] = cv;

                ++numOut;
            }

            return numOut;
        }

        /// <summary>
        /// Find the separation between poly1 and poly2 for a give edge normal on poly1.
        /// </summary>
        /// <param name="poly1">The poly1.</param>
        /// <param name="xf1">The XF1.</param>
        /// <param name="edge1">The edge1.</param>
        /// <param name="poly2">The poly2.</param>
        /// <param name="xf2">The XF2.</param>
        /// <returns></returns>
        private static Fix64 EdgeSeparation(PolygonShape poly1, ref Transform xf1To2, int edge1, PolygonShape poly2)
        {
            List<AetherVector2> vertices1 = poly1.Vertices;
            List<AetherVector2> normals1 = poly1.Normals;

            int count2 = poly2.Vertices.Count;
            List<AetherVector2> vertices2 = poly2.Vertices;

            Debug.Assert(0 <= edge1 && edge1 < poly1.Vertices.Count);

            // Convert normal from poly1's frame into poly2's frame.
            AetherVector2 normal1 = Complex.Multiply(normals1[edge1], ref xf1To2.q);

            // Find support vertex on poly2 for -normal.
            int index = 0;
            Fix64 minDot = Settings.MaxFloat;

            for (int i = 0; i < count2; ++i)
            {
                Fix64 dot = MathUtils.Dot(vertices2[i], ref normal1);
                if (dot < minDot)
                {
                    minDot = dot;
                    index = i;
                }
            }

            AetherVector2 v1 = Transform.Multiply(vertices1[edge1], ref xf1To2);
            AetherVector2 v2 = vertices2[index];
            Fix64 separation = MathUtils.Dot(v2 - v1, ref normal1);

            return separation;
        }

        /// <summary>
        /// Find the max separation between poly1 and poly2 using edge normals from poly1.
        /// </summary>
        /// <param name="edgeIndex">Index of the edge.</param>
        /// <param name="poly1">The poly1.</param>
        /// <param name="xf1">The XF1.</param>
        /// <param name="poly2">The poly2.</param>
        /// <param name="xf2">The XF2.</param>
        /// <returns></returns>
        private static Fix64 FindMaxSeparation(out int edgeIndex, PolygonShape poly1, ref Transform xf1, PolygonShape poly2, ref Transform xf2)
        {
            int count1 = poly1.Vertices.Count;
            List<AetherVector2> normals1 = poly1.Normals;

            var xf1To2 = Transform.Divide(ref xf1, ref xf2);

            // Vector pointing from the centroid of poly1 to the centroid of poly2.
            AetherVector2 c2local = Transform.Divide(poly2.MassData.Centroid, ref xf1To2);
            AetherVector2 dLocal1 = c2local - poly1.MassData.Centroid;            

            // Find edge normal on poly1 that has the largest projection onto d.
            int edge = 0;
            Fix64 maxDot = -Settings.MaxFloat;
            for (int i = 0; i < count1; ++i)
            {
                Fix64 dot = MathUtils.Dot(normals1[i], ref dLocal1);
                if (dot > maxDot)
                {
                    maxDot = dot;
                    edge = i;
                }
            }

            // Get the separation for the edge normal.
            Fix64 s = EdgeSeparation(poly1, ref xf1To2, edge, poly2);

            // Check the separation for the previous edge normal.
            int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
            Fix64 sPrev = EdgeSeparation(poly1, ref xf1To2, prevEdge, poly2);

            // Check the separation for the next edge normal.
            int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
            Fix64 sNext = EdgeSeparation(poly1, ref xf1To2, nextEdge, poly2);

            // Find the best edge and the search direction.
            int bestEdge;
            Fix64 bestSeparation;
            int increment;
            if (sPrev > s && sPrev > sNext)
            {
                increment = -1;
                bestEdge = prevEdge;
                bestSeparation = sPrev;
            }
            else if (sNext > s)
            {
                increment = 1;
                bestEdge = nextEdge;
                bestSeparation = sNext;
            }
            else
            {
                edgeIndex = edge;
                return s;
            }

            // Perform a local search for the best edge normal.
            for (; ; )
            {
                if (increment == -1)
                    edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
                else
                    edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;

                s = EdgeSeparation(poly1, ref xf1To2, edge, poly2);

                if (s > bestSeparation)
                {
                    bestEdge = edge;
                    bestSeparation = s;
                }
                else
                {
                    break;
                }
            }

            edgeIndex = bestEdge;
            return bestSeparation;
        }

        private static void FindIncidentEdge(out FixedArray2<ClipVertex> c, PolygonShape poly1, ref Transform xf1, int edge1, PolygonShape poly2, ref Transform xf2)
        {
            c = new FixedArray2<ClipVertex>();
            Vertices normals1 = poly1.Normals;

            int count2 = poly2.Vertices.Count;
            Vertices vertices2 = poly2.Vertices;
            Vertices normals2 = poly2.Normals;

            Debug.Assert(0 <= edge1 && edge1 < poly1.Vertices.Count);

            // Get the normal of the reference edge in poly2's frame.
            AetherVector2 normal1 = Complex.Divide(Complex.Multiply(normals1[edge1], ref xf1.q), ref xf2.q);


            // Find the incident edge on poly2.
            int index = 0;
            Fix64 minDot = Settings.MaxFloat;
            for (int i = 0; i < count2; ++i)
            {
                Fix64 dot = AetherVector2.Dot(normal1, normals2[i]);
                if (dot < minDot)
                {
                    minDot = dot;
                    index = i;
                }
            }

            // Build the clip vertices for the incident edge.
            int i1 = index;
            int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

            ClipVertex cv0 = c[0];

            cv0.V = Transform.Multiply(vertices2[i1], ref xf2);
            cv0.ID.Features.IndexA = (byte)edge1;
            cv0.ID.Features.IndexB = (byte)i1;
            cv0.ID.Features.TypeA = (byte)ContactFeatureType.Face;
            cv0.ID.Features.TypeB = (byte)ContactFeatureType.Vertex;

            c[0] = cv0;

            ClipVertex cv1 = c[1];
            cv1.V = Transform.Multiply(vertices2[i2], ref xf2);
            cv1.ID.Features.IndexA = (byte)edge1;
            cv1.ID.Features.IndexB = (byte)i2;
            cv1.ID.Features.TypeA = (byte)ContactFeatureType.Face;
            cv1.ID.Features.TypeB = (byte)ContactFeatureType.Vertex;

            c[1] = cv1;
        }
    }
}