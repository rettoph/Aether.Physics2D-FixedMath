﻿/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

using FixedMath.NET;
using tainicom.Aether.Physics2D.Common;
#if XNAAPI
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif


namespace tainicom.Aether.Physics2D.Common.ConvexHull
{
    /// <summary>
    /// Creates a convex hull.
    /// Note:
    /// 1. Vertices must be of a simple polygon, i.e. edges do not overlap.
    /// 2. Melkman does not work on point clouds
    /// </summary>
    /// <remarks>
    /// Implemented using Melkman's Convex Hull Algorithm - O(n) time complexity.
    /// Reference: http://www.ams.sunysb.edu/~jsbm/courses/345/melkman.pdf
    /// </remarks>
    public static class Melkman
    {
        //Melkman based convex hull algorithm contributed by Cowdozer

        /// <summary>
        /// Returns a convex hull from the given vertices.
        /// </summary>
        /// <returns>A convex hull in counter clockwise winding order.</returns>
        public static Vertices GetConvexHull(Vertices vertices)
        {
            if (vertices.Count <= 3)
                return vertices;

            //We'll never need a queue larger than the current number of Vertices +1
            //Create double-ended queue
            AetherVector2[] deque = new AetherVector2[vertices.Count + 1];
            int qf = 3, qb = 0; //Queue front index, queue back index

            //Start by placing first 3 vertices in convex CCW order
            int startIndex = 3;
            Fix64 k = MathUtils.Area(vertices[0], vertices[1], vertices[2]);
            if (k == Fix64.Zero)
            {
                //Vertices are collinear.
                deque[0] = vertices[0];
                deque[1] = vertices[2]; //We can skip vertex 1 because it should be between 0 and 2
                deque[2] = vertices[0];
                qf = 2;

                //Go until the end of the collinear sequence of vertices
                for (startIndex = 3; startIndex < vertices.Count; startIndex++)
                {
                    AetherVector2 tmp = vertices[startIndex];
                    if (MathUtils.Area(ref deque[0], ref deque[1], ref tmp) == Fix64.Zero) //This point is also collinear
                        deque[1] = vertices[startIndex];
                    else break;
                }
            }
            else
            {
                deque[0] = deque[3] = vertices[2];
                if (k > Fix64.Zero)
                {
                    //Is Left.  Set deque = {2, 0, 1, 2}
                    deque[1] = vertices[0];
                    deque[2] = vertices[1];
                }
                else
                {
                    //Is Right. Set deque = {2, 1, 0, 2}
                    deque[1] = vertices[1];
                    deque[2] = vertices[0];
                }
            }

            int qfm1 = qf == 0 ? deque.Length - 1 : qf - 1;
            int qbm1 = qb == deque.Length - 1 ? 0 : qb + 1;

            //Add vertices one at a time and adjust convex hull as needed
            for (int i = startIndex; i < vertices.Count; i++)
            {
                AetherVector2 nextPt = vertices[i];

                //Ignore if it is already within the convex hull we have constructed
                if (MathUtils.Area(ref deque[qfm1], ref deque[qf], ref nextPt) > Fix64.Zero && MathUtils.Area(ref deque[qb], ref deque[qbm1], ref nextPt) > Fix64.Zero)
                    continue;

                //Pop front until convex
                while (!(MathUtils.Area(ref deque[qfm1], ref deque[qf], ref nextPt) > Fix64.Zero))
                {
                    //Pop the front element from the queue
                    qf = qfm1; //qf--;
                    qfm1 = qf == 0 ? deque.Length - 1 : qf - 1; //qfm1 = qf - 1;
                }

                //Add vertex to the front of the queue
                qf = qf == deque.Length - 1 ? 0 : qf + 1; //qf++;
                qfm1 = qf == 0 ? deque.Length - 1 : qf - 1; //qfm1 = qf - 1;
                deque[qf] = nextPt;

                //Pop back until convex
                while (!(MathUtils.Area(ref deque[qb], ref deque[qbm1], ref nextPt) > Fix64.Zero))
                {
                    //Pop the back element from the queue
                    qb = qbm1; //qb++;
                    qbm1 = qb == deque.Length - 1 ? 0 : qb + 1; //qbm1 = qb + 1;
                }
                //Add vertex to the back of the queue
                qb = qb == 0 ? deque.Length - 1 : qb - 1; //qb--;
                qbm1 = qb == deque.Length - 1 ? 0 : qb + 1; //qbm1 = qb + 1;
                deque[qb] = nextPt;
            }

            //Create the convex hull from what is left in the deque
            if (qb < qf)
            {
                Vertices convexHull = new Vertices(qf);

                for (int i = qb; i < qf; i++)
                    convexHull.Add(deque[i]);

                return convexHull;
            }
            else
            {
                Vertices convexHull = new Vertices(qf + deque.Length);

                for (int i = 0; i < qf; i++)
                    convexHull.Add(deque[i]);

                for (int i = qb; i < deque.Length; i++)
                    convexHull.Add(deque[i]);

                return convexHull;
            }
        }
    }
}