﻿/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

using FixedMath.NET;
using System.Collections.Generic;

namespace tainicom.Aether.Physics2D.Common.Decomposition.Seidel
{
    internal class Edge
    {
        // Pointers used for building trapezoidal map
        public Trapezoid Above;
        public Fix64 B;
        public Trapezoid Below;

        // Montone mountain points
        public HashSet<Point> MPoints;
        public Point P;
        public Point Q;

        // Slope of the line (m)
        public Fix64 Slope;


        public Edge(Point p, Point q)
        {
            P = p;
            Q = q;

            if (q.X - p.X != Fix64.Zero)
                Slope = (q.Y - p.Y) / (q.X - p.X);
            else
                Slope = Fix64.Zero;

            B = p.Y - (p.X * Slope);
            Above = null;
            Below = null;
            MPoints = new HashSet<Point>();
            MPoints.Add(p);
            MPoints.Add(q);
        }

        public bool IsAbove(Point point)
        {
            return P.Orient2D(Q, point) < Fix64.Zero;
        }

        public bool IsBelow(Point point)
        {
            return P.Orient2D(Q, point) > Fix64.Zero;
        }

        public void AddMpoint(Point point)
        {
            foreach (Point mp in MPoints)
            {
                if (!mp.Neq(point))
                    return;
            }

            MPoints.Add(point);
        }
    }
}