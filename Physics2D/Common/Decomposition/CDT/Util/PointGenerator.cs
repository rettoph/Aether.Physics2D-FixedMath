/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

using FixedMath.NET;
using System;
using System.Collections.Generic;

namespace tainicom.Aether.Physics2D.Common.Decomposition.CDT.Util
{
    internal class PointGenerator
    {
        private static readonly Random RNG = new Random();

        public static List<TriangulationPoint> UniformDistribution(int n, Fix64 scale)
        {
            List<TriangulationPoint> points = new List<TriangulationPoint>();
            for (int i = 0; i < n; i++)
            {
                points.Add(new TriangulationPoint(scale*(Fix64Constants.PointFive - Fix64.FromRaw(BitConverter.DoubleToInt64Bits(RNG.NextDouble()))), scale*(Fix64Constants.PointFive - Fix64.FromRaw(BitConverter.DoubleToInt64Bits(RNG.NextDouble())))));
            }
            return points;
        }

        public static List<TriangulationPoint> UniformGrid(int n, Fix64 scale)
        {
            Fix64 x = Fix64.Zero;
            Fix64 size = scale/(Fix64)n;
            Fix64 halfScale = Fix64Constants.PointFive * scale;

            List<TriangulationPoint> points = new List<TriangulationPoint>();
            for (int i = 0; i < n + 1; i++)
            {
                x = halfScale - (Fix64)i * size;
                for (int j = 0; j < n + 1; j++)
                {
                    points.Add(new TriangulationPoint(x, halfScale - (Fix64)j * size));
                }
            }
            return points;
        }
    }
}