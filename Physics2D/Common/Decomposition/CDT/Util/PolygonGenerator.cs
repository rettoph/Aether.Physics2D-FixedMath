/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

/* Poly2Tri
 * Copyright (c) 2009-2010, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

using FixedMath.NET;
using System;
using tainicom.Aether.Physics2D.Common.Decomposition.CDT.Polygon;

namespace tainicom.Aether.Physics2D.Common.Decomposition.CDT.Util
{
    internal class PolygonGenerator
    {
        private static readonly Random RNG = new Random();

        private static Fix64 PI_2 = Fix64Constants.Two * Fix64.Pi;

        public static Polygon.Polygon RandomCircleSweep(Fix64 scale, int vertexCount)
        {
            PolygonPoint point;
            PolygonPoint[] points;
            Fix64 radius = scale/Fix64Constants.Four;

            points = new PolygonPoint[vertexCount];
            for (int i = 0; i < vertexCount; i++)
            {
                do
                {
                    if (i%250 == 0)
                    {
                        radius += scale/Fix64Constants.Two * (Fix64Constants.PointFive - Fix64.FromRaw(BitConverter.DoubleToInt64Bits(RNG.NextDouble())));
                    }
                    else if (i%50 == 0)
                    {
                        radius += scale/Fix64Constants.Five*(Fix64Constants.PointFive - Fix64.FromRaw(BitConverter.DoubleToInt64Bits(RNG.NextDouble())));
                    }
                    else
                    {
                        radius += Fix64Constants.TwentyFive*scale/(Fix64)vertexCount*(Fix64Constants.PointFive - Fix64.FromRaw(BitConverter.DoubleToInt64Bits(RNG.NextDouble())));
                    }
                    radius = radius > scale/Fix64Constants.Two ? scale/Fix64Constants.Two : radius;
                    radius = radius < scale/Fix64Constants.Ten ? scale/Fix64Constants.Ten : radius;
                } while (radius < scale/Fix64Constants.Ten || radius > scale/Fix64Constants.Two);
                point = new PolygonPoint(radius* Fix64.Cos((PI_2*(Fix64)i)/(Fix64)vertexCount),
                                         radius* Fix64.Sin((PI_2*(Fix64)i)/(Fix64)vertexCount));
                points[i] = point;
            }
            return new Polygon.Polygon(points);
        }

        public static Polygon.Polygon RandomCircleSweep2(Fix64 scale, int vertexCount)
        {
            PolygonPoint point;
            PolygonPoint[] points;
            Fix64 radius = scale/Fix64Constants.Four;

            points = new PolygonPoint[vertexCount];
            for (int i = 0; i < vertexCount; i++)
            {
                do
                {
                    radius += scale/Fix64Constants.Five*(Fix64Constants.PointFive - Fix64.FromRaw(BitConverter.DoubleToInt64Bits(RNG.NextDouble())));
                    radius = radius > scale/Fix64Constants.Two ? scale/Fix64Constants.Two : radius;
                    radius = radius < scale/Fix64Constants.Ten ? scale/Fix64Constants.Ten : radius;
                } while (radius < scale/Fix64Constants.Ten || radius > scale/Fix64Constants.Two);
                point = new PolygonPoint(radius* Fix64.Cos((PI_2*(Fix64)i)/(Fix64)vertexCount),
                                         radius* Fix64.Sin((PI_2*(Fix64)i)/(Fix64)vertexCount));
                points[i] = point;
            }
            return new Polygon.Polygon(points);
        }
    }
}