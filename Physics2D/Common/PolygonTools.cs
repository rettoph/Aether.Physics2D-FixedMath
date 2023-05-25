/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

using FixedMath.NET;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using tainicom.Aether.Physics2D.Common.TextureTools;
#if XNAAPI
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Common
{
    public static class PolygonTools
    {
        /// <summary>
        /// Build vertices to represent an axis-aligned box.
        /// </summary>
        /// <param name="hx">the half-width.</param>
        /// <param name="hy">the half-height.</param>
        public static Vertices CreateRectangle(Fix64 hx, Fix64 hy)
        {
            Vertices vertices = new Vertices(4);
            vertices.Add(new AetherVector2(-hx, -hy));
            vertices.Add(new AetherVector2(hx, -hy));
            vertices.Add(new AetherVector2(hx, hy));
            vertices.Add(new AetherVector2(-hx, hy));

            return vertices;
        }

        /// <summary>
        /// Build vertices to represent an oriented box.
        /// </summary>
        /// <param name="hx">the half-width.</param>
        /// <param name="hy">the half-height.</param>
        /// <param name="center">the center of the box in local coordinates.</param>
        /// <param name="angle">the rotation of the box in local coordinates.</param>
        public static Vertices CreateRectangle(Fix64 hx, Fix64 hy, AetherVector2 center, Fix64 angle)
        {
            Vertices vertices = CreateRectangle(hx, hy);

            Transform xf = new Transform(center, angle);

            // Transform vertices
            for (int i = 0; i < 4; ++i)
            {
                vertices[i] = Transform.Multiply(vertices[i], ref xf);
            }

            return vertices;
        }

        //Rounded rectangle contributed by Jonathan Smars - jsmars@gmail.com

        /// <summary>
        /// Creates a rounded rectangle with the specified width and height.
        /// </summary>
        /// <param name="width">The width.</param>
        /// <param name="height">The height.</param>
        /// <param name="xRadius">The rounding X radius.</param>
        /// <param name="yRadius">The rounding Y radius.</param>
        /// <param name="segments">The number of segments to subdivide the edges.</param>
        /// <returns></returns>
        public static Vertices CreateRoundedRectangle(Fix64 width, Fix64 height, Fix64 xRadius, Fix64 yRadius,
                                                      int segments)
        {
            if (yRadius > height / Fix64Constants.Two || xRadius > width / Fix64Constants.Two)
                throw new Exception("Rounding amount can't be more than half the height and width respectively.");
            if (segments < 0)
                throw new Exception("Segments must be zero or more.");

            //We need at least 8 vertices to create a rounded rectangle
            Debug.Assert(Settings.MaxPolygonVertices >= 8);

            Vertices vertices = new Vertices();
            if (segments == 0)
            {
                vertices.Add(new AetherVector2(width * Fix64Constants.PointFive - xRadius, -height * Fix64Constants.PointFive));
                vertices.Add(new AetherVector2(width * Fix64Constants.PointFive, -height * Fix64Constants.PointFive + yRadius));

                vertices.Add(new AetherVector2(width * Fix64Constants.PointFive, height * Fix64Constants.PointFive - yRadius));
                vertices.Add(new AetherVector2(width * Fix64Constants.PointFive - xRadius, height * Fix64Constants.PointFive));

                vertices.Add(new AetherVector2(-width * Fix64Constants.PointFive + xRadius, height * Fix64Constants.PointFive));
                vertices.Add(new AetherVector2(-width * Fix64Constants.PointFive, height * Fix64Constants.PointFive - yRadius));

                vertices.Add(new AetherVector2(-width * Fix64Constants.PointFive, -height * Fix64Constants.PointFive + yRadius));
                vertices.Add(new AetherVector2(-width * Fix64Constants.PointFive + xRadius, -height * Fix64Constants.PointFive));
            }
            else
            {
                int numberOfEdges = (segments * 4 + 8);

                Fix64 stepSize = Constant.Tau / (Fix64)(numberOfEdges - 4);
                int perPhase = numberOfEdges / 4;

                AetherVector2 posOffset = new AetherVector2(width / Fix64Constants.Two - xRadius, height / Fix64Constants.Two - yRadius);
                vertices.Add(posOffset + new AetherVector2(xRadius, -yRadius + yRadius));
                short phase = 0;
                for (int i = 1; i < numberOfEdges; i++)
                {
                    if (i - perPhase == 0 || i - perPhase * 3 == 0)
                    {
                        posOffset.X *= -Fix64.One;
                        phase--;
                    }
                    else if (i - perPhase * 2 == 0)
                    {
                        posOffset.Y *= -Fix64.One;
                        phase--;
                    }

                    vertices.Add(posOffset + new AetherVector2(xRadius * Fix64.Cos(stepSize * -(Fix64)(i + phase)),
                                                         -yRadius * Fix64.Sin(stepSize * -(Fix64)(i + phase))));
                }
            }

            return vertices;
        }

        /// <summary>
        /// Set this as a single edge.
        /// </summary>
        /// <param name="start">The first point.</param>
        /// <param name="end">The second point.</param>
        public static Vertices CreateLine(AetherVector2 start, AetherVector2 end)
        {
            Vertices vertices = new Vertices(2);
            vertices.Add(start);
            vertices.Add(end);

            return vertices;
        }

        /// <summary>
        /// Creates a circle with the specified radius and number of edges.
        /// </summary>
        /// <param name="radius">The radius.</param>
        /// <param name="numberOfEdges">The number of edges. The more edges, the more it resembles a circle</param>
        /// <returns></returns>
        public static Vertices CreateCircle(Fix64 radius, int numberOfEdges)
        {
            return CreateEllipse(radius, radius, numberOfEdges);
        }

        /// <summary>
        /// Creates a ellipse with the specified width, height and number of edges.
        /// </summary>
        /// <param name="xRadius">Width of the ellipse.</param>
        /// <param name="yRadius">Height of the ellipse.</param>
        /// <param name="numberOfEdges">The number of edges. The more edges, the more it resembles an ellipse</param>
        /// <returns></returns>
        public static Vertices CreateEllipse(Fix64 xRadius, Fix64 yRadius, int numberOfEdges)
        {
            Vertices vertices = new Vertices();

            Fix64 stepSize = Constant.Tau / (Fix64)numberOfEdges;

            vertices.Add(new AetherVector2(xRadius, Fix64.Zero));
            for (int i = numberOfEdges - 1; i > 0; --i)
                vertices.Add(new AetherVector2(xRadius * Fix64.Cos(stepSize * (Fix64)i),
                                         -yRadius * Fix64.Sin(stepSize * (Fix64)i)));

            return vertices;
        }

        public static Vertices CreateArc(Fix64 radians, int sides, Fix64 radius)
        {
            Debug.Assert(radians > Fix64.Zero, "The arc needs to be larger than 0");
            Debug.Assert(sides > 1, "The arc needs to have more than 1 sides");
            Debug.Assert(radius > Fix64.Zero, "The arc needs to have a radius larger than 0");

            Vertices vertices = new Vertices();

            Fix64 stepSize = radians / (Fix64)sides;
            for (int i = sides - 1; i > 0; i--)
            {
                vertices.Add(new AetherVector2(radius * Fix64.Cos(stepSize * (Fix64)i),
                                         radius * Fix64.Sin(stepSize * (Fix64)i)));
            }

            return vertices;
        }

        //Capsule contributed by Yobiv

        /// <summary>
        /// Creates an capsule with the specified height, radius and number of edges.
        /// A capsule has the same form as a pill capsule.
        /// </summary>
        /// <param name="height">Height (inner height + 2 * radius) of the capsule.</param>
        /// <param name="endRadius">Radius of the capsule ends.</param>
        /// <param name="edges">The number of edges of the capsule ends. The more edges, the more it resembles an capsule</param>
        /// <returns></returns>
        public static Vertices CreateCapsule(Fix64 height, Fix64 endRadius, int edges)
        {
            if (endRadius >= height / Fix64Constants.Two)
                throw new ArgumentException(
                    "The radius must be lower than height / 2. Higher values of radius would create a circle, and not a half circle.",
                    "endRadius");

            return CreateCapsule(height, endRadius, edges, endRadius, edges);
        }

        /// <summary>
        /// Creates an capsule with the specified  height, radius and number of edges.
        /// A capsule has the same form as a pill capsule.
        /// </summary>
        /// <param name="height">Height (inner height + radii) of the capsule.</param>
        /// <param name="topRadius">Radius of the top.</param>
        /// <param name="topEdges">The number of edges of the top. The more edges, the more it resembles an capsule</param>
        /// <param name="bottomRadius">Radius of bottom.</param>
        /// <param name="bottomEdges">The number of edges of the bottom. The more edges, the more it resembles an capsule</param>
        /// <returns></returns>
        public static Vertices CreateCapsule(Fix64 height, Fix64 topRadius, int topEdges, Fix64 bottomRadius,
                                             int bottomEdges)
        {
            if (height <= Fix64.Zero)
                throw new ArgumentException("Height must be longer than 0", "height");

            if (topRadius <= Fix64.Zero)
                throw new ArgumentException("The top radius must be more than 0", "topRadius");

            if (topEdges <= 0)
                throw new ArgumentException("Top edges must be more than 0", "topEdges");

            if (bottomRadius <= Fix64.Zero)
                throw new ArgumentException("The bottom radius must be more than 0", "bottomRadius");

            if (bottomEdges <= 0)
                throw new ArgumentException("Bottom edges must be more than 0", "bottomEdges");

            if (topRadius >= height / Fix64Constants.Two)
                throw new ArgumentException(
                    "The top radius must be lower than height / 2. Higher values of top radius would create a circle, and not a half circle.",
                    "topRadius");

            if (bottomRadius >= height / Fix64Constants.Two)
                throw new ArgumentException(
                    "The bottom radius must be lower than height / 2. Higher values of bottom radius would create a circle, and not a half circle.",
                    "bottomRadius");

            Vertices vertices = new Vertices();

            Fix64 newHeight = (height - topRadius - bottomRadius) * Fix64Constants.PointFive;

            // top
            vertices.Add(new AetherVector2(topRadius, newHeight));

            Fix64 stepSize = Constant.Pi / (Fix64)topEdges;
            for (int i = 1; i < topEdges; i++)
            {
                vertices.Add(new AetherVector2(topRadius * Fix64.Cos(stepSize * (Fix64)i),
                                         topRadius * Fix64.Sin(stepSize * (Fix64)i) + newHeight));
            }

            vertices.Add(new AetherVector2(-topRadius, newHeight));

            // bottom
            vertices.Add(new AetherVector2(-bottomRadius, -newHeight));

            stepSize = Constant.Pi / (Fix64)bottomEdges;
            for (int i = 1; i < bottomEdges; i++)
            {
                vertices.Add(new AetherVector2(-bottomRadius * (Fix64) Fix64.Cos(stepSize * (Fix64)i),
                                         -bottomRadius * (Fix64) Fix64.Sin(stepSize * (Fix64)i) - newHeight));
            }

            vertices.Add(new AetherVector2(bottomRadius, -newHeight));

            return vertices;
        }

        /// <summary>
        /// Creates a gear shape with the specified radius and number of teeth.
        /// </summary>
        /// <param name="radius">The radius.</param>
        /// <param name="numberOfTeeth">The number of teeth.</param>
        /// <param name="tipPercentage">The tip percentage.</param>
        /// <param name="toothHeight">Height of the tooth.</param>
        /// <returns></returns>
        public static Vertices CreateGear(Fix64 radius, int numberOfTeeth, Fix64 tipPercentage, Fix64 toothHeight)
        {
            Vertices vertices = new Vertices();

            Fix64 stepSize = Constant.Tau / (Fix64)numberOfTeeth;
            tipPercentage /= Fix64Constants.OneHundred;
            MathUtils.Clamp(tipPercentage, Fix64.Zero, Fix64.One);
            Fix64 toothTipStepSize = (stepSize / Fix64Constants.Two) * tipPercentage;

            Fix64 toothAngleStepSize = (stepSize - (toothTipStepSize * Fix64Constants.Two)) / Fix64Constants.Two;

            for (int i = numberOfTeeth - 1; i >= 0; --i)
            {
                if (toothTipStepSize > Fix64.Zero)
                {
                    vertices.Add(
                        new AetherVector2(radius *
                                    Fix64.Cos(stepSize * (Fix64)i + toothAngleStepSize * Fix64Constants.Two + toothTipStepSize),
                                    -radius *
                                    Fix64.Sin(stepSize * (Fix64)i + toothAngleStepSize * Fix64Constants.Two + toothTipStepSize)));

                    vertices.Add(
                        new AetherVector2((radius + toothHeight) *
                                    Fix64.Cos(stepSize * (Fix64)i + toothAngleStepSize + toothTipStepSize),
                                    -(radius + toothHeight) *
                                    Fix64.Sin(stepSize * (Fix64)i + toothAngleStepSize + toothTipStepSize)));
                }

                vertices.Add(new AetherVector2((radius + toothHeight) *
                                         Fix64.Cos(stepSize * (Fix64)i + toothAngleStepSize),
                                         -(radius + toothHeight) *
                                         Fix64.Sin(stepSize * (Fix64)i + toothAngleStepSize)));

                vertices.Add(new AetherVector2(radius * Fix64.Cos(stepSize * (Fix64)i),
                                         -radius * Fix64.Sin(stepSize * (Fix64)i)));
            }

            return vertices;
        }

#if XNAAPI
        /// <summary>
        /// Detects the vertices by analyzing the texture data.
        /// </summary>
        /// <param name="data">The texture data.</param>
        /// <param name="width">The texture width.</param>
        /// <returns></returns>
        public static Vertices CreatePolygon(uint[] data, int width)
        {
            return TextureConverter.DetectVertices(data, width);
        }

        /// <summary>
        /// Detects the vertices by analyzing the texture data.
        /// </summary>
        /// <param name="data">The texture data.</param>
        /// <param name="width">The texture width.</param>
        /// <param name="holeDetection">if set to <c>true</c> it will perform hole detection.</param>
        /// <returns></returns>
        public static Vertices CreatePolygon(uint[] data, int width, bool holeDetection)
        {
            return TextureConverter.DetectVertices(data, width, holeDetection);
        }

        /// <summary>
        /// Detects the vertices by analyzing the texture data.
        /// </summary>
        /// <param name="data">The texture data.</param>
        /// <param name="width">The texture width.</param>
        /// <param name="hullTolerance">The hull tolerance.</param>
        /// <param name="alphaTolerance">The alpha tolerance.</param>
        /// <param name="multiPartDetection">if set to <c>true</c> it will perform multi part detection.</param>
        /// <param name="holeDetection">if set to <c>true</c> it will perform hole detection.</param>
        /// <returns></returns>
        public static List<Vertices> CreatePolygon(uint[] data, int width, Fix64 hullTolerance,
                                                   byte alphaTolerance, bool multiPartDetection, bool holeDetection)
        {
            return TextureConverter.DetectVertices(data, width, hullTolerance, alphaTolerance,
                                                   multiPartDetection, holeDetection);
        }
#endif
    }
}