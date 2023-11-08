/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

using FixedMath.NET;
using System;
using System.Collections.Generic;
using tainicom.Aether.Physics2D.Collision.Shapes;
using tainicom.Aether.Physics2D.Common;
using tainicom.Aether.Physics2D.Common.Decomposition;
using tainicom.Aether.Physics2D.Dynamics;
#if XNAAPI
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Dynamics
{
    // An easy to use factory for creating bodies
    public partial class Body
    {
        /// <summary>
        /// Creates a fixture and attach it to this body.
        /// If the density is non-zero, this function automatically updates the mass of the body.
        /// Contacts are not created until the next time step.
        /// Warning: This method is locked during callbacks.
        /// </summary>
        /// <param name="shape">The shape.</param>
        /// <param name="userData">Application specific data</param>
        /// <returns></returns>
        public virtual Fixture CreateFixture(Shape shape)
        {
            Fixture fixture = new Fixture(shape);
            Add(fixture);
            return fixture;
        }

        public Fixture CreateEdge(AetherVector2 start, AetherVector2 end)
        {
            EdgeShape edgeShape = new EdgeShape(start, end);
            return CreateFixture(edgeShape);
        }

        public Fixture CreateChainShape(Vertices vertices)
        {
            ChainShape shape = new ChainShape(vertices);
            return CreateFixture(shape);
        }

        public Fixture CreateLoopShape(Vertices vertices)
        {
            ChainShape shape = new ChainShape(vertices, true);
            return CreateFixture(shape);
        }

        public Fixture CreateRectangle(Fix64 width, Fix64 height, Fix64 density, AetherVector2 offset)
        {
            Vertices rectangleVertices = PolygonTools.CreateRectangle(width / Fix64Constants.Two, height / Fix64Constants.Two);
            rectangleVertices.Translate(ref offset);
            PolygonShape rectangleShape = new PolygonShape(rectangleVertices, density);
            return CreateFixture(rectangleShape);
        }

        public Fixture CreateCircle(Fix64 radius, Fix64 density)
        {
            if (radius <= Fix64.Zero)
                throw new ArgumentOutOfRangeException("radius", "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius, density);
            return CreateFixture(circleShape);
        }

        public Fixture CreateCircle(Fix64 radius, Fix64 density, AetherVector2 offset)
        {
            if (radius <= Fix64.Zero)
                throw new ArgumentOutOfRangeException("radius", "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius, density);
            circleShape.Position = offset;
            return CreateFixture(circleShape);
        }

        public Fixture CreatePolygon(Vertices vertices, Fix64 density)
        {
            if (vertices.Count <= 1)
                throw new ArgumentOutOfRangeException("vertices", "Too few points to be a polygon");

            PolygonShape polygon = new PolygonShape(vertices, density);
            return CreateFixture(polygon);
        }

        public Fixture CreateEllipse(Fix64 xRadius, Fix64 yRadius, int edges, Fix64 density)
        {
            if (xRadius <= Fix64.Zero)
                throw new ArgumentOutOfRangeException("xRadius", "X-radius must be more than 0");

            if (yRadius <= Fix64.Zero)
                throw new ArgumentOutOfRangeException("yRadius", "Y-radius must be more than 0");

            Vertices ellipseVertices = PolygonTools.CreateEllipse(xRadius, yRadius, edges);
            PolygonShape polygonShape = new PolygonShape(ellipseVertices, density);
            return CreateFixture(polygonShape);
        }

        public List<Fixture> CreateCompoundPolygon(List<Vertices> list, Fix64 density)
        {
            List<Fixture> res = new List<Fixture>(list.Count);

            //Then we create several fixtures using the body
            foreach (Vertices vertices in list)
            {
                if (vertices.Count == 2)
                {
                    EdgeShape shape = new EdgeShape(vertices[0], vertices[1]);
                    res.Add(CreateFixture(shape));
                }
                else
                {
                    PolygonShape shape = new PolygonShape(vertices, density);
                    res.Add(CreateFixture(shape));
                }
            }

            return res;
        }

        public Fixture CreateLineArc(Fix64 radians, int sides, Fix64 radius, bool closed)
        {
            Vertices arc = PolygonTools.CreateArc(radians, sides, radius);
            arc.Rotate((Constant.Pi - radians) / Fix64Constants.Two);
            return closed ? CreateLoopShape(arc) : CreateChainShape(arc);
        }

        public List<Fixture> CreateSolidArc(Fix64 density, Fix64 radians, int sides, Fix64 radius)
        {
            Vertices arc = PolygonTools.CreateArc(radians, sides, radius);
            arc.Rotate((Constant.Pi - radians) / Fix64Constants.Two);

            //Close the arc
            arc.Add(arc[0]);

            List<Vertices> triangles = Triangulate.ConvexPartition(arc, TriangulationAlgorithm.Earclip);

            return CreateCompoundPolygon(triangles, density);
        }
    }
}