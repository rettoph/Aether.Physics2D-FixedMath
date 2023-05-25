// Copyright (c) 2017 Kastellanos Nikolaos

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
using tainicom.Aether.Physics2D.Dynamics.Joints;
#if XNAAPI
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Dynamics
{
    public partial class World
    {
        public virtual Body CreateBody(AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            Body body = new Body();
            body.Position = position;
            body.Rotation = rotation;            
            body.BodyType = bodyType;
            
#if LEGACY_ASYNCADDREMOVE
            AddAsync(body);
#else
            Add(body);
#endif

            return body;
        }

        public virtual Body CreateBody(BodyType bodyType = BodyType.Static)
        {
            return this.CreateBody(AetherVector2.Zero, Fix64.Zero);
        }

        public Body CreateEdge(AetherVector2 start, AetherVector2 end)
        {
            Body body = CreateBody();

            body.CreateEdge(start, end);
            return body;
        }

        public Body CreateChainShape(Vertices vertices, AetherVector2 position = new AetherVector2())
        {
            Body body = CreateBody(position, Fix64.Zero);

            body.CreateChainShape(vertices);
            return body;
        }

        public Body CreateLoopShape(Vertices vertices, AetherVector2 position = new AetherVector2())
        {
            Body body = CreateBody(position, Fix64.Zero);

            body.CreateLoopShape(vertices);
            return body;
        }

        public Body CreateRectangle(Fix64 width, Fix64 height, Fix64 density, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            if (width <= Fix64.Zero)
                throw new ArgumentOutOfRangeException("width", "Width must be more than 0 meters");

            if (height <= Fix64.Zero)
                throw new ArgumentOutOfRangeException("height", "Height must be more than 0 meters");

            Body body = CreateBody(position, rotation, bodyType);

            Vertices rectangleVertices = PolygonTools.CreateRectangle(width / Fix64Constants.Two, height / Fix64Constants.Two);
            body.CreatePolygon(rectangleVertices, density);

            return body;
        }

        public Body CreateCircle(Fix64 radius, Fix64 density, AetherVector2 position = new AetherVector2(), BodyType bodyType = BodyType.Static)
        {
            Body body = CreateBody(position, Fix64.Zero, bodyType);
            body.CreateCircle(radius, density);
            return body;
        }

        public Body CreateEllipse(Fix64 xRadius, Fix64 yRadius, int edges, Fix64 density, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            Body body = CreateBody(position, rotation, bodyType);
            body.CreateEllipse(xRadius, yRadius, edges, density);
            return body;
        }

        public Body CreatePolygon(Vertices vertices, Fix64 density, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            Body body = CreateBody(position, rotation, bodyType);
            body.CreatePolygon(vertices, density);
            return body;
        }

        public Body CreateCompoundPolygon(List<Vertices> list, Fix64 density, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            //We create a single body
            Body body = CreateBody(position, rotation, bodyType);
            body.CreateCompoundPolygon(list, density);
            return body;
        }

        public Body CreateGear(Fix64 radius, int numberOfTeeth, Fix64 tipPercentage, Fix64 toothHeight, Fix64 density, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            Vertices gearPolygon = PolygonTools.CreateGear(radius, numberOfTeeth, tipPercentage, toothHeight);

            //Gears can in some cases be convex
            if (!gearPolygon.IsConvex())
            {
                //Decompose the gear:
                List<Vertices> list = Triangulate.ConvexPartition(gearPolygon, TriangulationAlgorithm.Earclip);

                return CreateCompoundPolygon(list, density, position, rotation, bodyType);
            }

            return CreatePolygon(gearPolygon, density, position, rotation, bodyType);
        }

        public Body CreateCapsule(Fix64 height, Fix64 topRadius, int topEdges, Fix64 bottomRadius, int bottomEdges, Fix64 density, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            Vertices verts = PolygonTools.CreateCapsule(height, topRadius, topEdges, bottomRadius, bottomEdges);

            //There are too many vertices in the capsule. We decompose it.
            if (verts.Count >= Settings.MaxPolygonVertices)
            {
                List<Vertices> vertList = Triangulate.ConvexPartition(verts, TriangulationAlgorithm.Earclip);
                return CreateCompoundPolygon(vertList, density, position, rotation, bodyType);
            }

            return CreatePolygon(verts, density, position, rotation, bodyType);
        }

        public Body CreateCapsule(Fix64 height, Fix64 endRadius, Fix64 density, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            //Create the middle rectangle
            Vertices rectangle = PolygonTools.CreateRectangle(endRadius, height / Fix64Constants.Two);

            List<Vertices> list = new List<Vertices>();
            list.Add(rectangle);

            Body body = CreateCompoundPolygon(list, density, position, rotation, bodyType);
            body.CreateCircle(endRadius, density, new AetherVector2(Fix64.Zero, height / Fix64Constants.Two));
            body.CreateCircle(endRadius, density, new AetherVector2(Fix64.Zero, -(height / Fix64Constants.Two)));

            //Create the two circles
            //CircleShape topCircle = new CircleShape(endRadius, density);
            //topCircle.Position = new Vector2(0, height / 2);
            //body.CreateFixture(topCircle);

            //CircleShape bottomCircle = new CircleShape(endRadius, density);
            //bottomCircle.Position = new Vector2(0, -(height / 2));
            //body.CreateFixture(bottomCircle);
            return body;
        }

        public Body CreateRoundedRectangle(Fix64 width, Fix64 height, Fix64 xRadius, Fix64 yRadius, int segments, Fix64 density, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            Vertices verts = PolygonTools.CreateRoundedRectangle(width, height, xRadius, yRadius, segments);

            //There are too many vertices in the capsule. We decompose it.
            if (verts.Count >= Settings.MaxPolygonVertices)
            {
                List<Vertices> vertList = Triangulate.ConvexPartition(verts, TriangulationAlgorithm.Earclip);
                return CreateCompoundPolygon(vertList, density, position, rotation, bodyType);
            }

            return CreatePolygon(verts, density, position, rotation, bodyType);
        }

        public Body CreateLineArc(Fix64 radians, int sides, Fix64 radius, bool closed, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            Body body = CreateBody(position, rotation, bodyType);
            body.CreateLineArc(radians, sides, radius, closed);
            return body;
        }

        public Body CreateSolidArc(Fix64 density, Fix64 radians, int sides, Fix64 radius, AetherVector2 position, Fix64 rotation, BodyType bodyType = BodyType.Static)
        {
            Body body = CreateBody(position, rotation, bodyType);
            body.CreateSolidArc(density, radians, sides, radius);

            return body;
        }

        /// <summary>
        /// Creates a chain.
        /// </summary>
        /// <param name="world">The world.</param>
        /// <param name="start">The start.</param>
        /// <param name="end">The end.</param>
        /// <param name="linkWidth">The width.</param>
        /// <param name="linkHeight">The height.</param>
        /// <param name="numberOfLinks">The number of links.</param>
        /// <param name="linkDensity">The link density.</param>
        /// <param name="attachRopeJoint">Creates a rope joint between start and end. This enforces the length of the rope. Said in another way: it makes the rope less bouncy.</param>
        /// <returns></returns>
        public Path CreateChain(AetherVector2 start, AetherVector2 end, Fix64 linkWidth, Fix64 linkHeight, int numberOfLinks, Fix64 linkDensity, bool attachRopeJoint)
        {
            System.Diagnostics.Debug.Assert(numberOfLinks >= 2);

            //Chain start / end
            Path path = new Path();
            path.Add(start);
            path.Add(end);

            //A single chainlink
            PolygonShape shape = new PolygonShape(PolygonTools.CreateRectangle(linkWidth, linkHeight), linkDensity);

            //Use PathManager to create all the chainlinks based on the chainlink created before.
            List<Body> chainLinks = PathManager.EvenlyDistributeShapesAlongPath(this, path, shape, BodyType.Dynamic, numberOfLinks);

            //TODO
            //if (fixStart)
            //{
            //    //Fix the first chainlink to the world
            //    JointFactory.CreateFixedRevoluteJoint(this, chainLinks[0], new Vector2(0, -(linkHeight / 2)),
            //                                          chainLinks[0].Position);
            //}

            //if (fixEnd)
            //{
            //    //Fix the last chainlink to the world
            //    JointFactory.CreateFixedRevoluteJoint(this, chainLinks[chainLinks.Count - 1],
            //                                          new Vector2(0, (linkHeight / 2)),
            //                                          chainLinks[chainLinks.Count - 1].Position);
            //}

            //Attach all the chainlinks together with a revolute joint
            PathManager.AttachBodiesWithRevoluteJoint(this, chainLinks, new AetherVector2(Fix64.Zero, -linkHeight), new AetherVector2(Fix64.Zero, linkHeight), false, false);

            if (attachRopeJoint)
                JointFactory.CreateRopeJoint(this, chainLinks[0], chainLinks[chainLinks.Count - 1], AetherVector2.Zero, AetherVector2.Zero);

            return path;
        }



    }
}