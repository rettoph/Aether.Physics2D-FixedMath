/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

using FixedMath.NET;
using System.Collections.Generic;
using tainicom.Aether.Physics2D.Collision;
using tainicom.Aether.Physics2D.Collision.Shapes;
using tainicom.Aether.Physics2D.Common;
using tainicom.Aether.Physics2D.Common.PhysicsLogic;
using tainicom.Aether.Physics2D.Dynamics;
#if XNAAPI
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Controllers
{
    public sealed class BuoyancyController : Controller
    {
        /// <summary>
        /// Controls the rotational drag that the fluid exerts on the bodies within it. Use higher values will simulate thick fluid, like honey, lower values to
        /// simulate water-like fluids. 
        /// </summary>
        public Fix64 AngularDragCoefficient;

        /// <summary>
        /// Density of the fluid. Higher values will make things more buoyant, lower values will cause things to sink.
        /// </summary>
        public Fix64 Density;

        /// <summary>
        /// Controls the linear drag that the fluid exerts on the bodies within it.  Use higher values will simulate thick fluid, like honey, lower values to
        /// simulate water-like fluids.
        /// </summary>
        public Fix64 LinearDragCoefficient;

        /// <summary>
        /// Acts like waterflow. Defaults to 0,0.
        /// </summary>
        public AetherVector2 Velocity;

        private AABB _container;

        private AetherVector2 _gravity;
        private AetherVector2 _normal;
        private Fix64 _offset;
        private ICollection<Body> _uniqueBodies = new List<Body>();

        /// <summary>
        /// Initializes a new instance of the <see cref="BuoyancyController"/> class.
        /// </summary>
        /// <param name="container">Only bodies inside this AABB will be influenced by the controller</param>
        /// <param name="density">Density of the fluid</param>
        /// <param name="linearDragCoefficient">Linear drag coefficient of the fluid</param>
        /// <param name="rotationalDragCoefficient">Rotational drag coefficient of the fluid</param>
        /// <param name="gravity">The direction gravity acts. Buoyancy force will act in opposite direction of gravity.</param>
        public BuoyancyController(AABB container, Fix64 density, Fix64 linearDragCoefficient, Fix64 rotationalDragCoefficient, AetherVector2 gravity)
        {
            Container = container;
            _normal = new AetherVector2(Fix64.Zero, Fix64.One);
            Density = density;
            LinearDragCoefficient = linearDragCoefficient;
            AngularDragCoefficient = rotationalDragCoefficient;
            _gravity = gravity;
        }

        public AABB Container
        {
            get { return _container; }
            set
            {
                _container = value;
                _offset = _container.UpperBound.Y;
            }
        }

        public override void Update(Fix64 dt)
        {
            _uniqueBodies.Clear();
            World.QueryAABB(fixture =>
                                {
                                    if (fixture.Body.BodyType == BodyType.Static || !fixture.Body.Awake)
                                        return true;

                                    if (!_uniqueBodies.Contains(fixture.Body))
                                        _uniqueBodies.Add(fixture.Body);

                                    return true;
                                }, ref _container);

            foreach (Body body in _uniqueBodies)
            {
                AetherVector2 areac = AetherVector2.Zero;
                AetherVector2 massc = AetherVector2.Zero;
                Fix64 area = Fix64.Zero;
                Fix64 mass = Fix64.Zero;

                foreach (Fixture fixture in body.FixtureList)
                {
                    if (fixture.Shape.ShapeType != ShapeType.Polygon && fixture.Shape.ShapeType != ShapeType.Circle)
                        continue;

                    Shape shape = fixture.Shape;

                    AetherVector2 sc;
                    Fix64 sarea = shape.ComputeSubmergedArea(ref _normal, _offset, ref body._xf, out sc);
                    area += sarea;
                    areac.X += sarea * sc.X;
                    areac.Y += sarea * sc.Y;

                    mass += sarea * shape.Density;
                    massc.X += sarea * sc.X * shape.Density;
                    massc.Y += sarea * sc.Y * shape.Density;
                }

                areac.X /= area;
                areac.Y /= area;
                massc.X /= mass;
                massc.Y /= mass;

                if (area < Settings.Epsilon)
                    continue;

                //Buoyancy
                AetherVector2 buoyancyForce = -Density * area * _gravity;
                body.ApplyForce(buoyancyForce, massc);

                //Linear drag
                AetherVector2 dragVelocity = body.GetLinearVelocityFromWorldPoint(areac) - Velocity;
                AetherVector2 dragForce = dragVelocity * (-LinearDragCoefficient * area);
                body.ApplyForce(dragForce, areac);

                //Angular drag
                body.ApplyTorque(-body.Inertia / body.Mass * area * body.AngularVelocity * AngularDragCoefficient);
            }
        }
    }
}