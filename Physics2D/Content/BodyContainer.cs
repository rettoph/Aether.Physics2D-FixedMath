/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

using FixedMath.NET;
using System.Collections.Generic;
using tainicom.Aether.Physics2D.Collision.Shapes;
using tainicom.Aether.Physics2D.Dynamics;

namespace tainicom.Aether.Physics2D.Content
{
    public class FixtureTemplate
    {
        public Shape Shape;
        public Fix64 Restitution;
        public Fix64 Friction;
        public string Name;
    }

    public class BodyTemplate
    {
        public List<FixtureTemplate> Fixtures;
        public Fix64 Mass;
        public BodyType BodyType;

        public BodyTemplate()
        {
            Fixtures = new List<FixtureTemplate>();
        }

        public Body Create(World world)
        {
            Body body = world.CreateBody();
            body.BodyType = BodyType;

            foreach (FixtureTemplate fixtureTemplate in Fixtures)
            {
                Fixture fixture = body.CreateFixture(fixtureTemplate.Shape);
                fixture.Tag = fixtureTemplate.Name;
                fixture.Restitution = fixtureTemplate.Restitution;
                fixture.Friction = fixtureTemplate.Friction;
            }

            if (Mass > Fix64.Zero)
                body.Mass = Mass;

            return body;
        }

    }

    public class BodyContainer : Dictionary<string, BodyTemplate> { }
}
