using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Text;
using tainicom.Aether.Physics2D.Common;

namespace tainicom.Aether.Physics2D.Diagnostics.Extensions
{
    internal static class AetherVector2Extensions
    {
        public static Vector2 ToXnaVector2(this AetherVector2 vector)
        {
            Vector2 instance = new Vector2((float)vector.X, (float)vector.Y);

            return instance;
        }
    }
}
