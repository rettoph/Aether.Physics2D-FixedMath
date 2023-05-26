// Copyright (c) 2017 Kastellanos Nikolaos

using FixedMath.NET;
using System;

namespace tainicom.Aether.Physics2D.Common
{
    public struct AetherVector2 : IEquatable<AetherVector2>
    {
        private static readonly AetherVector2 _zero = new AetherVector2(Fix64.Zero, Fix64.Zero);
        private static readonly AetherVector2 _one  = new AetherVector2(Fix64.One, Fix64.One);
        private static readonly AetherVector2 _x  = new AetherVector2(Fix64.One, Fix64.Zero);
        private static readonly AetherVector2 _Y  = new AetherVector2(Fix64.Zero, Fix64.One);


        public Fix64 X;
        public Fix64 Y;

        public static AetherVector2 Zero { get { return _zero; } }
        public static AetherVector2 One { get { return _one; } }
        public static AetherVector2 UnitX { get { return _x; } }
        public static AetherVector2 UnitY { get { return _Y; } }


        public AetherVector2(Fix64 x, Fix64 y)
        {
            this.X = x;
            this.Y = y;
        }

        public AetherVector2(Fix64 xy)
        {
            this.X = xy;
            this.Y = xy;
        }
        
        internal static Fix64 Dot(AetherVector2 left, AetherVector2 right)
        {
            return left.X * right.X + left.Y * right.Y;
        }

        internal static Fix64 Distance(AetherVector2 v1, AetherVector2 v2)
        {
            Fix64 dX = v1.X - v2.X;
            Fix64 dY = v1.Y - v2.Y;
            return Fix64.Sqrt(dX * dX + dY * dY);
        }

        internal static Fix64 DistanceSquared(AetherVector2 v1, AetherVector2 v2)
        {
            Fix64 dX = v1.X - v2.X;
            Fix64 dY = v1.Y - v2.Y;
            return (dX * dX + dY * dY);
        }

        public Fix64 Length()
        {
            return Fix64.Sqrt(X * X + Y * Y);
        }

        public Fix64 LengthSquared()
        {
            return (X * X + Y * Y);
        }

        public void Normalize()
        {
            var length = Fix64.Sqrt((X * X) + (Y * Y));
            var invLength = Fix64.One / length;
            X *= invLength;
            Y *= invLength;
        }

        public override int GetHashCode()
        {
            return (X.GetHashCode() ^ Y.GetHashCode());
        }

        public override bool Equals(object obj)
        {
            return (obj is AetherVector2) ? Equals((AetherVector2)obj) : false;
        }

        #region Implement IEquatable<Vector2>
        public bool Equals(AetherVector2 other)
        {
            return (X == other.X && Y == other.Y);
        }
        #endregion Implement IEquatable<Vector2>

        public static AetherVector2 operator +(AetherVector2 left, AetherVector2 right)
        {
            left.X += right.X;
            left.Y += right.Y;
            return left;
        }
        
        public static AetherVector2 operator -(AetherVector2 left, AetherVector2 right)
        {
            left.X -= right.X;
            left.Y -= right.Y;
            return left;
        }

        public static AetherVector2 operator -(AetherVector2 right)
        {
            right.X = -right.X;
            right.Y = -right.Y;
            return right;
        }

        public static AetherVector2 operator *(AetherVector2 left, AetherVector2 right)
        {
            left.X *= right.X;
            left.Y *= right.Y;
            return left;
        }

        public static AetherVector2 operator *(AetherVector2 left, Fix64 right)
        {
            left.X *= right;
            left.Y *= right;
            return left;
        }

        public static AetherVector2 operator *(Fix64 left, AetherVector2 right)
        {
            right.X *= left;
            right.Y *= left;
            return right;
        }
        
        public static AetherVector2 operator /(AetherVector2 left, Fix64 right)
        {
            Fix64 invRight = Fix64.One / right;
            left.X *= invRight;
            left.Y *= invRight;
            return left;
        }

        public static bool operator ==(AetherVector2 left, AetherVector2 right)
        {
            return left.X == right.X && left.Y == right.Y;
        }

        public static bool operator !=(AetherVector2 left, AetherVector2 right)
        {
            return left.X != right.X || left.Y != right.Y;
        }

        public override string ToString()
        {
            return String.Format("{{X: {0} Y: {1}}}", X, Y);
        }

        #region Fast ref methods
        public static void Dot(ref AetherVector2 left, ref AetherVector2 right, out Fix64 result)
        {
            result = left.X * right.X + left.Y * right.Y;
        }

        public static void Min(ref AetherVector2 v1, ref AetherVector2 v2, out AetherVector2 result)
        {
            result.X = (v1.X < v2.X) ? v1.X : v2.X;
            result.Y = (v1.Y < v2.Y) ? v1.Y : v2.Y;
        }

        public static void Max(ref AetherVector2 v1, ref AetherVector2 v2, out AetherVector2 result)
        {
            result.X = (v1.X > v2.X) ? v1.X : v2.X;
            result.Y = (v1.Y > v2.Y) ? v1.Y : v2.Y;
        }

        public static void Distance(ref AetherVector2 v1, ref AetherVector2 v2, out Fix64 result)
        {
            Fix64 dx = v1.X - v2.X;
            Fix64 dy = v1.Y - v2.Y;
            result = Fix64.Sqrt(dx * dx + dy * dy);
        }

        public static void DistanceSquared(ref AetherVector2 v1, ref AetherVector2 v2, out Fix64 result)
        {
            Fix64 dx = v1.X - v2.X;
            Fix64 dy = v1.Y - v2.Y;
            result = (dx * dx) + (dy * dy);
        }

        public static void Add(ref AetherVector2 left, ref AetherVector2 right, out AetherVector2 result)
        {
            result.X = left.X + right.X;
            result.Y = left.Y + right.Y;
        }
        public static void Subtract(ref AetherVector2 left, ref AetherVector2 right, out AetherVector2 result)
        {
            result.X = left.X - right.X;
            result.Y = left.Y - right.Y;
        }

        public static void Multiply(ref AetherVector2 left, ref AetherVector2 right, out AetherVector2 result)
        {
            result.X = left.X * right.X;
            result.Y = left.Y * right.Y;
        }

        public static void Multiply(ref AetherVector2 left, Fix64 right, out AetherVector2 result)
        {
            result.X = left.X * right;
            result.Y = left.Y * right;
        }

        public static void Divide(ref AetherVector2 left, Fix64 right, out AetherVector2 result)
        {
            Fix64 invRight = Fix64.One / right;
            result.X = left.X * invRight;
            result.Y = left.Y * invRight;
        }

        #endregion Fast ref methods

    }
}
