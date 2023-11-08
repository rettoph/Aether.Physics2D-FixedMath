// Copyright (c) 2017 Kastellanos Nikolaos

using FixedMath.NET;
using System;

namespace tainicom.Aether.Physics2D.Common
{
    public struct AetherVector3 : IEquatable<AetherVector3>
    {
        private static readonly AetherVector3 _zero = new AetherVector3(Fix64.Zero, Fix64.Zero, Fix64.Zero);
        private static readonly AetherVector3 _one  = new AetherVector3(Fix64.One, Fix64.One, Fix64.One);


        public Fix64 X;
        public Fix64 Y;
        public Fix64 Z;

        public static AetherVector3 Zero { get { return _zero; } }
        public static AetherVector3 One { get { return _one; } }


        public AetherVector3(Fix64 x, Fix64 y, Fix64 z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        public AetherVector3(Fix64 xyz)
        {
            this.X = xyz;
            this.Y = xyz;
            this.Z = xyz;
        }

        public Fix64 Length()
        {
            return Fix64.Sqrt(X * X + Y * Y + Z * Z);
        }

        public Fix64 LengthSquared()
        {
            return (X * X + Y * Y + Z * Z);
        }


        internal static AetherVector3 Cross(AetherVector3 left, AetherVector3 right)
        {
            AetherVector3 result;
            result.X = left.Y * right.Z - left.Z * right.Y;
            result.Y = left.Z * right.X - left.X * right.Z;
            result.Z = left.X * right.Y - left.Y * right.X;
            return result;

        }

        internal static Fix64 Dot(AetherVector3 left, AetherVector3 right)
        {
            return left.X * right.X + left.Y * right.Y + left.Z * right.Z;
        }

        public override int GetHashCode()
        {
            return (X.GetHashCode() ^ Y.GetHashCode()^ Z.GetHashCode());
        }

        public override bool Equals(object obj)
        {
            return (obj is AetherVector3) ? Equals((AetherVector3)obj) : false;
        }

        #region Implement IEquatable<Vector3>
        public bool Equals(AetherVector3 other)
        {
            return (X == other.X && Y == other.Y && Z == other.Z);
        }
        #endregion Implement IEquatable<Vector3>

        public static AetherVector3 operator +(AetherVector3 left, AetherVector3 right)
        {
            left.X += right.X;
            left.Y += right.Y;
            left.Z += right.Z;
            return left;
        }
        
        public static AetherVector3 operator -(AetherVector3 left, AetherVector3 right)
        {
            left.X -= right.X;
            left.Y -= right.Y;
            left.Z -= right.Z;
            return left;
        }

        public static AetherVector3 operator -(AetherVector3 right)
        {
            right.X = -right.X;
            right.Y = -right.Y;
            right.Z = -right.Z;
            return right;
        }

        public static AetherVector3 operator *(AetherVector3 left, AetherVector3 right)
        {
            left.X *= right.X;
            left.Y *= right.Y;
            left.Z *= right.Z;
            return left;
        }

        public static AetherVector3 operator *(AetherVector3 left, Fix64 right)
        {
            left.X *= right;
            left.Y *= right;
            left.Z *= right;
            return left;
        }

        public static AetherVector3 operator *(Fix64 left, AetherVector3 right)
        {
            right.X *= left;
            right.Y *= left;
            right.Z *= left;
            return right;
        }
        
        public static AetherVector3 operator /(AetherVector3 left, Fix64 right)
        {
            Fix64 invRight = Fix64.One / right;
            left.X *= invRight;
            left.Y *= invRight;
            left.Z *= invRight;
            return left;
        }

        public static bool operator ==(AetherVector3 left, AetherVector3 right)
        {
            return left.X == right.X && left.Y == right.Y && left.Z == right.Z;
        }

        public static bool operator !=(AetherVector3 left, AetherVector3 right)
        {
            return left.X != right.X || left.Y != right.Y || left.Z != right.Z;
        }

        public override string ToString()
        {
            return String.Format("{{X: {0} Y: {1} Z: {2}}}", X, Y, Z);
        }
    }
}
