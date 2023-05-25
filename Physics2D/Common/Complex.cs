// Copyright (c) 2017 Kastellanos Nikolaos

using FixedMath.NET;
using System;
#if XNAAPI
using Complex = tainicom.Aether.Physics2D.Common.Complex;
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Common
{
    public struct Complex
    {
        private static readonly Complex _one = new Complex(Fix64.One, Fix64.Zero);
        private static readonly Complex _imaginaryOne = new Complex(Fix64.Zero, Fix64.One);

        public Fix64 R;
        public Fix64 i;

        public static Complex One { get { return _one; } }
        public static Complex ImaginaryOne { get { return _imaginaryOne; } }

        public Fix64 Phase
        {
            get { return Fix64.Atan2(i, R); }
            set 
            {
                if (value == Fix64.Zero)
                {
                    this = Complex.One;
                    return;
                }
                this.R = Fix64.Cos(value);
                this.i = Fix64.Sin(value);
            }
        }

        public Fix64 Magnitude
        {
            get { return Fix64.Sqrt(MagnitudeSquared()); }
        }


        public Complex(Fix64 real, Fix64 imaginary)
        {
            R = real;
            i = imaginary;
        }
                
        public static Complex FromAngle(Fix64 angle)
        {
            if (angle == Fix64.Zero)
                return Complex.One;

            return new Complex(
                Fix64.Cos(angle),
                Fix64.Sin(angle));
        }        

        public void Conjugate()
        {
            i = -i;
        }
                
        public void Negate()
        {
            R = -R;
            i = -i;
        }

        public Fix64 MagnitudeSquared()
        {
            return (R * R) + (i * i);
        }

        public void Normalize()
        {
            var mag = Magnitude;
            R = R / mag;
            i = i / mag;            
        }

        public AetherVector2 ToVector2()
        {
            return new AetherVector2(R, i);
        }
        
        public static Complex Multiply(ref Complex left, ref Complex right)
        {
            return new Complex( left.R * right.R  - left.i * right.i,
                                left.i * right.R  + left.R * right.i);
        }

        public static Complex Divide(ref Complex left, ref Complex right)
        {
            return new Complex( right.R * left.R + right.i * left.i,
                                right.R * left.i - right.i * left.R);
        }
        public static void Divide(ref Complex left, ref Complex right, out Complex result)
        {
            result = new Complex(right.R * left.R + right.i * left.i,
                                 right.R * left.i - right.i * left.R);
        }

        public static AetherVector2 Multiply(ref AetherVector2 left, ref Complex right)
        {
            return new AetherVector2(left.X * right.R - left.Y * right.i,
                               left.Y * right.R + left.X * right.i);
        }
        public static void Multiply(ref AetherVector2 left, ref Complex right, out AetherVector2 result)
        {
            result = new AetherVector2(left.X * right.R - left.Y * right.i,
                                 left.Y * right.R + left.X * right.i);
        }
        public static AetherVector2 Multiply(AetherVector2 left, ref Complex right)
        {
            return new AetherVector2(left.X * right.R - left.Y * right.i,
                               left.Y * right.R + left.X * right.i);
        }

        public static AetherVector2 Divide(ref AetherVector2 left, ref Complex right)
        {
            return new AetherVector2(left.X * right.R + left.Y * right.i,
                               left.Y * right.R - left.X * right.i);
        }

        public static AetherVector2 Divide(AetherVector2 left, ref Complex right)
        {
            return new AetherVector2(left.X * right.R + left.Y * right.i,
                               left.Y * right.R - left.X * right.i);
        }
        public static void Divide(AetherVector2 left, ref Complex right, out AetherVector2 result)
        {
            result = new AetherVector2(left.X * right.R + left.Y * right.i,
                                 left.Y * right.R - left.X * right.i);
        }
        
        public static Complex Conjugate(ref Complex value)
        {
            return new Complex(value.R, -value.i);
        }

        public static Complex Negate(ref Complex value)
        {
            return new Complex(-value.R, -value.i);
        }

        public static Complex Normalize(ref Complex value)
        {
            var mag = value.Magnitude;
            return new Complex(value.R / mag, -value.i / mag);
        }
        
        public override string ToString()
        {
            return String.Format("{{R: {0} i: {1} Phase: {2} Magnitude: {3}}}", R, i, Phase, Magnitude);
        }
    }
}
