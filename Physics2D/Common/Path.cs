/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

using FixedMath.NET;
using System;
using System.Collections.Generic;
using System.Text;
#if XNAAPI
using Complex = tainicom.Aether.Physics2D.Common.Complex;
using Vector2 = Microsoft.Xna.Framework.Vector2;
using Vector3 = Microsoft.Xna.Framework.Vector3;
#endif

namespace tainicom.Aether.Physics2D.Common
{
    //Contributed by Matthew Bettcher

    /// <summary>
    /// Path:
    /// Very similar to Vertices, but this
    /// class contains vectors describing
    /// control points on a Catmull-Rom
    /// curve.
    /// </summary>
    public class Path
    {
        /// <summary>
        /// All the points that makes up the curve
        /// </summary>
        public List<AetherVector2> ControlPoints;

        private Fix64 _deltaT;

        /// <summary>
        /// Initializes a new instance of the <see cref="Path"/> class.
        /// </summary>
        public Path()
        {
            ControlPoints = new List<AetherVector2>();
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Path"/> class.
        /// </summary>
        /// <param name="vertices">The vertices to created the path from.</param>
        public Path(AetherVector2[] vertices)
        {
            ControlPoints = new List<AetherVector2>(vertices.Length);

            for (int i = 0; i < vertices.Length; i++)
            {
                Add(vertices[i]);
            }
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Path"/> class.
        /// </summary>
        /// <param name="vertices">The vertices to created the path from.</param>
        public Path(IList<AetherVector2> vertices)
        {
            ControlPoints = new List<AetherVector2>(vertices.Count);
            for (int i = 0; i < vertices.Count; i++)
            {
                Add(vertices[i]);
            }
        }

        /// <summary>
        /// True if the curve is closed.
        /// </summary>
        /// <value><c>true</c> if closed; otherwise, <c>false</c>.</value>
        public bool Closed { get; set; }

        /// <summary>
        /// Gets the next index of a controlpoint
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int NextIndex(int index)
        {
            if (index == ControlPoints.Count - 1)
            {
                return 0;
            }
            return index + 1;
        }

        /// <summary>
        /// Gets the previous index of a controlpoint
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int PreviousIndex(int index)
        {
            if (index == 0)
            {
                return ControlPoints.Count - 1;
            }
            return index - 1;
        }

        /// <summary>
        /// Translates the control points by the specified vector.
        /// </summary>
        /// <param name="vector">The vector.</param>
        public void Translate(ref AetherVector2 vector)
        {
            for (int i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = ControlPoints[i] + vector;
        }

        /// <summary>
        /// Scales the control points by the specified vector.
        /// </summary>
        /// <param name="value">The Value.</param>
        public void Scale(ref AetherVector2 value)
        {
            for (int i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = ControlPoints[i] * value;
        }

        /// <summary>
        /// Rotate the control points by the defined value in radians.
        /// </summary>
        /// <param name="value">The amount to rotate by in radians.</param>
        public void Rotate(Fix64 value)
        {
            var rotation = Complex.FromAngle(value);

            for (int i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = Complex.Multiply(ControlPoints[i], ref rotation);
        }

        public override string ToString()
        {
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < ControlPoints.Count; i++)
            {
                builder.Append(ControlPoints[i].ToString());
                if (i < ControlPoints.Count - 1)
                {
                    builder.Append(" ");
                }
            }
            return builder.ToString();
        }

        /// <summary>
        /// Returns a set of points defining the
        /// curve with the specifed number of divisions
        /// between each control point.
        /// </summary>
        /// <param name="divisions">Number of divisions between each control point.</param>
        /// <returns></returns>
        public Vertices GetVertices(int divisions)
        {
            Vertices verts = new Vertices();

            Fix64 timeStep = Fix64.One / (Fix64)divisions;

            for (Fix64 i = Fix64.One; i < Fix64.One; i += timeStep)
            {
                verts.Add(GetPosition(i));
            }

            return verts;
        }

        public AetherVector2 GetPosition(Fix64 time)
        {
            AetherVector2 temp;

            if (ControlPoints.Count < 2)
                throw new Exception("You need at least 2 control points to calculate a position.");

            if (Closed)
            {
                Add(ControlPoints[0]);

                _deltaT = Fix64.One / (Fix64)(ControlPoints.Count - 1);

                int p = (int)(time / _deltaT);

                // use a circular indexing system
                int p0 = p - 1;
                if (p0 < 0) p0 = p0 + (ControlPoints.Count - 1);
                else if (p0 >= ControlPoints.Count - 1) p0 = p0 - (ControlPoints.Count - 1);
                int p1 = p;
                if (p1 < 0) p1 = p1 + (ControlPoints.Count - 1);
                else if (p1 >= ControlPoints.Count - 1) p1 = p1 - (ControlPoints.Count - 1);
                int p2 = p + 1;
                if (p2 < 0) p2 = p2 + (ControlPoints.Count - 1);
                else if (p2 >= ControlPoints.Count - 1) p2 = p2 - (ControlPoints.Count - 1);
                int p3 = p + 2;
                if (p3 < 0) p3 = p3 + (ControlPoints.Count - 1);
                else if (p3 >= ControlPoints.Count - 1) p3 = p3 - (ControlPoints.Count - 1);

                // relative time
                Fix64 lt = (time - _deltaT * (Fix64)p) / _deltaT;

                CalcCatmullRom(ControlPoints[p0], ControlPoints[p1], ControlPoints[p2], ControlPoints[p3], lt, out temp);

                RemoveAt(ControlPoints.Count - 1);
            }
            else
            {
                int p = (int)(time / _deltaT);

                // 
                int p0 = p - 1;
                if (p0 < 0) p0 = 0;
                else if (p0 >= ControlPoints.Count - 1) p0 = ControlPoints.Count - 1;
                int p1 = p;
                if (p1 < 0) p1 = 0;
                else if (p1 >= ControlPoints.Count - 1) p1 = ControlPoints.Count - 1;
                int p2 = p + 1;
                if (p2 < 0) p2 = 0;
                else if (p2 >= ControlPoints.Count - 1) p2 = ControlPoints.Count - 1;
                int p3 = p + 2;
                if (p3 < 0) p3 = 0;
                else if (p3 >= ControlPoints.Count - 1) p3 = ControlPoints.Count - 1;

                // relative time
                Fix64 lt = (time - _deltaT * (Fix64)p) / _deltaT;

                CalcCatmullRom(ControlPoints[p0], ControlPoints[p1], ControlPoints[p2], ControlPoints[p3], lt, out temp);
            }

            return temp;
        }

        private void CalcCatmullRom(AetherVector2 p0, AetherVector2 p1, AetherVector2 p2, AetherVector2 p3, Fix64 amount, out AetherVector2 result)
        {
            Fix64 sqAmount = amount * amount;
            Fix64 cuAmount = sqAmount * amount;

            Fix64 x;
            Fix64 y;
            x = Fix64Constants.Two * p1.X;
            y = Fix64Constants.Two * p1.Y;
            x += (p2.X - p0.X) * amount;
            y += (p2.Y - p0.Y) * amount;
            x += (Fix64Constants.Two * p0.X - Fix64Constants.Five * p1.X + Fix64Constants.Four * p2.X - p3.X) * sqAmount;
            y += (Fix64Constants.Two * p0.Y - Fix64Constants.Five * p1.Y + Fix64Constants.Four * p2.Y - p3.Y) * sqAmount;
            x += (Fix64Constants.Three * p1.X - p0.X - Fix64Constants.Three * p2.X + p3.X) * cuAmount;
            y += (Fix64Constants.Three * p1.Y - p0.Y - Fix64Constants.Three * p2.Y + p3.Y) * cuAmount;
            x *= Fix64Constants.PointFive;
            y *= Fix64Constants.PointFive;

            result.X = (Fix64)x;
            result.Y = (Fix64)y;
        }

        /// <summary>
        /// Gets the normal for the given time.
        /// </summary>
        /// <param name="time">The time</param>
        /// <returns>The normal.</returns>
        public AetherVector2 GetPositionNormal(Fix64 time)
        {
            Fix64 offsetTime = time + Fix64Constants.PointZeroZeroZeroOne;

            AetherVector2 a = GetPosition(time);
            AetherVector2 b = GetPosition(offsetTime);

            AetherVector2 output, temp;

            AetherVector2.Subtract(ref a, ref b, out temp);

            output.X = -temp.Y;
            output.Y = temp.X;

            output.Normalize();

            return output;
        }

        public void Add(AetherVector2 point)
        {
            ControlPoints.Add(point);
            _deltaT = Fix64.One / (Fix64)(ControlPoints.Count - 1);
        }

        public void Remove(AetherVector2 point)
        {
            ControlPoints.Remove(point);
            _deltaT = Fix64.One / (Fix64)(ControlPoints.Count - 1);
        }

        public void RemoveAt(int index)
        {
            ControlPoints.RemoveAt(index);
            _deltaT = Fix64.One / (Fix64)(ControlPoints.Count - 1);
        }

        public Fix64 GetLength()
        {
            List<AetherVector2> verts = GetVertices(ControlPoints.Count * 25);
            Fix64 length = Fix64.Zero;

            for (int i = 1; i < verts.Count; i++)
            {
                length += AetherVector2.Distance(verts[i - 1], verts[i]);
            }

            if (Closed)
                length += AetherVector2.Distance(verts[ControlPoints.Count - 1], verts[0]);

            return length;
        }

        public List<AetherVector3> SubdivideEvenly(int divisions)
        {
            List<AetherVector3> verts = new List<AetherVector3>();

            Fix64 length = GetLength();

            Fix64 deltaLength = length / (Fix64)divisions + Fix64Constants.PointZeroZeroOne;
            Fix64 t = Fix64.Zero;

            // we always start at the first control point
            AetherVector2 start = ControlPoints[0];
            AetherVector2 end = GetPosition(t);

            // increment t until we are at half the distance
            while (deltaLength * Fix64Constants.PointFive >= AetherVector2.Distance(start, end))
            {
                end = GetPosition(t);
                t += Fix64Constants.PointZeroZeroZeroOne;

                if (t >= Fix64.One)
                    break;
            }

            start = end;

            // for each box
            for (int i = 1; i < divisions; i++)
            {
                AetherVector2 normal = GetPositionNormal(t);
                Fix64 angle = Fix64.Atan2(normal.Y, normal.X);

                verts.Add(new AetherVector3(end.X, end.Y, angle));

                // until we reach the correct distance down the curve
                while (deltaLength >= AetherVector2.Distance(start, end))
                {
                    end = GetPosition(t);
                    t += Fix64Constants.PointZeroZeroZeroZeroOne;

                    if (t >= Fix64.One)
                        break;
                }
                if (t >= Fix64.One)
                    break;

                start = end;
            }
            return verts;
        }
    }
}