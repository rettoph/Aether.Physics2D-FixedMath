﻿/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

/* Poly2Tri
 * Copyright (c) 2009-2010, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

using FixedMath.NET;
using System.Collections.Generic;
using tainicom.Aether.Physics2D.Common.Decomposition.CDT.Delaunay.Sweep;

namespace tainicom.Aether.Physics2D.Common.Decomposition.CDT
{
    internal class TriangulationPoint
    {
        // List of edges this point constitutes an upper ending point (CDT)

        public Fix64 X, Y;

        public TriangulationPoint(Fix64 x, Fix64 y)
        {
            X = x;
            Y = y;
        }

        public List<DTSweepConstraint> Edges { get; private set; }

        public Fix64 Xf
        {
            get { return (Fix64) X; }
            set { X = value; }
        }

        public Fix64 Yf
        {
            get { return (Fix64) Y; }
            set { Y = value; }
        }

        public bool HasEdges
        {
            get { return Edges != null; }
        }

        public override string ToString()
        {
            return "[" + X + "," + Y + "]";
        }

        public void AddEdge(DTSweepConstraint e)
        {
            if (Edges == null)
            {
                Edges = new List<DTSweepConstraint>();
            }
            Edges.Add(e);
        }
    }
}