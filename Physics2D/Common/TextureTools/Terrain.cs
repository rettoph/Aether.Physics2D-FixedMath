/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

using FixedMath.NET;
using System.Collections.Generic;
using tainicom.Aether.Physics2D.Collision;
using tainicom.Aether.Physics2D.Common;
using tainicom.Aether.Physics2D.Common.Decomposition;
using tainicom.Aether.Physics2D.Common.PolygonManipulation;
using tainicom.Aether.Physics2D.Dynamics;
#if XNAAPI
using Vector2 = Microsoft.Xna.Framework.Vector2;
#endif

namespace tainicom.Aether.Physics2D.Common.TextureTools
{
    /// <summary>
    /// Simple class to maintain a terrain. It can keep track
    /// </summary>
    public class Terrain
    {
        /// <summary>
        /// World to manage terrain in.
        /// </summary>
        public World World;

        /// <summary>
        /// Center of terrain in world units.
        /// </summary>
        public AetherVector2 Center;

        /// <summary>
        /// Width of terrain in world units.
        /// </summary>
        public Fix64 Width;

        /// <summary>
        /// Height of terrain in world units.
        /// </summary>
        public Fix64 Height;

        /// <summary>
        /// Points per each world unit used to define the terrain in the point cloud.
        /// </summary>
        public int PointsPerUnit;

        /// <summary>
        /// Points per cell.
        /// </summary>
        public int CellSize;

        /// <summary>
        /// Points per sub cell.
        /// </summary>
        public int SubCellSize;

        /// <summary>
        /// Number of iterations to perform in the Marching Squares algorithm.
        /// Note: More then 3 has almost no effect on quality.
        /// </summary>
        public int Iterations = 2;

        /// <summary>
        /// Decomposer to use when regenerating terrain. Can be changed on the fly without consequence.
        /// Note: Some decomposerers are unstable.
        /// </summary>
        public TriangulationAlgorithm Decomposer;

        /// <summary>
        /// Point cloud defining the terrain.
        /// </summary>
        private sbyte[,] _terrainMap;

        /// <summary>
        /// Generated bodies.
        /// </summary>
        private List<Body>[,] _bodyMap;

        private Fix64 _localWidth;
        private Fix64 _localHeight;
        private int _xnum;
        private int _ynum;
        private AABB _dirtyArea;
        private AetherVector2 _topLeft;

        /// <summary>
        /// Creates a new terrain.
        /// </summary>
        /// <param name="world">The World</param>
        /// <param name="area">The area of the terrain.</param>
        public Terrain(World world, AABB area)
        {
            World = world;
            Width = area.Width;
            Height = area.Height;
            Center = area.Center;
        }

        /// <summary>
        /// Creates a new terrain
        /// </summary>
        /// <param name="world">The World</param>
        /// <param name="position">The position (center) of the terrain.</param>
        /// <param name="width">The width of the terrain.</param>
        /// <param name="height">The height of the terrain.</param>
        public Terrain(World world, AetherVector2 position, Fix64 width, Fix64 height)
        {
            World = world;
            Width = width;
            Height = height;
            Center = position;
        }

        /// <summary>
        /// Initialize the terrain for use.
        /// </summary>
        public void Initialize()
        {
            // find top left of terrain in world space
            _topLeft = new AetherVector2(Center.X - (Width * Fix64Constants.PointFive), Center.Y - (-Height * Fix64Constants.PointFive));

            // convert the terrains size to a point cloud size
            _localWidth = Width * (Fix64)PointsPerUnit;
            _localHeight = Height * (Fix64)PointsPerUnit;

            _terrainMap = new sbyte[(int)_localWidth + 1, (int)_localHeight + 1];

            for (int x = 0; (Fix64)x < _localWidth; x++)
            {
                for (int y = 0; (Fix64)y < _localHeight; y++)
                {
                    _terrainMap[x, y] = 1;
                }
            }

            _xnum = (int)(_localWidth / (Fix64)CellSize);
            _ynum = (int)(_localHeight / (Fix64)CellSize);
            _bodyMap = new List<Body>[_xnum, _ynum];

            // make sure to mark the dirty area to an infinitely small box
            _dirtyArea = new AABB(new AetherVector2(Fix64.MaxValue, Fix64.MaxValue), new AetherVector2(Fix64.MinValue, Fix64.MinValue));
        }

        /// <summary>
        /// Apply the specified texture data to the terrain.
        /// </summary>
        /// <param name="data"></param>
        /// <param name="offset"></param>
        public void ApplyData(sbyte[,] data, AetherVector2 offset = default(AetherVector2))
        {
            for (int x = 0; x < data.GetUpperBound(0); x++)
            {
                for (int y = 0; y < data.GetUpperBound(1); y++)
                {
                    if ((Fix64)x + offset.X >= Fix64.Zero && (Fix64)x + offset.X < _localWidth && (Fix64)y + offset.Y >= Fix64.Zero && (Fix64)y + offset.Y < _localHeight)
                    {
                        _terrainMap[(int)((Fix64)x + offset.X), (int)((Fix64)y + offset.Y)] = data[x, y];
                    }
                }
            }

            RemoveOldData(0, _xnum, 0, _ynum);
        }

        /// <summary>
        /// Modify a single point in the terrain.
        /// </summary>
        /// <param name="location">World location to modify. Automatically clipped.</param>
        /// <param name="value">-1 = inside terrain, 1 = outside terrain</param>
        public void ModifyTerrain(AetherVector2 location, sbyte value)
        {
            // find local position
            // make position local to map space
            AetherVector2 p = location - _topLeft;

            // find map position for each axis
            p.X = p.X * _localWidth / Width;
            p.Y = p.Y * -_localHeight / Height;

            if (p.X >= Fix64.Zero && p.X < _localWidth && p.Y >= Fix64.Zero && p.Y < _localHeight)
            {
                _terrainMap[(int)p.X, (int)p.Y] = value;

                // expand dirty area
                if (p.X < _dirtyArea.LowerBound.X) _dirtyArea.LowerBound.X = p.X;
                if (p.X > _dirtyArea.UpperBound.X) _dirtyArea.UpperBound.X = p.X;

                if (p.Y < _dirtyArea.LowerBound.Y) _dirtyArea.LowerBound.Y = p.Y;
                if (p.Y > _dirtyArea.UpperBound.Y) _dirtyArea.UpperBound.Y = p.Y;
            }
        }

        /// <summary>
        /// Regenerate the terrain.
        /// </summary>
        public void RegenerateTerrain()
        {
            //iterate effected cells
            int xStart = (int)(_dirtyArea.LowerBound.X / (Fix64)CellSize);
            if (xStart < 0) xStart = 0;

            int xEnd = (int)(_dirtyArea.UpperBound.X / (Fix64)CellSize) + 1;
            if (xEnd > _xnum) xEnd = _xnum;

            int yStart = (int)(_dirtyArea.LowerBound.Y / (Fix64)CellSize);
            if (yStart < 0) yStart = 0;

            int yEnd = (int)(_dirtyArea.UpperBound.Y / (Fix64)CellSize) + 1;
            if (yEnd > _ynum) yEnd = _ynum;

            RemoveOldData(xStart, xEnd, yStart, yEnd);

            _dirtyArea = new AABB(new AetherVector2(Fix64.MaxValue, Fix64.MaxValue), new AetherVector2(Fix64.MinValue, Fix64.MinValue));
        }

        private void RemoveOldData(int xStart, int xEnd, int yStart, int yEnd)
        {
            for (int x = xStart; x < xEnd; x++)
            {
                for (int y = yStart; y < yEnd; y++)
                {
                    //remove old terrain object at grid cell
                    if (_bodyMap[x, y] != null)
                    {
                        for (int i = 0; i < _bodyMap[x, y].Count; i++)
                        {
                            World.Remove(_bodyMap[x, y][i]);
                        }
                    }

                    _bodyMap[x, y] = null;

                    //generate new one
                    GenerateTerrain(x, y);
                }
            }
        }

        private void GenerateTerrain(int gx, int gy)
        {
            Fix64 ax = (Fix64)(gx * CellSize);
            Fix64 ay = (Fix64)(gy * CellSize);

            List<Vertices> polys = MarchingSquares.DetectSquares(new AABB(new AetherVector2(ax, ay), new AetherVector2(ax + (Fix64)CellSize, ay + (Fix64)CellSize)), (Fix64)SubCellSize, (Fix64)SubCellSize, _terrainMap, Iterations, true);
            if (polys.Count == 0) return;

            _bodyMap[gx, gy] = new List<Body>();

            // create the scale vector
            AetherVector2 scale = new AetherVector2(Fix64.One / (Fix64)PointsPerUnit, Fix64.One / -(Fix64)PointsPerUnit);

            // create physics object for this grid cell
            foreach (Vertices item in polys)
            {
                // does this need to be negative?
                item.Scale(ref scale);
                item.Translate(ref _topLeft);
                Vertices simplified = SimplifyTools.CollinearSimplify(item);

                List<Vertices> decompPolys = Triangulate.ConvexPartition(simplified, Decomposer);

                foreach (Vertices poly in decompPolys)
                {
                    if (poly.Count > 2)
                        _bodyMap[gx, gy].Add(World.CreatePolygon(poly, Fix64.One, AetherVector2.Zero, Fix64.Zero));
                }
            }
        }
    }
}