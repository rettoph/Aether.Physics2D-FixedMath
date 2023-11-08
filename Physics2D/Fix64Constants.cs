using FixedMath.NET;
using System;
using System.Collections.Generic;
using System.Text;

namespace tainicom.Aether.Physics2D
{
    internal static class Fix64Constants
    {
        public static readonly Fix64 PointZeroZeroZeroZeroOne = Fix64.FromRaw(42950);
        public static readonly Fix64 PointZeroZeroZeroOne = Fix64.FromRaw(429497);
        public static readonly Fix64 PointZeroZeroOne = Fix64.FromRaw(4294968);
        public static readonly Fix64 PointZeroOne = Fix64.FromRaw(42949672);

        public static readonly Fix64 PointZeroZeroFive = Fix64.FromRaw(21474836);

        public static readonly Fix64 PointOne = Fix64.FromRaw(429496736);
        public static readonly Fix64 PointTwo = Fix64.FromRaw(858993472);
        public static readonly Fix64 PointTwoFive = Fix64.FromRaw(1073741824);
        public static readonly Fix64 PointThree = Fix64.FromRaw(1288490240);
        public static readonly Fix64 PointFive = Fix64.FromRaw(2147483648);
        public static readonly Fix64 PointNineEight = Fix64.FromRaw(4209068032);

        public static readonly Fix64 OnePointFive = Fix64.FromRaw(6442450944);
        public static readonly Fix64 Two = Fix64.FromRaw(8589934592);
        public static readonly Fix64 Three = Fix64.FromRaw(12884901888);
        public static readonly Fix64 Four = Fix64.FromRaw(17179869184);
        public static readonly Fix64 Five = Fix64.FromRaw(21474836480);
        public static readonly Fix64 Seven = Fix64.FromRaw(30064771072);
        public static readonly Fix64 Eight = Fix64.FromRaw(34359738368);
        public static readonly Fix64 Ten = Fix64.FromRaw(42949672960);
        public static readonly Fix64 Fifteen = Fix64.FromRaw(64424509440);
        public static readonly Fix64 TwentyFive = Fix64.FromRaw(107374182400);
        public static readonly Fix64 Fourty = Fix64.FromRaw(171798691840);
        public static readonly Fix64 Fifty = Fix64.FromRaw(214748364800);
        public static readonly Fix64 Ninety = Fix64.FromRaw(386547056640);
        public static readonly Fix64 OneHundred = Fix64.FromRaw(429496729600);
        public static readonly Fix64 FiveHundred = Fix64.FromRaw(2147483648000);
        public static readonly Fix64 OneEighty = Fix64.FromRaw(773094113280);
        public static readonly Fix64 OneThousand = Fix64.FromRaw(4294967296000);

        public static readonly Fix64 k_inv3 = Fix64.One / Fix64.FromRaw(12884901888); // 1.0 / 3.0
        public static readonly Fix64 k_maxConditionNumber = Fix64.FromRaw(4294967296000); // 1000.0
        public static readonly Fix64 k_errorTol = Fix64.FromRaw(4294968); // 1e-3
        public static readonly Fix64 k_relativeTol = Fix64.FromRaw(4209068032); // 0.98
        public static readonly Fix64 k_absoluteTol = Fix64.FromRaw(4294968); // 0.001
        public static readonly Fix64 linearError = Fix64.Zero;
        public static readonly Fix64 gravity = Fix64.FromRaw(-42119241728);
        public static readonly Fix64 tolerance = Fix64Constants.PointTwoFive * Settings.LinearSlop;
        public static readonly Fix64 ALPHA = Fix64.FromRaw(1288490240); // 0.3
        public static readonly Fix64 PiSlop = Fix64.FromRaw(13314398208); // 0.31
        public static readonly Fix64 ClipperEpsilonSquared = Fix64.FromRaw(512); //1.192092896e-07
        public static readonly Fix64 Epsilon = Fix64.FromRaw(1); // very smol
    }
}
