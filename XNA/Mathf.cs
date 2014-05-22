using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;

namespace XNA {
    /// <summary>
    /// Provides constants and static methods for trigonometric, logarithmic and other common mathematical functions.
    /// </summary>
    public class Mathf {
        /// <summary>
        /// Degrees-to-radians conversion constant.
        /// </summary>
        public const float Deg2Rad = 0.0174533f;
        /// <summary>
        /// Degrees-to-grad conversion constant.
        /// </summary>
        public const float Deg2Grad = 1.1111111f;
        /// <summary>
        /// A tiny floating point value.
        /// </summary>
        public const float Epsilon = 1.4013e-045f;
        /// <summary>
        /// Exponential e.
        /// </summary>
        public const float ExponentialE = 2.71828f;
        /// <summary>
        /// The golden ratio. Oooooh!
        /// </summary>
        public const float GoldenRatio = 1.61803f;
        /// <summary>
        /// Grad-to-degrees conversion constant.
        /// </summary>
        public const float Grad2Deg = 0.9f;
        /// <summary>
        /// Grad-to-radians conversion constant.
        /// </summary>
        public const float Grad2Rad = 0.015708f;
        /// <summary>
        /// A representation of positive infinity.
        /// </summary>
        public const float Infinity = 1.0f / 0.0f;
        /// <summary>
        /// A representation of negative infinity.
        /// </summary>
        public const float NegativeInfinity = -1.0f / 0.0f;
        /// <summary>
        /// The infamous 3.14159265358979... value.
        /// </summary>
        public const float PI = 3.14159f;
        /// <summary>
        /// Radians-to-degrees conversion constant.
        /// </summary>
        public const float Rad2Deg = 57.2958f;
        /// <summary>
        /// Radians-to-grad conversion constant.
        /// </summary>
        public const float Rad2Grad = 63.6619772f;
        /// <summary>
        /// The not-so-infamous TAU value.
        /// </summary>
        public const float TAU = PI * 2;

        /// <summary>
        /// Returns the absolute value of a.
        /// </summary>
        /// <param name="a">The value</param>
        public static int Abs(int a) { return Math.Abs(a); }
        /// <summary>
        /// Returns the absolute value of a.
        /// </summary>
        /// <param name="a">The value</param>
        public static float Abs(float a) { return (float)Math.Abs(a); }
        /// <summary>
        /// Returns the arc-cosine of a - the angle in radians whose cosine is a.
        /// </summary>
        /// <param name="a">The value</param>
        public static float Acos(float a) { return (float)Math.Acos(a); }
        /// <summary>
        /// Returns if the two values are approximately close to eachother
        /// </summary>
        /// <param name="a">First value</param>
        /// <param name="b">Second value</param>
        public static bool Approximately(float a, float b) { return RoughlyEqual(a, b, 0.1f); }
        /// <summary>
        /// Returns the arc-sine of a - the angle in radians whose sine is a.
        /// </summary>
        /// <param name="a">The value</param>
        public static float Asin(float a) { return (float)Math.Asin(a); }
        /// <summary>
        /// Returns the arc-tangent of a - the angle in radians whose tangent is a.
        /// </summary>
        /// <param name="a">The value</param>
        public static float Atan(float a) { return (float)Math.Atan(a); }
        /// <summary>
        /// Returns the angle in radians whose Tan is y/x.
        /// </summary>
        /// <param name="y">The y value</param>
        /// <param name="x">The x value</param>
        public static float Atan2(float y, float x) { return (float)Math.Atan2(y, x); }
        /// <summary>
        /// Returns the smallest integer greater to or equal to a.
        /// </summary>
        /// <param name="a">The value</param>
        public static float Ceil(float a) { return (float)Math.Ceiling(a); }
        /// <summary>
        /// Returns the smallest integer greater to or equal to a.
        /// </summary>
        /// <param name="a">The value</param>
        public static int CeilToInt(float a) { return (int)Ceil(a); }
        /// <summary>
        /// Clamps a value between a minimum int and maximum int value.
        /// </summary>
        /// <param name="value">The value</param>
        /// <param name="min">The minimum value</param>
        /// <param name="max">The maximum value</param>
        public static int Clamp(int value, int min, int max) {
            return (int)Clamp((float)value, (float)min, (float)max);
        }
        /// <summary>
        /// Clamps a value between a minimum float and maximum float value.
        /// </summary>
        /// <param name="value">The value</param>
        /// <param name="min">The minimum value</param>
        /// <param name="max">The maximum value</param>
        public static float Clamp(float value, float min, float max) {
            if (max <= min)
                return min;
            return value < min ? min : value > max ? max : value;
        }
        /// <summary>
        /// Clamps value between 0 and 1 and returns value.
        /// </summary>
        /// <param name="value">The value</param>
        public static float Clamp01(float value) { return value < 0 ? 0 : value > 1 ? 1 : value; }
        /// <summary>
        /// Returns the closest power of two to a value.
        /// </summary>
        /// <param name="a">The value</param>
        public static int ClosestPowerOfTwo(int a) {
            int b = NextPowerOfTwo(a),
                c = b / 2;
            return a - c < b - a ? c : b;
        }
        /// <summary>
        /// Returns the cosine of angle f in radians.
        /// </summary>
        /// <param name="a">The value</param>
        public static float Cos(float a) { return (float)Math.Cos(a); }
        /// <summary>
        /// Returns e raised to the specified power.
        /// </summary>
        /// <param name="power">The power</param>
        public static float Exp(float power) { return (float)Math.Exp(power); }
        /// <summary>
        /// Returns the largest integer smaller to or equal to a.
        /// </summary>
        /// <param name="a">The value</param>
        public static float Floor(float a) { return (float)Math.Floor(a); }
        /// <summary>
        /// Returns the largest integer smaller to or equal to a.
        /// </summary>
        /// <param name="a">The value</param>
        public static int FloorToInt(float a) { return (int)Floor(a); }
        /// <summary>
        /// Returns if the value is powered by two.
        /// </summary>
        /// <param name="value">A value</param>
        public static bool IsPowerOfTwo(int value) { return (value > 0) && ((value & (value - 1)) == 0); }
        /// <summary>
        /// Interpolates between from and to by t. t is clamped between 0 and 1.
        /// </summary>
        /// <param name="from">The from value</param>
        /// <param name="to">The to value</param>
        /// <param name="t">The t value</param>
        public static float Lerp(float from, float to, float t) { return t >= 1 ? to : t < 0 ? from : from + (to - from) * t; }
        /// <summary>
        /// Returns the natural (base e) logarithm of a specified value.
        /// </summary>
        /// <param name="value">The value</param>
        public static float Log(float value) { return (float)Math.Log(value); }
        /// <summary>
        /// Returns the base 10 logarithm of a specified value.
        /// </summary>
        /// <param name="value">The value</param>
        public static float Log10(float value) { return (float)Math.Log10(value); }
        /// <summary>
        /// Returns the largest of two integer values.
        /// </summary>
        /// <param name="a">First value</param>
        /// <param name="b">Second value</param>
        public static int Max(int a, int b) { return Math.Max(a, b); }
        /// <summary>
        /// Returns the largest of two float values.
        /// </summary>
        /// <param name="a">First value</param>
        /// <param name="b">Second value</param>
        public static float Max(float a, float b) { return Math.Max(a, b); }
        /// <summary>
        /// Returns the largest of a set of integer values.
        /// </summary>
        /// <param name="values">The set of values</param>
        public static int Max(params int[] values) { return values.Max(); }
        /// <summary>
        /// Returns the largest of a set of float values.
        /// </summary>
        /// <param name="values">The set of values</param>
        public static float Max(params float[] values) { return values.Max(); }
        /// <summary>
        /// Returns the smaller of two integer values.
        /// </summary>
        /// <param name="a">First value</param>
        /// <param name="b">Second value</param>
        public static int Min(int a, int b) { return Math.Min(a, b); }
        /// <summary>
        /// Returns the smaller of two float values.
        /// </summary>
        /// <param name="a">First value</param>
        /// <param name="b">Second value</param>
        public static float Min(float a, float b) { return Math.Min(a, b); }
        /// <summary>
        /// Get the next power of two after a value.
        /// </summary>
        /// <param name="a">The value</param>
        public static int NextPowerOfTwo(int a) {
            if (a < 0)
                return 0;
            a |= a >> 1;
            a |= a >> 2;
            a |= a >> 4;
            a |= a >> 8;
            a |= a >> 16;
            return a + 1;
        }
        /// <summary>
        /// Returns f raised to power p.
        /// </summary>
        /// <param name="f">The value to raise</param>
        /// <param name="p">The power</param>
        public static float Pow(float f, float p) { return (float)Math.Pow(f, p); }
        /// <summary>
        /// Compares two floating point values if they are similar.
        /// </summary>
        /// <param name="a">First value</param>
        /// <param name="b">Second value</param>
        /// <param name="threshold">The threshold of similarity</param>
        /// <returns>True if the values are similar, otherwise false.</returns>
        public static bool RoughlyEqual(float a, float b, float threshold = 0.01f) { return Mathf.Abs(a - b) <= threshold; }
        /// <summary>
        /// Returns f rounded to the nearest integer.
        /// </summary>
        /// <param name="f">The value</param>
        public static float Round(float f) { return (float)Math.Round(f); }
        /// <summary>
        /// Rounds a floating-point value to a specified number of fractional digits. 
        /// </summary>
        /// <param name="f">The value</param>
        /// <param name="decimals">The number of fractional digits to round to</param>
        public static float Round(float f, int decimals) { return (float)Math.Round(f, decimals); }
        /// <summary>
        /// Rounds a floating-point value to a specified number of fractional digits. A parameter specifies how to round a value if it is midway between two other numbers.
        /// </summary>
        /// <param name="f">The value</param>
        /// <param name="decimals">The number of fractional digits to round to</param>
        /// <param name="mode">The rounding mode to use</param>
        public static float Round(float f, int decimals, MidpointRounding mode) { return (float)Math.Round(f, decimals, mode); }
        /// <summary>
        /// Returns f rounded to the nearest integer.
        /// </summary>
        /// <param name="f">The value to round</param>
        public static int RoundToInt(float f) { return (int)Round(f); }
        /// <summary>
        ///  Rounds a floating-point value. A parameter specifies how to round a value if it is midway between two other numbers.
        /// </summary>
        /// <param name="f">The value</param>
        /// <param name="mode">The rounding mode to use</param>
        public static int RoundToInt(float f, MidpointRounding mode) { return (int)Round(f, 0, mode); }
        /// <summary>
        /// Returns the sign of f.
        /// </summary>
        /// <param name="f">The value</param>
        public static float Sign(float f) { return (float)Math.Sign(f); }
        /// <summary>
        /// Returns the sine of angle f in radians.
        /// </summary>
        /// <param name="f">The value</param>
        public static float Sin(float f) { return (float)Math.Sin(f); }
        /// <summary>
        /// Returns square root of f.
        /// </summary>
        /// <param name="f">The value</param>
        public static float Sqrt(float f) { return (float)Math.Sqrt(f); }
        /// <summary>
        /// Returns the tangent of angle f in radians.
        /// </summary>
        /// <param name="f">The value</param>
        public static float Tan(float f) { return (float)Math.Tan(f); }
        /// <summary>
        /// Rotates one point around another
        /// </summary>
        /// <param name="pointToRotate">The point to rotate.</param>
        /// <param name="centerPoint">The centre point of rotation.</param>
        /// <param name="angleInDegrees">The rotation angle in degrees.</param>
        /// <returns>Rotated point</returns>
        public static Vector2 RotatePoint(Vector2 pointToRotate, Vector2 centerPoint, float angleInDegrees) {
            float angleInRadians = angleInDegrees * Mathf.Deg2Rad;
            float cosTheta = Mathf.Cos(angleInRadians);
            float sinTheta = Mathf.Sin(angleInRadians);
            return new Vector2 {
                X = (cosTheta * (pointToRotate.X - centerPoint.X) -
                    sinTheta * (pointToRotate.Y - centerPoint.Y) + centerPoint.X),
                Y = (sinTheta * (pointToRotate.X - centerPoint.X) +
                    cosTheta * (pointToRotate.Y - centerPoint.Y) + centerPoint.Y)
            };
        }
        /// <summary>
        /// Calculates the average of a series of float values.
        /// </summary>
        /// <returns>The average value.</returns>
        public static float Average(params float[] values) {
            return values.Average();
        }

        /// <summary>
        /// Calculates the average of a series of Vector2 values.
        /// </summary>
        /// <returns>The average value.</returns>
        public static Vector2 Average(params Vector2[] values) {
            Vector2 result = Vector2.Zero;

            foreach (var value in values)
                result += value;

            return result / values.Length;
        }

        #region Polygon Math
        /// <summary>
        /// Checks whether a given line intersects with a given polygon and returns the intersections points in an out parameter.
        /// </summary>
        /// <param name="points">The points defining the polygon.</param>
        /// <param name="a">Starting point of the line.</param>
        /// <param name="b">Ending point of the line.</param>
        /// <param name="intersections">The out parameter which contains the intersections points with the PolygonCollider if there were any.</param>
        /// <returns>True if the line intersects the PolygonCollider, otherwise false.</returns>
        public static bool LineIntersectsPolygon(Vector2[] points, Vector2 a, Vector2 b, out List<Vector2> intersections) {
            intersections = new List<Vector2>();

            Vector2 result;

            for (int i = 0;i < points.Length;i++)
                if (LineIntersectsLine(a, b, points[i], points[(i + 1) % points.Length], out result))
                    intersections.Add(result);

            return intersections.Count > 0;
        }

        /// <summary>
        /// Checks whether a given point is within an array of given points.
        /// </summary>
        /// <param name="point">The point to check.</param>
        /// <param name="points">The points defining the polygon to check within.</param>
        /// <returns>True if the point is within the PolygonCollider, otherwise false.</returns>
        public static bool PointInPolygon(Vector2 point, Vector2[] points) {
            List<Vector2> result;

            if (LineIntersectsPolygon(points, point, point - new Vector2(-1000000, 0), out result)) {
                if (result.Count % 2 == 0)
                    return false;
                else
                    return true;
            }

            return false;
        }

        /// <summary>
        /// Checks whether two given polygons intersect one another.
        /// </summary>
        /// <param name="aPoints">One polygon.</param>
        /// <param name="bPoints">The other polygon.</param>
        /// <returns>True if the two polygons intersect, otherwise false.</returns>
        public static bool PolygonIntersectsPolygon(Vector2[] aPoints, Vector2[] bPoints) {
            Vector2 result;

            // This is probably pretty inefficient
            if (PointInPolygon(Average(aPoints), bPoints) ||
                PointInPolygon(Average(bPoints), aPoints))
                return true;

            for (int i1 = 0;i1 < aPoints.Length;i1++)
                for (int i2 = 0;i2 < bPoints.Length;i2++)
                    if (LineIntersectsLine(aPoints[i1],
                                            aPoints[(i1 + 1) % aPoints.Length],
                                            bPoints[i2],
                                            bPoints[(i2 + 1) % bPoints.Length],
                                            out result))
                        return true;

            return false;
        }

        /// <summary>
        /// Checks whether a given circle intersects with a given polygon.
        /// </summary>
        public static bool CircleIntersectsPolygon(Vector2 center, float radius, Vector2[] polygon) {
            if (PointInPolygon(center, polygon))
                return true;

            for (int i = 0;i < polygon.Length;i++)
                if (LineIntersectsCircle(center, radius, polygon[i], polygon[(i + 1) % polygon.Length]))
                    return true;

            return false;
        }

        private static float DistanceToLine(Vector2 point, Vector2 a, Vector2 b) {
            float l2 = (b - a).LengthSquared();
            if (l2 == 0.0)
                return Vector2.Distance(point, a);
            float t = Vector2.Dot(point - a, b - a) / l2;
            if (t < 0.0)
                return Vector2.Distance(point, a);
            else if (t > 1.0)
                return Vector2.Distance(point, b);
            Vector2 projection = a + t * (b - a);
            return Vector2.Distance(point, projection);
        }

        /// <summary>
        /// Checks whether a given line intersects with a given circle.
        /// </summary>
        public static bool LineIntersectsCircle(Vector2 center, float radius, Vector2 a, Vector2 b) {
            float dist = DistanceToLine(center, a, b);

            return dist <= radius;
        }
        /// <summary>
        /// Checks whether a given line intersects with a given CircleCollider and returns the intersections points in an out parameter.
        /// </summary>
        public static bool LineIntersectsCircle(Vector2 center, float radius, Vector2 a, Vector2 b, out List<Vector2> intersects) {
            intersects = new List<Vector2>();

            if (!LineIntersectsCircle(center, radius, a, b))
                return false;

            float dx, dy, A, B, C, det, t;

            dx = b.X - a.X;
            dy = b.Y - a.Y;

            A = dx * dx + dy * dy;
            B = 2 * (dx * (a.X - center.X) + dy * (a.Y - center.Y));
            C = (a.X - center.X) * (a.X - center.X) + (a.Y - center.Y) * (a.Y - center.Y) - radius * radius;

            det = B * B - 4 * A * C;

            if (A <= 0.0000001 || det < 0)
                // No solutions
                return false;
            else if (det == 0) {
                // One solution
                t = -B / (2 * A);

                if (0 <= t && t <= 1)
                    intersects.Add(new Vector2(a.X + t * dx, a.Y + t * dy));
            } else {
                // Two solutions
                t = (-B + Mathf.Sqrt(det)) / (2 * A);
                if (0 <= t && t <= 1)
                    intersects.Add(new Vector2(a.X + t * dx, a.Y + t * dy));
                t = (-B - Mathf.Sqrt(det)) / (2 * A);
                if (0 <= t && t <= 1)
                    intersects.Add(new Vector2(a.X + t * dx, a.Y + t * dy));
            }

            return true;
        }

        /// <summary>
        /// Checks if two given lines intersect with one another and returns the intersection point (if any) in an out parameter.
        /// Source: http://stackoverflow.com/questions/3746274/line-intersection-with-aabb-rectangle.
        /// Edited to implement Cohen-Sutherland type pruning for efficiency.
        /// </summary>
        /// <param name="a1">Starting point of line a.</param>
        /// <param name="a2">Ending point of line a.</param>
        /// <param name="b1">Starting point of line b.</param>
        /// <param name="b2">Ending point of line b.</param>
        /// <param name="intersection">The out parameter which contains the intersection point if there was any.</param>
        /// <returns>True if the two lines intersect, otherwise false.</returns>
        public static bool LineIntersectsLine(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2, out Vector2 intersection) {
            intersection = Vector2.Zero;

            Vector2 bLowerLeft = new Vector2(b1.X < b2.X ? b1.X : b2.X, b1.Y > b2.Y ? b1.Y : b2.Y);
            Vector2 bUpperRight = new Vector2(b1.X > b2.X ? b1.X : b2.X, b1.Y < b2.Y ? b1.Y : b2.Y);

            if ((a1.X < bLowerLeft.X && a2.X < bLowerLeft.X)
                || (a1.Y > bLowerLeft.Y && a2.Y > bLowerLeft.Y)
                || (a1.X > bUpperRight.X && a2.X > bUpperRight.X)
                || (a1.Y < bUpperRight.Y && a2.Y < bUpperRight.Y))
                return false;

            Vector2 b = a2 - a1;
            Vector2 d = b2 - b1;

            float bDotDPerp = b.X * d.Y - b.Y * d.X;

            // If b dot d == 0, it means the lines are parallel and have infinite intersection points
            if (bDotDPerp == 0)
                return false;

            Vector2 c = b1 - a1;
            float t = (c.X * d.Y - c.Y * d.X) / bDotDPerp;
            if (t < 0 || t > 1)
                return false;

            float u = (c.X * b.Y - c.Y * b.X) / bDotDPerp;
            if (u < 0 || u > 1)
                return false;

            intersection = a1 + t * b;

            return true;
        }
        #endregion
    }
}
