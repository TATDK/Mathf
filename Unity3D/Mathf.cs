using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace Unity3D {
    public class Mathf {
        public const float Deg2Rad = UnityEngine.Mathf.Deg2Rad;
        public const float Deg2Grad = 1.1111111f;
        public const float Epsilon = UnityEngine.Mathf.Epsilon;
        public const float ExponentialE = 2.71828f;
        public const float GoldenRatio = 1.61803f;
        public const float Grad2Deg = 0.9f;
        public const float Grad2Rad = 0.015708f;
        public const float Infinity = UnityEngine.Mathf.Infinity;
        public const float NegativeInfinity = UnityEngine.Mathf.NegativeInfinity;
        public const float PI = UnityEngine.Mathf.PI;
        public const float Rad2Deg = UnityEngine.Mathf.Rad2Deg;
        public const float Rad2Grad = 63.6619772f;
        public const float TAU = UnityEngine.Mathf.PI * 2;

        public static int Abs(int a) { return UnityEngine.Mathf.Abs(a); }
        public static float Abs(float a) { return UnityEngine.Mathf.Abs(a); }
        public static float Acos(float a) { return UnityEngine.Mathf.Acos(a); }
        public static bool Approximately(float a, float b) { return UnityEngine.Mathf.Approximately(a, b); }
        public static float Asin(float a) { return UnityEngine.Mathf.Asin(a); }
        public static float Atan(float a) { return UnityEngine.Mathf.Atan(a); }
        public static float Atan2(float y, float x) { return UnityEngine.Mathf.Atan2(y, x); }
        public static float Ceil(float a) { return UnityEngine.Mathf.Ceil(a); }
        public static int CeilToInt(float a) { return UnityEngine.Mathf.CeilToInt(a); }
        public static int Clamp(int value, int min, int max) { return UnityEngine.Mathf.Clamp(value, min, max); }
        public static float Clamp(float value, float min, float max) { return UnityEngine.Mathf.Clamp(value, min, max); }
        public static float Clamp01(float value) { return UnityEngine.Mathf.Clamp01(value); }
        public static int ClosestPowerOfTwo(int a) { return UnityEngine.Mathf.ClosestPowerOfTwo(a); }
        public static float Cos(float a) { return UnityEngine.Mathf.Cos(a); }
        public static float DeltaAngle(float current, float target) { return UnityEngine.Mathf.DeltaAngle(current, target); }
        public static float Exp(float power) { return UnityEngine.Mathf.Exp(power); }
        public static float Floor(float a) { return UnityEngine.Mathf.Floor(a); }
        public static int FloorToInt(float a) { return UnityEngine.Mathf.FloorToInt(a); }
        public static float Gamma(float value, float absmax, float gamma) { return UnityEngine.Mathf.Gamma(value, absmax, gamma); }
        public static float GammaToLinearSpace(float value) { return UnityEngine.Mathf.GammaToLinearSpace(value); }
        public static float InverseLerp(float from, float to, float value) { return UnityEngine.Mathf.InverseLerp(from, to, value); }
        public static bool IsPowerOfTwo(int a) { return UnityEngine.Mathf.IsPowerOfTwo(a); }
        public static float Lerp(float from, float to, float t) { return UnityEngine.Mathf.Lerp(from, to, t); }
        public static float LerpAngle(float a, float b, float t) { return UnityEngine.Mathf.LerpAngle(a, b, t); }
        public static float LinearToGammaSpace(float value) { return UnityEngine.Mathf.LinearToGammaSpace(value); }
        public static float Log(float value) { return UnityEngine.Mathf.Log(value); }
        public static float Log10(float value) { return UnityEngine.Mathf.Log10(value); }
        public static int Max(int a, int b) { return UnityEngine.Mathf.Max(a, b); }
        public static float Max(float a, float b) { return UnityEngine.Mathf.Max(a, b); }
        public static int Max(params int[] values) { return UnityEngine.Mathf.Max(values); }
        public static float Max(params float[] values) { return UnityEngine.Mathf.Max(values); }
        public static int Min(int a, int b) { return UnityEngine.Mathf.Min(a, b); }
        public static float Min(float a, float b) { return UnityEngine.Mathf.Min(a, b); }
        public static int Min(params int[] values) { return UnityEngine.Mathf.Min(values); }
        public static float Min(params float[] values) { return UnityEngine.Mathf.Min(values); }
        public static float MoveTowards(float current, float target, float maxDelta) { return UnityEngine.Mathf.MoveTowards(current, target, maxDelta); }
        public static float MoveTowardsAngle(float current, float target, float maxDelta) { return UnityEngine.Mathf.MoveTowardsAngle(current, target, maxDelta); }
        public static int NextPowerOfTwo(int a) { return UnityEngine.Mathf.NextPowerOfTwo(a); }
        public static float PerlinNoise(float x, float y) { return UnityEngine.Mathf.PerlinNoise(x, y); }
        public static float PingPong(float t, float length) { return UnityEngine.Mathf.PingPong(t, length); }
        public static float Pow(float f, float p) { return UnityEngine.Mathf.Pow(f, p); }
        public static float Repeat(float t, float length) { return UnityEngine.Mathf.Repeat(t, length); }
        public static bool RoughlyEqual(float a, float b, float threshold) { return UnityEngine.Mathf.Abs(a - b) <= threshold; }
        public static float Round(float f) { return UnityEngine.Mathf.Round(f); }
        public static int RoundToInt(float f) { return UnityEngine.Mathf.RoundToInt(f); }
        public static float Sign(float f) { return UnityEngine.Mathf.Sign(f); }
        public static float Sin(float f) { return UnityEngine.Mathf.Sin(f); }
        public static float SmoothDamp(float current, float target, ref float currentVelocity, float smoothTime) { return UnityEngine.Mathf.SmoothDamp(current, target, ref currentVelocity, smoothTime); }
        public static float SmoothDamp(float current, float target, ref float currentVelocity, float smoothTime, float maxSpeed) { return UnityEngine.Mathf.SmoothDamp(current, target, ref currentVelocity, smoothTime, maxSpeed); }
        public static float SmoothDamp(float current, float target, ref float currentVelocity, float smoothTime, float maxSpeed, float deltaTime) { return UnityEngine.Mathf.SmoothDamp(current, target, ref currentVelocity, smoothTime, maxSpeed, deltaTime); }
        public static float SmoothDampAngle(float current, float target, ref float currentVelocity, float smoothTime) { return UnityEngine.Mathf.SmoothDampAngle(current, target, ref currentVelocity, smoothTime); }
        public static float SmoothDampAngle(float current, float target, ref float currentVelocity, float smoothTime, float maxSpeed) { return UnityEngine.Mathf.SmoothDampAngle(current, target, ref currentVelocity, smoothTime, maxSpeed); }
        public static float SmoothDampAngle(float current, float target, ref float currentVelocity, float smoothTime, float maxSpeed, float deltaTime) { return UnityEngine.Mathf.SmoothDampAngle(current, target, ref currentVelocity, smoothTime, maxSpeed, deltaTime); }
        public static float SmoothStep(float from, float to, float t) { return UnityEngine.Mathf.SmoothStep(from, to, t); }
        public static float Sqrt(float f) { return UnityEngine.Mathf.Sqrt(f); }
        public static float Tan(float f) { return UnityEngine.Mathf.Tan(f); }

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
                x = (cosTheta * (pointToRotate.x - centerPoint.x) -
                    sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x),
                y = (sinTheta * (pointToRotate.x - centerPoint.x) +
                    cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y)
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
            Vector2 result = Vector2.zero;

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

        private static float LengthSquared(Vector2 vec) {
            return (vec.x * vec.x) + (vec.y * vec.y);
        }

        private static float DistanceToLine(Vector2 point, Vector2 a, Vector2 b) {
            float l2 = LengthSquared(b - a);
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

            dx = b.x - a.x;
            dy = b.y - a.y;

            A = dx * dx + dy * dy;
            B = 2 * (dx * (a.x - center.x) + dy * (a.y - center.y));
            C = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y) - radius * radius;

            det = B * B - 4 * A * C;

            if (A <= 0.0000001 || det < 0)
                return false;
            else if (det == 0) {
                t = -B / (2 * A);

                if (0 <= t && t <= 1)
                    intersects.Add(new Vector2(a.x + t * dx, a.y + t * dy));
            } else {
                t = (-B + Mathf.Sqrt(det)) / (2 * A);
                if (0 <= t && t <= 1)
                    intersects.Add(new Vector2(a.x + t * dx, a.y + t * dy));
                t = (-B - Mathf.Sqrt(det)) / (2 * A);
                if (0 <= t && t <= 1)
                    intersects.Add(new Vector2(a.x + t * dx, a.y + t * dy));
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
            intersection = Vector2.zero;

            Vector2 bLowerLeft = new Vector2(b1.x < b2.x ? b1.x : b2.x, b1.y > b2.y ? b1.y : b2.y);
            Vector2 bUpperRight = new Vector2(b1.x > b2.x ? b1.x : b2.x, b1.y < b2.y ? b1.y : b2.y);

            if ((a1.x < bLowerLeft.x && a2.x < bLowerLeft.x)
                || (a1.y > bLowerLeft.y && a2.y > bLowerLeft.y)
                || (a1.x > bUpperRight.x && a2.x > bUpperRight.x)
                || (a1.y < bUpperRight.y && a2.y < bUpperRight.y))
                return false;

            Vector2 b = a2 - a1;
            Vector2 d = b2 - b1;

            float bDotDPerp = b.x * d.y - b.y * d.x;

            if (bDotDPerp == 0)
                return false;

            Vector2 c = b1 - a1;
            float t = (c.x * d.y - c.y * d.x) / bDotDPerp;
            if (t < 0 || t > 1)
                return false;

            float u = (c.x * b.y - c.y * b.x) / bDotDPerp;
            if (u < 0 || u > 1)
                return false;

            intersection = a1 + t * b;

            return true;
        }
        #endregion
    }
}