using UnityEngine;

namespace NavMeshGraph
{
    public static class NavMeshMath
    {
        public static bool PointInTriangle(Vector3 point, Vector3 A, Vector3 B, Vector3 C, float maxPlaneDistance = float.MaxValue)
        {
            if (Vector3.Distance(A, point) < float.Epsilon
                || Vector3.Distance(B, point) < float.Epsilon
                || Vector3.Distance(C, point) < float.Epsilon)
                return false;
            var planeDistance = Mathf.Abs(Vector3.Dot(TriangleNormal(A, B, C), point - A));
            if (planeDistance > maxPlaneDistance) return false;
            return PointInTriangle2D(point._xz(), A._xz(), B._xz(), C._xz());
        }

        public static bool PointInTriangle2D(Vector2 point, Vector2 A, Vector2 B, Vector2 C)
        {
            if (Vector2.Distance(A, point) < float.Epsilon
                || Vector2.Distance(B, point) < float.Epsilon
                || Vector2.Distance(C, point) < float.Epsilon)
                return false;
            return SameSide(point, C, A, B) && SameSide(point, A, B, C) && SameSide(point, B, C, A);
        }

        public static bool PointInRect2D(Vector2 point, Vector2 min, Vector2 max)
        {
            if (point.x < min.x || point.y < min.y) return false;
            if (point.x > max.x || point.y > max.y) return false;
            return true;
        }

        public static Vector3 TriangleNormal(Vector3 A, Vector3 B, Vector3 C)
        {
            var n = Vector3.Cross(A - B, B - C);
            if (n.y < 0) n = -n;
            return n.normalized;
        }

        public static bool SameSide(Vector3 p1, Vector3 p2, Vector3 a, Vector3 b)
        {
            var cp1 = Vector3.Cross(b - a, p1 - a);
            var cp2 = Vector3.Cross(b - a, p2 - a);
            if (Vector3.Dot(cp1, cp2) >= 0) return true;
            return false;
        }

        public static bool TriangleBoxOverlap(Vector2 a, Vector2 b, Vector2 c, Vector2 min, Vector2 max)
        {
            //any triangle points in box?
            if (PointInRect2D(a, min, max)) return true;
            if (PointInRect2D(b, min, max)) return true;
            if (PointInRect2D(c, min, max)) return true;

            //any box corners in triangle
            var x = new Vector2(min.x, max.y);
            var y = new Vector2(min.x, max.y);
            var z = new Vector2(min.x, max.y);
            var w = new Vector2(min.x, max.y);

            //any line intersections?
            if (PointInTriangle2D(x, a, b, c)) return true;
            if (PointInTriangle2D(y, a, b, c)) return true;
            if (PointInTriangle2D(z, a, b, c)) return true;
            if (PointInTriangle2D(w, a, b, c)) return true;

            if (LineIntersect(x, y, a, b)) return true;
            if (LineIntersect(y, z, a, b)) return true;
            if (LineIntersect(z, w, a, b)) return true;
            if (LineIntersect(w, x, a, b)) return true;

            if (LineIntersect(x, y, b, c)) return true;
            if (LineIntersect(y, z, b, c)) return true;
            if (LineIntersect(z, w, b, c)) return true;
            if (LineIntersect(w, x, b, c)) return true;

            if (LineIntersect(x, y, c, a)) return true;
            if (LineIntersect(y, z, c, a)) return true;
            if (LineIntersect(z, w, c, a)) return true;
            if (LineIntersect(w, x, c, a)) return true;

            //they dont overlap
            return false;
        }

        static bool OnLineSegment(Vector2 p, Vector2 q, Vector2 r)
        {
            if (q.x <= Mathf.Max(p.x, r.x) && q.x >= Mathf.Min(p.x, r.x) &&
                q.y <= Mathf.Max(p.y, r.y) && q.y >= Mathf.Min(p.y, r.y))
                return true;

            return false;
        }

        static int Orientation(Vector2 p, Vector2 q, Vector2 r)
        {
            float val = (q.y - p.y) * (r.x - q.x) -
                    (q.x - p.x) * (r.y - q.y);

            if (val == 0) return 0;
            return (val > 0) ? 1 : 2;
        }

        static bool LineIntersect(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2)
        {
            int o1 = Orientation(p1, q1, p2);
            int o2 = Orientation(p1, q1, q2);
            int o3 = Orientation(p2, q2, p1);
            int o4 = Orientation(p2, q2, q1);

            if (o1 != o2 && o3 != o4)
                return true;

            if (o1 == 0 && OnLineSegment(p1, p2, q1)) return true;
            if (o2 == 0 && OnLineSegment(p1, q2, q1)) return true;
            if (o3 == 0 && OnLineSegment(p2, p1, q2)) return true;
            if (o4 == 0 && OnLineSegment(p2, q1, q2)) return true;
            return false;
        }


        public static Vector3 LineSegmentProjection(Vector3 Point, Vector3 A, Vector3 B)
        {
            var AB = B - A;
            var Proj = Mathf.Clamp01(Vector3.Dot(Point - A, AB) / AB.sqrMagnitude) * AB + A;
            return Proj;
        }

        public static Vector3 LineIntersection(Vector3 A, Vector3 B, Vector3 C, Vector3 D)
        {
            var AB = B - A;

            var ProjC = Vector3.Dot(C - A, AB) * AB / AB.sqrMagnitude + A;
            var ProjD = Vector3.Dot(D - A, AB) * AB / AB.sqrMagnitude + A;
            var distC = (ProjC - C).magnitude;
            var distD = (ProjD - D).magnitude;
            if (distC == 0) return ProjC;
            if (distD == 0) return ProjD;
            return (distD * ProjC + distC * ProjD) / (distC + distD);
        }
    }
}