using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace NavMeshGraph.Triangulation
{
    using Outlining;
    public static class NavMeshTriangulator
    {
        public class TriangleSurface
        {
            public List<NavMeshTriangle> Triangles { get; set; }
            public List<Vector3> Positions { get; set; }
        }
        public class NavMeshTriangle
        {
            public int A, B, C;

            public NavMeshTriangle(int a, int b, int c)
            {
                A = a;
                B = b;
                C = c;
            }
        }

        //pretty standard earlipping by projecting onto xz plane for angles
        //first checks overlap without regards to y axis, should that fail does another pass respecting y axis
        public static TriangleSurface FromOutline(SceneOutliner.Outline outline, ISceneRaycaster raycaster, float voxelSize, int idOffset = 0)
        {
            if (outline.indices.Length < 3) return null;
            var surface = new TriangleSurface
            {
                Positions = new List<Vector3>(outline.Vertices),
                Triangles = new()
            };
            var indices = outline.indices.ToList();

            while (indices.Count > 3)
            {
                var shortestDiag = float.MaxValue;
                var index = -1;
                for (int i = 0; i < indices.Count; i++)
                {
                    if (!IsEarStrict(indices, surface.Positions, i, raycaster, voxelSize)) continue;
                    //clip
                    var A_ID = indices[(i - 1 + indices.Count) % indices.Count];
                    var C_ID = indices[(i + 1 + indices.Count) % indices.Count];
                    var A = surface.Positions[A_ID];
                    var C = surface.Positions[C_ID];
                    var diagLength = (A - C).sqrMagnitude;
                    if (shortestDiag <= diagLength) continue;
                    shortestDiag = diagLength;
                    index = i;
                }
                if (index < 0)
                    for (int i = 0; i < indices.Count; i++)
                    {
                        if (!IsEarLoose(indices, surface.Positions, i)) continue;
                        //clip
                        var A_ID = indices[(i - 1 + indices.Count) % indices.Count];
                        var C_ID = indices[(i + 1 + indices.Count) % indices.Count];
                        var A = surface.Positions[A_ID];
                        var C = surface.Positions[C_ID];
                        var diagLength = (A - C).sqrMagnitude;
                        if (shortestDiag <= diagLength) continue;
                        shortestDiag = diagLength;
                        index = i;
                    }
                if (index < 0) break;
                var PA = indices[(index - 1 + indices.Count) % indices.Count];
                var PB = indices[index];
                var PC = indices[(index + 1 + indices.Count) % indices.Count];
                indices.RemoveAt(index);
                surface.Triangles.Add(new(PA + idOffset, PB + idOffset, PC + idOffset));
            }
            surface.Triangles.Add(new(indices[0] + idOffset, indices[1] + idOffset, indices[2] + idOffset));
            return surface;
        }

        private static bool IsEarLoose(List<int> indices, List<Vector3> positions, int index)
        {
            var iA = (index - 1 + indices.Count) % indices.Count;
            var iB = index;
            var iC = (index + 1 + indices.Count) % indices.Count;

            var A_ID = indices[iA];
            var B_ID = indices[iB];
            var C_ID = indices[iC];
            var A = positions[A_ID];
            var B = positions[B_ID];
            var C = positions[C_ID];
            var AB = A - B;
            var BC = B - C;
            var alpha = -Vector2.SignedAngle(AB._xz(), BC._xz());
            if (alpha <= 0 || alpha >= 180) return false;

            for (int i = 0; i < indices.Count; i++) //check other vert is in ear
            {
                if (i == iA || i == iB || i == iC) continue;
                if (PointInTriangleLoose(positions[indices[i]], A, B, C))
                    return false;
            }
            return true;
        }

        private static bool IsEarStrict(List<int> indices, List<Vector3> positions, int index, ISceneRaycaster raycaster, float voxelSize)
        {
            var iA = (index - 1 + indices.Count) % indices.Count;
            var iB = index;
            var iC = (index + 1 + indices.Count) % indices.Count;

            var A_ID = indices[iA];
            var B_ID = indices[iB];
            var C_ID = indices[iC];
            var A = positions[A_ID];
            var B = positions[B_ID];
            var C = positions[C_ID];

            var AB = A - B;
            var BC = B - C;
            var AC = A - C;
            var alpha = -Vector2.SignedAngle(AB._xz(), BC._xz());
            if (alpha <= 0 || alpha >= 180) return false;

            //check with raycaster
            var distance = AC.magnitude;
            if (raycaster.Raycast(C, AC, out _, false, distance)) return false;
            var ray = new Ray(C, AC);
            var voxelDist = Mathf.FloorToInt(distance / voxelSize);
            if (!Enumerable.Range(0, voxelDist).All(ri => raycaster.Raycast(ray.GetPoint(ri * voxelSize), Vector3.down, out _, false, voxelSize * 1.5f))) return false;

            for (int i = 0; i < indices.Count; i++) //check other vert is in ear
            {
                if (i == iA || i == iB || i == iC) continue;
                if (PointInTriangleStrict(positions[indices[i]], A, B, C))
                    return false;
            }
            return true;
        }

        private static bool PointInTriangleLoose(Vector3 point, Vector3 A, Vector3 B, Vector3 C) => NavMeshMath.PointInTriangle(point, A, B, C, .05f);
        private static bool PointInTriangleStrict(Vector3 point, Vector3 A, Vector3 B, Vector3 C) => NavMeshMath.PointInTriangle(point, A, B, C, .05f);
    }
}