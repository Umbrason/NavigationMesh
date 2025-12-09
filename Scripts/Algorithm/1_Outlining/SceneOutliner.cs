using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace NavMeshGraph.Outlining
{
    using Voxelization;

    public static class SceneOutliner
    {
        public struct Outline
        {
            public Vector3[] Vertices;
            public int[] indices;
        }
        public static Outline GenerateOutline(VoxelSet floorVoxels, ISceneRaycaster sceneRaycaster, float voxelSize, Bounds bounds)
        {
            var wallVoxels = floorVoxels.Copy().RemoveKernel(FillingKernel); //only outermost voxels of each island        
            var polygons = VoxelsToPolygons(wallVoxels, floorVoxels, bounds, voxelSize).ToArray(); //build polygons
            foreach (var polygon in polygons) Polygon_DecimateSoft(polygon);

            var poylgonsByCircumference = polygons.OrderByDescending(p => CalcCircumference(p.Select(v => v))).ToArray();
            for (int i = 0; i < poylgonsByCircumference.Length; i++)
            {
                var polygon = poylgonsByCircumference[i];
                var exteriorAngleSum = ExteriorAngle(polygon.Select(v => v));
                if (exteriorAngleSum > 0) polygon.Reverse();
                Polygon_DecimateAgressive(polygon, i > 0, sceneRaycaster, voxelSize);
            }

            var outlinePolygon = poylgonsByCircumference.First();
            var holePolygons = poylgonsByCircumference.Skip(1).ToArray();
            return MergePolygons(outlinePolygon, holePolygons, sceneRaycaster, voxelSize);
        }

        private static List<Vector3> Polygon_DecimateSoft(List<Vector3> polygon)
        {
            bool vertChanged;
            do
            {
                vertChanged = false;
                for (int i = 0; i < polygon.Count; i++)
                {
                    var A = polygon[(i - 1 + polygon.Count) % polygon.Count];
                    var B = polygon[i];
                    var C = polygon[(i + 1) % polygon.Count];
                    var AB = A - B;
                    var BC = B - C;
                    var alpha = Vector3.SignedAngle(AB, BC, Vector3.up);
                    if (alpha != 0) continue; //Colinear, not important                                
                    polygon.RemoveAt(i);
                    vertChanged = true;
                    break;
                }
            } while (vertChanged);
            return polygon;
        }

        private static List<Vector3> Polygon_DecimateAgressive(List<Vector3> polygon, bool isHole, ISceneRaycaster raycaster, float voxelSize)
        {
            bool vertChanged;
            do
            {
                vertChanged = false;
                for (int i = 0; i < polygon.Count; i++)
                {
                    var A = polygon[(i - 1 + polygon.Count) % polygon.Count];
                    var B = polygon[i];
                    var C = polygon[(i + 1) % polygon.Count];
                    var AB = A - B;
                    var BC = B - C;
                    var AC = A - C;
                    var alpha = Vector2.SignedAngle(AB._xz(), BC._xz());

                    var isConvex = alpha < 0 || alpha > 180;
                    if (isConvex != isHole && alpha != 0) continue; //would shrink polygon. bad

                    var hitsGeometry = raycaster.Raycast(C, AC, out _, false, AC.magnitude);
                    if (hitsGeometry) continue;

                    var hasFloorBelow = raycaster.Raycast(C + (AC / 2f), Vector3.down, out _, false, voxelSize * 1.5f);
                    if (!hasFloorBelow) continue;

                    polygon.RemoveAt(i);
                    vertChanged = true;
                    break;
                }
            } while (vertChanged);
            return polygon;
        }

        //rember filling to know what direction the wall is, to then know which direction to push the verts into
        private static List<Vector3> ConstructPolygon(VoxelSet wallVoxels, VoxelSet floorVoxels, Vector3Int startPosition, Bounds bounds, float voxelSize, out HashSet<Vector3Int> eliminatedPositions)
        {
            var verts = new List<Vector3>();
            eliminatedPositions = new();
            int bestTotalAngle = 3;
            for (int i = 0; i < 4; i++)
            {
                var newEliminatedPositions = MarchVoxelOutline(wallVoxels, floorVoxels, startPosition, i, bounds, voxelSize, out var newVerts, out var newWrongTurns, out var totalAngle);
                if (newEliminatedPositions.Count > eliminatedPositions.Count) goto Replace;
                else if (newEliminatedPositions.Count < eliminatedPositions.Count) continue;

                if (Mathf.Abs(totalAngle) % 2 < Mathf.Abs(bestTotalAngle) % 2) goto Replace;
                else if (Mathf.Abs(totalAngle) % 2 > Mathf.Abs(bestTotalAngle) % 2) continue;

                Replace:
                eliminatedPositions = newEliminatedPositions;
                bestTotalAngle = totalAngle;
                verts = newVerts;
            }
            return verts;
        }

        public static float CalcCircumference(IEnumerable<Vector3> verts)
        {
            var len = 0f;
            var vertArray = verts.ToArray();
            for (int i = 0; i < vertArray.Length; i++)
                len += Vector3.Distance(vertArray[i], vertArray[(i + 1) % vertArray.Length]);
            return len;
        }

        public static float ExteriorAngle(IEnumerable<Vector3> verts)
        {
            var exteriorAngleSum = 0f;
            var vertArray = verts.ToArray();
            for (int i = 0; i < vertArray.Length; i++)
            {
                var iA = (i - 1 + vertArray.Length) % vertArray.Length;
                var iB = i;
                var iC = (i + 1 + vertArray.Length) % vertArray.Length;

                var A = vertArray[iA];
                var B = vertArray[iB];
                var C = vertArray[iC];
                var AB = A - B;
                var BC = B - C;
                var angle = Vector2.SignedAngle(AB._xz(), BC._xz());
                if (Vector2.Dot(AB._xz().normalized, BC._xz().normalized) == -1)
                    angle = -180;
                exteriorAngleSum += angle;
            }
            return exteriorAngleSum;
        }

        private static Outline MergePolygons(List<Vector3> outline, List<Vector3>[] holes, ISceneRaycaster raycaster, float voxelSize)
        {
            var mergedVerts = new List<Vector3>(outline);
            var mergedIndices = new List<int>(Enumerable.Range(0, outline.Count));
            foreach (var hole in holes)
            {
                var pairs = Enumerable.Range(0, mergedVerts.Count).SelectMany(i => Enumerable.Range(0, hole.Count), (i, j) => (i, j));
                pairs = pairs.OrderBy(touple => (mergedVerts[touple.i] - hole[touple.j]).sqrMagnitude);
                var holeIndices = Enumerable.Range(mergedVerts.Count, hole.Count);
                foreach (var (outlineIndex, holeIndex) in pairs)
                {
                    var outerVert = mergedVerts[outlineIndex];
                    var innerVert = hole[holeIndex];
                    var delta = outerVert - innerVert;
                    var ray = new Ray(innerVert, delta);
                    var distance = delta.magnitude;
                    if (raycaster.Raycast(ray.origin, ray.direction, out _, false, distance)) continue; //blocked by geometry                    

                    var voxelDist = Mathf.FloorToInt(distance / voxelSize);
                    if (!Enumerable.Range(0, voxelDist).All(ri => raycaster.Raycast(ray.GetPoint(ri * voxelSize), Vector3.down, out _, false, voxelSize * 1.5f))) continue;

                    var insert = holeIndices.Skip(holeIndex).Concat(holeIndices.Take(holeIndex + 1)).Reverse().Append(outlineIndex);
                    mergedIndices.InsertRange(outlineIndex + 1, insert); //insert hole-polygon at right location and duplicate attachment vertices
                    mergedVerts.AddRange(hole);
                    break;
                }
            }
            return new() { Vertices = mergedVerts.ToArray(), indices = mergedIndices.ToArray() };
        }

        private static HashSet<Vector3Int> MarchVoxelOutline(VoxelSet voxels, VoxelSet floorVoxels, Vector3Int start, int initialDirection, Bounds bounds, float voxelSize, out List<Vector3> verts, out int wrongTurns, out int totalAngle)
        {
            var visited = new HashSet<Vector3Int>() { start };
            wrongTurns = 0;
            totalAngle = 0;
            verts = new();
            var currentPosition = start;
            var nbs = new Vector3Int[]
            {
            Vector3Int.left,
            Vector3Int.forward,
            Vector3Int.right,
            Vector3Int.back,
            };
            var currentDirection = initialDirection;

            { //check for invalid start direction
                var any = false;
                for (int dy = -1; dy <= 1; dy++)
                {
                    var nb = start + nbs[rotateDirection(initialDirection, -1)] + Vector3Int.up * dy;
                    if (voxels.Contains(nb)) any = true;
                }
                if (!any) return visited;
            }

            do
            {
                var oldDirection = currentDirection;
                int[] offsets = new int[] { -1, 0, 1, 2 };
                for (int i = 0; i < offsets.Length; i++)
                {
                    currentDirection = rotateDirection(oldDirection, offsets[i]);
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        var nb = currentPosition + nbs[currentDirection] + Vector3Int.up * dy;
                        if (voxels.Contains(nb))
                        {
                            var vertex = (Vector3)currentPosition;
                            var vertexOffset = Vector3.zero;
                            foreach (var offset in VoxelUtils.HorizontalNeighbours)
                            {
                                if (!Enumerable.Range(-1, 3).Any(dy => floorVoxels.Contains(currentPosition + offset + Vector3Int.up * dy))) continue;
                                vertexOffset -= offset;
                            }
                            vertexOffset = new Vector3(Mathf.Clamp(vertexOffset.x, -1, 1), 0, Mathf.Clamp(vertexOffset.z, -1, 1)) * .45f;
                            verts.Add(VoxelToWorld(vertex, bounds, voxelSize));
                            totalAngle += offsets[i];
                            currentPosition = nb;
                            visited.Add(currentPosition);
                            goto Break;
                        }
                    }
                    if (i == 1) wrongTurns++;
                }

            Break:;
            }
            while (currentPosition != start);
            return visited;
        }


        private static Vector3 VoxelToWorld(Vector3 voxel, Bounds bounds, float voxelSize) => bounds.min + (voxel + Vector3.one * .5f) * voxelSize;

        private static int rotateDirection(int dir, int amount)
        {
            dir += amount;
            while (dir < 0) dir += 4;
            dir %= 4;
            return dir;
        }

        private static readonly List<Vector2Int>[,] neighbourLookup3x3 = new List<Vector2Int>[,]
        {
            { new() {new(0, 1), new(1, 2)}, new() {new(0, 2), new(2, 2)}, new() {new(1, 2), new(2, 1)}},
            { new() {new(0, 0), new(0, 2)}, new() {                    }, new() {new(2, 0), new(2, 2)}},
            { new() {new(0, 1), new(1, 0)}, new() {new(0, 0), new(2, 0)}, new() {new(1, 0), new(2, 1)}},
        };

        private static bool FillingKernel(Vector3Int key, HashSet<Vector3Int> set)
        {
            var ddy = 1;
            int[,] dys = new int[3, 3];
            foreach (var offset in VoxelUtils.HorizontalNeighbours)
            {
                bool any = false;
                for (int dy = -ddy; dy <= ddy; dy++)
                {
                    if (!set.Contains(key + offset + Vector3Int.up * dy)) continue;
                    dys[offset.x + 1, offset.z + 1] = dy;
                    any = true;
                    break;
                }
                if (!any) return false;
            }
            foreach (var offset in VoxelUtils.HorizontalNeighbours)
            {
                var k = offset._xz() + Vector2Int.one;
                foreach (var neighbour in neighbourLookup3x3[2 - k.y, k.x])
                    if (Mathf.Abs(dys[neighbour.x, neighbour.y] - dys[k.x, k.y]) >= 2)
                        return false;
            }
            return true;
        }

        private static IEnumerable<List<Vector3>> VoxelsToPolygons(VoxelSet wallVoxels, VoxelSet floorVoxels, Bounds bounds, float voxelSize)
        {
            var visited = new HashSet<Vector3Int>();
            var polygons = new List<List<Vector3>>();
            var candidates = wallVoxels.OrderByDescending(p => VoxelUtils.HorizontalNeighbours.Count(offset => wallVoxels.Contains(offset + p)));
            while (wallVoxels.Count > visited.Count)
            {
                var position = candidates.First(p => !visited.Contains(p));
                var polygon = ConstructPolygon(wallVoxels, floorVoxels, position, bounds, voxelSize, out var polygonVisited);
                if (!polygonVisited.All(visited.Contains))
                    polygons.Add(polygon);
                foreach (var pos in polygonVisited) visited.Add(pos);
            }
            wallVoxels.Clear();
            return polygons;
        }
    }
}