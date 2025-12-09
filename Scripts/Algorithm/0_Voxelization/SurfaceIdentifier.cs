using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace NavMeshGraph.Voxelization
{
    public static class SurfaceIdentifier
    {
        public static IEnumerable<VoxelSet> SplitSurfaces(VoxelSet set)
        {
            var surfaces = new List<VoxelSet>();
            while (set.Count > 0) //floodfill n4 until no voxels are left
            {
                var surfaceSet = new VoxelSet();
                var fillQueue = new Queue<Vector3Int>();
                fillQueue.Enqueue(set.First());
                while (fillQueue.Count > 0)
                {
                    var current = fillQueue.Dequeue();
                    if (!set.Contains(current)) continue;
                    set.Remove(current);
                    surfaceSet.Add(current);
                    for (int i = 0; i < 4; i++)
                    {
                        for (int dy = -1; dy <= 1; dy++)
                        {
                            var nb = current + VoxelUtils.HorizontalNeighbours[i] + Vector3Int.up * dy;
                            if (!set.Contains(nb)) continue;
                            fillQueue.Enqueue(nb);
                        }
                    }
                }
                surfaces.Add(surfaceSet);
            }
            return surfaces;
        }
    }
}