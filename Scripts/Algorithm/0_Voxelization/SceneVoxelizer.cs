using UnityEngine;

namespace NavMeshGraph.Voxelization
{
    public static class SceneVoxelizer
    {
        private const float OVERLAP_LENIENCE = .99f;

        public static VoxelSet Voxelize(float voxelSize, Bounds bounds, ISceneRaycaster raycaster, ISceneOverlapTester overlapTester)
        {
            var cx = Mathf.RoundToInt(bounds.size.x / voxelSize);
            var cy = Mathf.CeilToInt(bounds.size.y / voxelSize);
            var cz = Mathf.RoundToInt(bounds.size.z / voxelSize);

            var floor = new VoxelSet();

            for (int ix = 0; ix < cx; ix++)
                for (int iz = 0; iz < cz; iz++) //Loop over x-z surface area
                {
                    var currentVoxel = new Vector3Int(ix, cy + 1, iz);
                    var size = voxelSize * Vector3.one;

                    while (true) //loop over y-axis hits aka "floors"
                    {
                        var worldPos = VoxelToWorld(voxelSize, bounds, currentVoxel);
                        var hitIsValidFloor = FindNextFloor(worldPos, size, raycaster, out float nextFloorHeight);
                        var boundsLocalFloorHeight = nextFloorHeight - bounds.min.y;
                        currentVoxel = currentVoxel._x0z() + Vector3Int.up * Mathf.RoundToInt(boundsLocalFloorHeight / voxelSize); //voxelspace position of potential floor

                        if (hitIsValidFloor) //has hit floor -> add to voxels, continue one voxel lower
                        {
                            var floorCandidate = currentVoxel;
                            if (!isInsideGeo(voxelSize, bounds, floorCandidate, overlapTester)) floor.Add(floorCandidate);
                            else if (!isInsideGeo(voxelSize, bounds, floorCandidate + Vector3Int.up, overlapTester)) floor.Add(floorCandidate + Vector3Int.up);
                        }
                        currentVoxel += Vector3Int.down * 2;
                        if (nextFloorHeight < bounds.min.y) break; //hit bottom of bounds, exit
                    }
                }
            return floor;
        }

        private static bool isInsideGeo(float voxelSize, Bounds bounds, Vector3Int voxel, ISceneOverlapTester overlapTester)
        {
            var origin = VoxelToWorld(voxelSize, bounds, voxel);
            return overlapTester.BoxOverlaps(origin, Vector3.one * voxelSize * OVERLAP_LENIENCE);
        }

        private static bool FindNextFloor(Vector3 center, Vector3 size, ISceneRaycaster raycaster, out float maxHit)
        {
            var origins = new Vector3[]
            {
            center + size._x0z() / 2f * OVERLAP_LENIENCE,
            center - size._x0z() / 2f * OVERLAP_LENIENCE,
            center + (size._x00() - size._00z()) / 2f * OVERLAP_LENIENCE,
            center - (size._x00() - size._00z()) / 2f * OVERLAP_LENIENCE,
            };
            var minHit = float.MaxValue;
            maxHit = float.MinValue;
            foreach (var origin in origins)
            {
                if (!raycaster.Raycast(origin, Vector3.down, out var hit))
                {
                    minHit = float.MinValue;
                    maxHit = float.MinValue;
                    return false;
                }
                maxHit = Mathf.Max(maxHit, hit.y);
                minHit = Mathf.Min(minHit, hit.y);
                if (maxHit - minHit > size.y - float.Epsilon) return false;
            }
            return true;
        }

        private static Vector3 VoxelToWorld(float voxelSize, Bounds bounds, Vector3Int voxel) => bounds.min + (voxel + Vector3.one * .5f) * voxelSize;


    }
}