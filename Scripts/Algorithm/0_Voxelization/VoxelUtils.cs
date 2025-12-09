using UnityEngine;

namespace NavMeshGraph.Voxelization
{
    public static class VoxelUtils
    {
        public static readonly Vector3Int[] HorizontalNeighbours = new Vector3Int[]
        {
        //n4
        new(1,0,0),
        new(0,0,1),
        new(-1,0,0),
        new(0,0,-1),
        
        //n8
        new(1,0,1),
        new(1,0,-1),
        new(-1,0,1),
        new(-1,0,-1),
        };
    }
}