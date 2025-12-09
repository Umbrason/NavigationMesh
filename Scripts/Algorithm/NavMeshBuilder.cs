using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace NavMeshGraph
{
    using static Voxelization.SceneVoxelizer;
    using static Voxelization.SurfaceIdentifier;
    using static Triangulation.NavMeshTriangulator;
    using static Outlining.SceneOutliner;
    using static GraphAssembly.NavGraphConstructor;
    public static class NavMeshBuilder
    {
        public static NavMesh Build(float VoxelSize, UnitySceneRaycaster sceneRaycaster, UnitySceneBoundsProvider boundsProvider, UnitySceneOverlapTester overlapTester)
        {
            var bounds = boundsProvider.GetBounds();
            var floorVoxels = Voxelize(VoxelSize, bounds, sceneRaycaster, overlapTester);
            var surfaces = SplitSurfaces(floorVoxels).ToArray();
            var Positions = new List<Vector3>();
            var Triangles = new List<NavMeshTriangle>();
            foreach (var surfaceVoxels in surfaces)
            {
                var outline = GenerateOutline(surfaceVoxels, sceneRaycaster, VoxelSize, bounds);
                var triangleSurface = FromOutline(outline, sceneRaycaster, VoxelSize, idOffset: Positions.Count);
                Triangles.AddRange(triangleSurface.Triangles);
                Positions.AddRange(triangleSurface.Positions);
            }
            return FromTriangles(Triangles, Positions);
        }
    }
}