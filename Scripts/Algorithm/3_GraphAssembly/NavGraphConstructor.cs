using System.Collections.Generic;
using UnityEngine;

namespace NavMeshGraph.GraphAssembly
{
    using Triangulation;
    using static NavMeshGraph.NavMesh.NavMeshGraph;

    public static class NavGraphConstructor
    {
        public static NavMesh FromTriangles(List<NavMeshTriangulator.NavMeshTriangle> triangles, List<Vector3> positions)
        {
            var data = new NavMesh() { Graph = ConstructGraph(triangles, positions) };
            return data;
        }

        private static NavMesh.NavMeshGraph ConstructGraph(List<NavMeshTriangulator.NavMeshTriangle> triangles, List<Vector3> positions)
        {
            var Graph = new NavMesh.NavMeshGraph
            {
                Vertices = positions.ToArray(),
                GraphNodes = new GraphNode[triangles.Count]
            };
            Dictionary<uint, int> HalfEdgeLookup = new();
            for (int i = 0; i < triangles.Count; i++)
            {
                var triangle = triangles[i];
                var node = new GraphNode(i, triangle.A, triangle.B, triangle.C);
                foreach (var edge in node.HalfEdges)
                {
                    if (TryGetOtherByTriangleEdge(edge.A, edge.B, node, HalfEdgeLookup, out var neighbourID))
                    {
                        SetHalfEdgeNeighbourID(node, edge.A, edge.B, neighbourID);
                        var neighbour = Graph.GraphNodes[neighbourID];
                        SetHalfEdgeNeighbourID(neighbour, edge.A, edge.B, node.id);
                    }
                }
                Graph.GraphNodes[i] = node;
            }
            return Graph;
        }

        private static void SetHalfEdgeNeighbourID(GraphNode node, int a, int b, int neighbourID)
        {
            var halfEdgeHash = GraphUtil.HalfEdgeHash(a, b);
            if (GraphUtil.HalfEdgeHash(node.PointA, node.PointB) == halfEdgeHash) { node.NeighbourAB = neighbourID; return; }
            if (GraphUtil.HalfEdgeHash(node.PointB, node.PointC) == halfEdgeHash) { node.NeighbourBC = neighbourID; return; }
            if (GraphUtil.HalfEdgeHash(node.PointC, node.PointA) == halfEdgeHash) { node.NeighbourCA = neighbourID; return; }
            throw new System.Exception($"Edge ({a},{b}) not found in node {node.id}");
        }

        private static bool TryGetOtherByTriangleEdge(int A, int B, GraphNode node, Dictionary<uint, int> TriangleEdgeLookup, out int index)
        {
            var hash = GraphUtil.HalfEdgeHash(A, B);
            if (TriangleEdgeLookup.TryGetValue(hash, out index))
                return true;
            TriangleEdgeLookup.Add(hash, index = node.id);
            return false;
        }


    }
}