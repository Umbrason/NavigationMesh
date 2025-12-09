using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace NavMeshGraph
{
    [System.Serializable]
    public class NavMesh
    {
        //QuadTree
        [System.Serializable]
        public class QuadTree
        {
            public QuadTree childNE;
            public QuadTree childNW;
            public QuadTree childSE;
            public QuadTree childSW;
            public QuadTree[] Children => new[] { childNE, childNW, childSE, childSW };

            public List<int> ContainedGraphNodes;

            public Vector2 Center { get; set; }
            public float Size { get; set; }

            public Vector2 Min => Center - Vector2.one * Size * .5f;
            public Vector2 Max => Center + Vector2.one * Size * .5f;
        }

        //Bounds
        public NavMeshGraph Graph;
        [System.Serializable]
        public class NavMeshGraph
        {
            public Vector3[] Vertices;

            [System.Serializable]
            public class GraphNode
            {
                public GraphNode(int id, int pointA, int pointB, int pointC, int neighbourAB = -1, int neighbourBC = -1, int neighbourCA = -1)
                {
                    this.id = id;
                    PointA = pointA;
                    PointB = pointB;
                    PointC = pointC;
                    NeighbourAB = neighbourAB;
                    NeighbourBC = neighbourBC;
                    NeighbourCA = neighbourCA;
                    HalfEdges = new HalfEdge[] { new(PointA, PointB), new(PointB, PointC), new(PointA, PointC) };
                }
                public int id;
                public int PointA
                {
                    get => Points[0]; set
                    {
                        Points[0] = value;
                        HalfEdges[0] = new(value, PointB);
                        HalfEdges[2] = new(value, PointC);
                    }
                }
                public int PointB
                {
                    get => Points[1]; set
                    {
                        Points[1] = value;
                        HalfEdges[0] = new(PointA, value);
                        HalfEdges[1] = new(value, PointC);
                    }
                }
                public int PointC
                {
                    get => Points[2]; set
                    {
                        Points[2] = value;
                        HalfEdges[1] = new(PointB, value);
                        HalfEdges[2] = new(PointA, value);
                    }
                }
                public int[] Points = new int[3];
                public HalfEdge[] HalfEdges = new HalfEdge[3];

                [System.Serializable]
                public struct HalfEdge
                {
                    public int A;
                    public int B;

                    public HalfEdge(int a, int b)
                    {
                        A = a;
                        B = b;
                    }
                }

                public int NeighbourAB { get => Neighbours[0]; set => Neighbours[0] = value; }
                public int NeighbourBC { get => Neighbours[1]; set => Neighbours[1] = value; }
                public int NeighbourCA { get => Neighbours[2]; set => Neighbours[2] = value; }
                public int[] Neighbours = new int[3];
                public int NeighbourByEdge(int edgeA, int edgeB)
                {
                    if (GraphUtil.HalfEdgeHash(edgeA, edgeB) == GraphUtil.HalfEdgeHash(PointA, PointB)) return NeighbourAB;
                    if (GraphUtil.HalfEdgeHash(edgeA, edgeB) == GraphUtil.HalfEdgeHash(PointB, PointC)) return NeighbourBC;
                    if (GraphUtil.HalfEdgeHash(edgeA, edgeB) == GraphUtil.HalfEdgeHash(PointC, PointA)) return NeighbourCA;
                    return -1;
                }
            }
            public GraphNode[] GraphNodes;

            public bool NodeContains(GraphNode node, Vector2 point)
            {
                var A = Vertices[node.PointA];
                var B = Vertices[node.PointB];
                var C = Vertices[node.PointC];
                return NavMeshMath.PointInTriangle2D(point, A._xz(), B._xz(), C._xz());
            }

            public GraphNode NodeBelow(Vector3 point, List<int> nodeCandidates = null)
            {
                point += Vector3.up * .5f;
                var nodes = nodeCandidates?.Select(i => GraphNodes[i]) ?? GraphNodes;
                var minDepth = float.MinValue;
                var bestNode = (GraphNode)null;
                foreach (var node in nodes)
                {
                    if (!NodeContains(node, point._xz())) continue;
                    var verts = node.Points.Select(p => Vertices[p]).ToArray();
                    var normal = Vector3.Cross(verts[0] - verts[1], verts[2] - verts[1]).normalized;
                    if (normal.y < 0) normal = -normal;
                    var distance = Vector3.Dot(normal, point - verts[0]) * Vector3.Dot(normal, Vector3.up);

                    if ((distance > 0 && distance < minDepth) || (minDepth < 0 && distance > minDepth))
                    {
                        minDepth = distance;
                        bestNode = node;
                    }
                }
                return bestNode;
            }

            public (int, int) SharedEdge(GraphNode A, GraphNode B)
            {
                if (A == B || A == null || B == null) return (-1, -1);
                for (int i = 0; i < 3; i++)
                {
                    var halfEdgeHashA = GraphUtil.HalfEdgeHash(A.Points[i], A.Points[(i + 1) % 3]);
                    for (int j = 0; j < 3; j++)
                    {
                        var halfEdgeHashB = GraphUtil.HalfEdgeHash(B.Points[j], B.Points[(j + 1) % 3]);
                        if (halfEdgeHashA == halfEdgeHashB) return (A.Points[i], A.Points[(i + 1) % 3]);
                    }
                }
                return (-1, -1);
            }
        }

    }
}