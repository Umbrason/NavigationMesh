using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using static NavMeshGraph.NavMesh.NavMeshGraph;
using NavMeshGraph;

public class AStarNavigation : MonoBehaviour
{
    [Range(0, 100)] public int optimizationIterations;
    public NavMeshSurface surface;
    public Vector3[] Navigate(Vector3 start, Vector3 end)
    {
        var graph = surface.Data.Graph;
        var startNode = graph?.NodeBelow(start);
        var endNode = graph?.NodeBelow(end);
        if (startNode == null || endNode == null) return null;
        if (startNode == endNode) return new Vector3[] { start, end };

        var exploreQueue = new PriorityQueue<AStarEdgeNode, float>();
        var exploredNodes = new Dictionary<AStarEdgeID, AStarEdgeNode>();
        var startEdge = new AStarEdgeNode(start, graph, startNode)
        {
            fromNode = null
        };
        exploredNodes[startEdge.edgeID] = startEdge;
        exploreQueue.Enqueue(startEdge, 0);
        var exploreBuffer = new AStarEdgeNode[AStarEdgeID.VertsPerEdge * 3];
        while (exploreQueue.Count > 0)
        {
            var currentAStar = exploreQueue.Dequeue();
            if (currentAStar.fromNode == endNode) // found target
            {
                var output = new List<AStarEdgeNode>();
                do
                {
                    output.Insert(0, currentAStar);
                    if (currentAStar.fromEdge.GetHashCode() == currentAStar.edgeID.GetHashCode()) break;
                    currentAStar = exploredNodes[currentAStar.fromEdge];
                } while (exploredNodes.ContainsKey(currentAStar.fromEdge));
                return OptimizePath(output, optimizationIterations);
            }
            if (currentAStar.toNode == endNode) //next node contains target - only add target position instead of other edges
            {
                var AStarEndNode = new AStarEdgeNode(end, graph, endNode)
                {
                    distance = currentAStar.distance + (currentAStar.Position - end).magnitude,
                    fromEdge = currentAStar.edgeID
                };
                exploreQueue.Enqueue(AStarEndNode, AStarEndNode.distance);
                continue;
            }
            currentAStar.GetNextNodesNoAlloc(exploreBuffer);
            foreach (var nextNode in exploreBuffer) //default case - add all other nodes from the current triangle
            {
                if (nextNode == null) continue;
                if (exploredNodes.ContainsKey(nextNode.edgeID) && exploredNodes[nextNode.edgeID].distance <= nextNode.distance) continue;
                exploredNodes[nextNode.edgeID] = nextNode;
                exploreQueue.Enqueue(nextNode, nextNode.distance + (nextNode.Position - end).magnitude);
            }
        }
        return null;
    }

    public struct AStarEdgeID
    {
        public int A;
        public int B;
        public int id; //say max 8 edges (2^4)
        public const int VertsPerEdge = 3;

        public AStarEdgeID(int a, int b, int id)
        {
            A = a;
            B = b;
            this.id = id;
        }

        public override readonly int GetHashCode()
        {
            return Mathf.Min(A, B) | Mathf.Max(A, B) << 14 | id << 28;
        }
    }

    //problem is with traversing along edges for free
    class AStarEdgeNode //t can be 0-1 all t's must be added to exploration
    {
        public AStarEdgeID edgeID;
        public float t;
        public float distance;
        public GraphNode fromNode;
        public GraphNode toNode;
        public AStarEdgeID fromEdge;
        public NavMesh.NavMeshGraph graph;

        public AStarEdgeNode(AStarEdgeID edgeID, float distance, GraphNode fromNode, GraphNode toNode, AStarEdgeID fromEdge, NavMesh.NavMeshGraph graph)
        {
            this.edgeID = edgeID;
            this.distance = distance;
            this.fromNode = fromNode;
            this.toNode = toNode;
            this.fromEdge = fromEdge;
            this.graph = graph;
            this.t = (edgeID.id + .5f) / AStarEdgeID.VertsPerEdge;
            PositionA = graph.Vertices[edgeID.A];
            PositionB = graph.Vertices[edgeID.B];
        }

        public AStarEdgeNode(Vector3 FixedPosition, NavMesh.NavMeshGraph graph, GraphNode node)
        {
            this.graph = graph;
            fromNode = node;
            toNode = node;
            edgeID = new(-1, -1, -1);
            fromEdge = edgeID;
            PositionA = FixedPosition;
            PositionB = FixedPosition;
            t = 0;
        }

        public Vector3 PositionA;
        public Vector3 PositionB;
        public Vector3 Position => new(PositionB.x * t + PositionA.x * (1f - t),
                                       PositionB.y * t + PositionA.y * (1f - t),
                                       PositionB.z * t + PositionA.z * (1f - t));

        public void GetNextNodesNoAlloc(AStarEdgeNode[] array)
        {
            for (int i = 0; i < array.Length; i++) array[i] = null;
            if (toNode == null) return;
            var edges = toNode.HalfEdges;

            var (A, B) = graph.SharedEdge(fromNode, toNode);
            var halfEdgeHash = GraphUtil.HalfEdgeHash(A, B);

            var arrayIndex = 0;
            foreach (var halfEdge in edges)
            {
                int a = halfEdge.A;
                int b = halfEdge.B;
                if (GraphUtil.HalfEdgeHash(a, b) == halfEdgeHash) continue;
                var nbID = toNode.NeighbourByEdge(a, b);
                if (nbID < 0) continue;
                var nbNode = graph.GraphNodes[nbID];
                for (int i = 0; i < AStarEdgeID.VertsPerEdge; i++)
                {
                    var nextEdge = new AStarEdgeNode(new AStarEdgeID(a, b, i), distance, toNode, nbNode, edgeID, graph);
                    nextEdge.distance += (Position - nextEdge.Position).magnitude;
                    array[arrayIndex++] = nextEdge;
                }
            }
            return;
        }
    }




    Vector3[] OptimizePath(List<AStarEdgeNode> path, int iterations = 100)
    {
        var graph = surface.Data.Graph;
        for (int n = 0; n < iterations; n++)
        {
            for (int i = 1; i < path.Count - 1; i++)
            {
                var edge = path[i];
                var prev = path[i - 1].Position;
                var next = path[i + 1].Position;

                var AB = edge.PositionB - edge.PositionA;
                var intersect = NavMeshMath.LineIntersection(edge.PositionA, edge.PositionB, prev, next);
                var AIntersect = intersect - edge.PositionA;
                path[i].t = Mathf.Clamp01(Vector3.Dot(AIntersect, AB) / AB.sqrMagnitude);
            }
        }

        return path.Select(edge => edge.Position).ToArray();
    }
}
