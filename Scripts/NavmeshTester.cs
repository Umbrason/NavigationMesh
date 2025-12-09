using System.Linq;
using UnityEngine;

public class NavmeshTester : MonoBehaviour
{
    public AStarNavigation navigator;
    public MeshFilter meshFilter;
    public Transform target;

    void Update()
    {
        var m = meshFilter.sharedMesh ??= new();
        var path = navigator.Navigate(transform.position, target.position);
        if (path == null)
        {
            Destroy(meshFilter.sharedMesh);
            meshFilter.sharedMesh = null;
            return;
        }
        m.Clear();
        m.vertices = path.Select(p => transform.InverseTransformPoint(p)).ToArray();
        m.SetIndices(Enumerable.Range(0, path.Length).ToArray(), MeshTopology.LineStrip, 0);
        m.RecalculateBounds();
    }

    void OnDrawGizmos()
    {
        var m = meshFilter.sharedMesh ??= new();
        var pathCorners = m.vertices;
        var pathLength = 0f;
        for (int i = 1; i < pathCorners.Length; i++)
        {
            var A = pathCorners[i];
            var B = pathCorners[i - 1];
            var localLength = (A - B).magnitude;
            pathLength += localLength;
        }
    }
}
