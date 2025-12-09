using UnityEngine;
using System.Linq;

using NavMeshGraph;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class NavMeshSurface : MonoBehaviour
{
    [HideInInspector, SerializeField] private Bounds bounds;
    [field: SerializeField] public NavMesh Data { get; private set; }
    public int Layermask = 1;
    public float VoxelSize = .2f;

    public void GenerateNavMesh()
    {
        var sceneRaycaster = new UnitySceneRaycaster(Layermask);
        var boundsProvider = new UnitySceneBoundsProvider(Layermask);
        var overlapTester = new UnitySceneOverlapTester(Layermask);
        Data = NavMeshBuilder.Build(VoxelSize, sceneRaycaster, boundsProvider, overlapTester);
#if UNITY_EDITOR
        EditorUtility.SetDirty(this);
#endif        
    }


#if UNITY_EDITOR
    void OnDrawGizmos()
    {
        DrawBounds();
        DrawPolygons();
    }

    private void DrawBounds()
    {
        Gizmos.DrawWireCube(bounds.center, bounds.size);
    }

    internal Mesh surfaceMesh;
    private void DrawPolygons()
    {
        if (Data == null) return;
        var graph = Data.Graph;
        Random.InitState(1);
        if (surfaceMesh == null)
        {
            surfaceMesh = new Mesh()
            {
                vertices = graph.Vertices,
                triangles = graph.GraphNodes.SelectMany(node => node.Points).ToArray(),
                normals = graph.Vertices.Select(_ => Vector3.up).ToArray()
            };
        }
        Gizmos.color = Color.HSVToRGB(.5f, .6f, 1);
        Gizmos.color = new Color(Gizmos.color.r, Gizmos.color.g, Gizmos.color.b, .2f);
        Gizmos.DrawMesh(surfaceMesh);

        for (int i = 0; i < graph.GraphNodes.Length; i++)
        {
            var node = graph.GraphNodes[i];
            Gizmos.color = Color.HSVToRGB(.5f, .8f, 1);
            Gizmos.color = new Color(Gizmos.color.r, Gizmos.color.g, Gizmos.color.b, .2f);
            var points = node.Points.Select(i => graph.Vertices[i]).ToArray();
            Gizmos.DrawLineStrip(points, true);
            var nodeCenter = points.Aggregate((a, b) => a + b) / 3f;
        }
        Gizmos.color = new Color(1, 1, 1, .2f);
        Gizmos.color = Color.white;
    }
#endif
}


#if UNITY_EDITOR
[CustomEditor(typeof(NavMeshSurface))]
public class NavMeshSurfaceEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        if (GUILayout.Button("Bake"))
        {
            foreach (var target in targets)
                (target as NavMeshSurface).GenerateNavMesh();
            (target as NavMeshSurface).surfaceMesh = null;
        }
    }
}
#endif

