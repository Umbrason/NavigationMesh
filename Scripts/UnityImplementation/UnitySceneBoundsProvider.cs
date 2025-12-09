using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

public class UnitySceneBoundsProvider : ISceneBoundsProvider
{
    public readonly int Layermask;

    public UnitySceneBoundsProvider(int layermask)
    {
        Layermask = layermask;
    }

    public Bounds GetBounds()
    {
        var bounds = new Bounds();
        var colliders = SceneManager.GetActiveScene().GetRootGameObjects()
                                        .SelectMany(go => go.GetComponentsInChildren<Collider>())
                                        .Where(c => ((1 << c.gameObject.layer) & Layermask) > 0)
                                        .ToArray();
        if (colliders.Length == 0) return bounds;
        bounds = colliders[0].bounds;
        for (int i = 1; i < colliders.Length; i++)
        {
            var c = colliders[i];
            bounds.Encapsulate(c.bounds);
        }
        return bounds;
    }
}