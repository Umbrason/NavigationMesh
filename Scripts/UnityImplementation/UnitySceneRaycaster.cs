using System.Linq;
using UnityEngine;

public class UnitySceneRaycaster : ISceneRaycaster
{
    public readonly int Layermask;

    public UnitySceneRaycaster(int layermask)
    {
        Layermask = layermask;
    }

    public bool Raycast(Vector3 origin, Vector3 direction, out Vector3 hit, bool ignoreBackfaces = true, float maxDistance = float.PositiveInfinity)
    {
        using (new TempValueOverride<bool>(Physics.queriesHitBackfaces, !ignoreBackfaces, (v) => Physics.queriesHitBackfaces = v))
        {
            var result = Physics.Raycast(origin, direction, out var rayhit, maxDistance, Layermask);
            hit = rayhit.point;
            return result;
        }
    }

    public Vector3[] RaycastAll(Vector3 origin, Vector3 direction, bool ignoreBackfaces = true, float maxDistance = float.PositiveInfinity)
    {
        using (new TempValueOverride<bool>(Physics.queriesHitBackfaces, !ignoreBackfaces, (v) => Physics.queriesHitBackfaces = v))
        {
            var hits = Physics.RaycastAll(origin, direction, maxDistance, Layermask);
            return hits.Select(hit => hit.point).ToArray();
        }
    }
}