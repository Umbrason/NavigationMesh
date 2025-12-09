using UnityEngine;

public interface ISceneRaycaster
{
    bool Raycast(Vector3 origin, Vector3 direction, out Vector3 hit, bool ignoreBackfaces = true, float maxDistance = float.PositiveInfinity);
    Vector3[] RaycastAll(Vector3 origin, Vector3 direction, bool ignoreBackfaces = true, float maxDistance = float.PositiveInfinity);
}

