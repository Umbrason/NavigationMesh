using UnityEngine;

public interface ISceneOverlapTester
{
    bool BoxOverlaps(Vector3 position, Vector3 size);
}