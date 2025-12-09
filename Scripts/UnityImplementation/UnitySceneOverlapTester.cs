using UnityEngine;

public class UnitySceneOverlapTester : ISceneOverlapTester
{
    public readonly int Layermask;
    public UnitySceneOverlapTester(int layermask)
    {
        Layermask = layermask;
    }

    public bool BoxOverlaps(Vector3 position, Vector3 size)
    {
        return Physics.CheckBox(position, size / 2f, Quaternion.identity, Layermask);
    }
}