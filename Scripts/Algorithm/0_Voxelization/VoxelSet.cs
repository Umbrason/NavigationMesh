using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace NavMeshGraph.Voxelization
{
    [Serializable]
    public class VoxelSet : ISerializationCallbackReceiver, IEnumerable, IEnumerable<Vector3Int>
    {
        private HashSet<Vector3Int> m_set;
        private HashSet<Vector3Int> Set => m_set ??= new();
        public int Count => Set.Count;
        public Vector3Int[] Values => m_set.ToArray();
        [SerializeField, HideInInspector] private Vector3Int[] serializationData;
        public void OnAfterDeserialize()
        {
            foreach (var key in serializationData) Set.Add(key);
            serializationData = new Vector3Int[0];
        }

        public void OnBeforeSerialize()
        {
            serializationData = Set.ToArray();
        }

        public VoxelSet RemoveKernel(Func<Vector3Int, HashSet<Vector3Int>, bool> function)
        {
            var workingSet = new HashSet<Vector3Int>(Set.Count);
            foreach (var key in Set)
            {
                if (!function.Invoke(key, Set)) workingSet.Add(key);
            }
            Set.Clear();
            foreach (var key in workingSet) Set.Add(key);
            return this;
        }

        public void Clear() => Set.Clear();

        public void Add(Vector3Int item) => Set.Add(item);
        public void Remove(Vector3Int item) => Set.Remove(item);
        public bool Contains(Vector3Int item) => Set.Contains(item);

        public VoxelSet Copy() => new() { m_set = new(this.m_set) };
        public IEnumerator<Vector3Int> GetEnumerator() => Set.GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => Set.GetEnumerator();
    }
}