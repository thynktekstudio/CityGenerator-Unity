using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// High level settlement planner that interprets the baked NavMesh topology and
/// places medieval style structures such as keeps, walls and houses.  The
/// generator analyses the NavMesh triangulation to understand boundaries and the
/// walkable interior so that spawned buildings look intentional and believable
/// within the generated environment.
///
/// Attach the component to an empty GameObject in the sample scene, assign any
/// desired prefabs and call <see cref="RebuildSettlement"/> (or enable
/// <see cref="autoBuildOnStart"/>) after a NavMesh has been baked.
/// </summary>
[ExecuteAlways]
public class MedievalSettlementAI : MonoBehaviour
{
    [Header("Automatic Setup")]
    [Tooltip("Automatically (re)builds the settlement during Start.")]
    public bool autoBuildOnStart = true;

    [Tooltip("Seed used when choosing locations and random rotations.")]
    public int randomSeed = 1337;

    [Header("Prefabs")]
    [Tooltip("Large, central structure acting as the heart of the settlement.")]
    public GameObject keepPrefab;

    [Tooltip("Repeated element used to stitch the outer wall together.")]
    public GameObject wallSegmentPrefab;

    [Tooltip("Tower or bastion to add along the wall perimeter.")]
    public GameObject wallTowerPrefab;

    [Tooltip("Optional prefabs for civilian houses. One is chosen per house.")]
    public List<GameObject> housePrefabs = new List<GameObject>();

    [Header("Keep & Courtyard")]
    [Tooltip("Minimum clearance radius around the keep. Houses are kept outside this circle.")]
    public float keepClearRadius = 15f;

    [Tooltip("Scale applied to primitive keep when no prefab is provided.")]
    public Vector3 fallbackKeepScale = new Vector3(12f, 18f, 12f);

    [Header("Walls")]
    [Tooltip("Spacing between wall segments along the perimeter.")]
    public float wallSegmentSpacing = 8f;

    [Tooltip("Approximate spacing between towers placed along the wall.")]
    public float towerInterval = 32f;

    [Tooltip("Vertical scale used when generating primitive wall segments.")]
    public Vector3 fallbackWallScale = new Vector3(6f, 6f, 2f);

    [Header("Buildings")]
    [Tooltip("Maximum number of houses to attempt to place inside the walls.")]
    public int maxHouses = 140;

    [Tooltip("Minimum distance kept between spawned houses.")]
    public float houseSpacing = 10f;

    [Tooltip("Random planar offset applied to each house placement.")]
    public float houseJitter = 2.5f;

    [Tooltip("Random rotation offset (degrees) applied to each house.")]
    public float houseRotationJitter = 20f;

    [Tooltip("Height applied to generated primitive houses when no prefab exists.")]
    public Vector3 fallbackHouseScale = new Vector3(6f, 4f, 6f);

    [Header("Debugging")]
    [Tooltip("Draws gizmos representing analysed NavMesh features.")]
    public bool drawDebugGizmos = true;

    private readonly List<GameObject> _spawnedObjects = new List<GameObject>();
    private readonly List<Vector3> _housePositions = new List<Vector3>();
    private NavMeshTopologyData _cachedTopology;
    private System.Random _random;

    private void Start()
    {
        if (autoBuildOnStart)
        {
            RebuildSettlement();
        }
    }

    private void OnDisable()
    {
        if (!Application.isPlaying)
        {
            // In edit mode we proactively cleanup spawned objects so that
            // reloading the scene does not accumulate duplicates.
            ClearSpawnedObjects();
        }
    }

    /// <summary>
    /// Clears any previously instantiated structures and rebuilds the
    /// settlement layout based on the latest NavMesh triangulation.
    /// </summary>
    [ContextMenu("Rebuild Settlement")]
    public void RebuildSettlement()
    {
        ClearSpawnedObjects();
        _random = new System.Random(randomSeed);

        var triangulation = NavMesh.CalculateTriangulation();
        if (triangulation.vertices == null || triangulation.vertices.Length == 0)
        {
            Debug.LogWarning("MedievalSettlementAI: Unable to rebuild settlement – no baked NavMesh found.");
            _cachedTopology = null;
            return;
        }

        _cachedTopology = AnalyseTopology(triangulation);
        PlaceKeep(_cachedTopology.NavMeshCentroid);
        BuildWalls(_cachedTopology.BoundaryLoops);
        PopulateInterior(_cachedTopology.InteriorSamples, _cachedTopology.NavMeshCentroid);
    }

    private void ClearSpawnedObjects()
    {
        for (int i = _spawnedObjects.Count - 1; i >= 0; i--)
        {
            var obj = _spawnedObjects[i];
            if (obj != null)
            {
                if (Application.isPlaying)
                {
                    Destroy(obj);
                }
                else
                {
                    DestroyImmediate(obj);
                }
            }
        }

        _spawnedObjects.Clear();
        _housePositions.Clear();
    }

    private NavMeshTopologyData AnalyseTopology(NavMeshTriangulation triangulation)
    {
        var data = new NavMeshTopologyData();
        if (triangulation.vertices == null || triangulation.indices == null)
        {
            return data;
        }

        Vector3 centroid = Vector3.zero;
        for (int i = 0; i < triangulation.vertices.Length; i++)
        {
            centroid += triangulation.vertices[i];
        }

        if (triangulation.vertices.Length > 0)
        {
            centroid /= triangulation.vertices.Length;
        }

        data.NavMeshCentroid = centroid;

        // Prepare reusable dictionary for edges shared by triangles.
        var edgeUsage = new Dictionary<EdgeKey, EdgeRecord>(triangulation.indices.Length);
        var interiorSamples = new List<Vector3>(triangulation.indices.Length / 3);

        for (int i = 0; i < triangulation.indices.Length; i += 3)
        {
            int ia = triangulation.indices[i];
            int ib = triangulation.indices[i + 1];
            int ic = triangulation.indices[i + 2];

            var a = triangulation.vertices[ia];
            var b = triangulation.vertices[ib];
            var c = triangulation.vertices[ic];

            interiorSamples.Add((a + b + c) / 3f);

            AccumulateEdge(edgeUsage, ia, ib);
            AccumulateEdge(edgeUsage, ib, ic);
            AccumulateEdge(edgeUsage, ic, ia);
        }

        data.InteriorSamples = interiorSamples;
        data.BoundaryLoops = BuildBoundaryLoops(edgeUsage, triangulation.vertices);
        return data;
    }

    private void AccumulateEdge(Dictionary<EdgeKey, EdgeRecord> edgeUsage, int start, int end)
    {
        var key = new EdgeKey(start, end);
        if (edgeUsage.TryGetValue(key, out var record))
        {
            record.Usage++;
        }
        else
        {
            edgeUsage.Add(key, new EdgeRecord
            {
                Usage = 1,
                Start = start,
                End = end
            });
        }
    }

    private List<List<Vector3>> BuildBoundaryLoops(Dictionary<EdgeKey, EdgeRecord> edgeUsage, Vector3[] vertices)
    {
        var boundaryEdges = new List<EdgeRecord>();
        foreach (var kvp in edgeUsage)
        {
            if (kvp.Value.Usage == 1)
            {
                boundaryEdges.Add(kvp.Value);
            }
        }

        var remainingEdges = new HashSet<EdgeKey>();
        var adjacency = new Dictionary<int, List<int>>();
        foreach (var edge in boundaryEdges)
        {
            var key = new EdgeKey(edge.Start, edge.End);
            remainingEdges.Add(key);

            if (!adjacency.TryGetValue(edge.Start, out var listA))
            {
                listA = new List<int>();
                adjacency.Add(edge.Start, listA);
            }
            if (!adjacency.TryGetValue(edge.End, out var listB))
            {
                listB = new List<int>();
                adjacency.Add(edge.End, listB);
            }

            if (!listA.Contains(edge.End))
            {
                listA.Add(edge.End);
            }
            if (!listB.Contains(edge.Start))
            {
                listB.Add(edge.Start);
            }
        }

        var loops = new List<List<Vector3>>();
        var visitedVertices = new HashSet<int>();

        while (remainingEdges.Count > 0)
        {
            EdgeKey startKey = default;
            foreach (var edge in remainingEdges)
            {
                startKey = edge;
                break;
            }

            var loopIndices = new List<int>();
            int startIndex = startKey.V0;
            int current = startIndex;
            int previous = -1;

            loopIndices.Add(startIndex);
            visitedVertices.Add(startIndex);

            bool closed = false;
            int guard = 0;
            while (guard++ < 4096)
            {
                if (!adjacency.TryGetValue(current, out var neighbours) || neighbours.Count == 0)
                {
                    break;
                }

                int next = -1;
                for (int i = 0; i < neighbours.Count; i++)
                {
                    int candidate = neighbours[i];
                    if (candidate == previous)
                    {
                        continue;
                    }

                    var candidateKey = new EdgeKey(current, candidate);
                    if (remainingEdges.Contains(candidateKey))
                    {
                        next = candidate;
                        remainingEdges.Remove(candidateKey);
                        break;
                    }
                }

                if (next == -1)
                {
                    break;
                }

                if (next == startIndex)
                {
                    closed = true;
                    loopIndices.Add(next);
                    break;
                }

                loopIndices.Add(next);
                visitedVertices.Add(next);
                previous = current;
                current = next;
            }

            if (closed && loopIndices.Count >= 4)
            {
                var loop = new List<Vector3>(loopIndices.Count);
                for (int i = 0; i < loopIndices.Count; i++)
                {
                    loop.Add(vertices[loopIndices[i]]);
                }
                loops.Add(loop);
            }
        }

        return loops;
    }

    private void PlaceKeep(Vector3 position)
    {
        GameObject keepInstance;
        if (keepPrefab != null)
        {
            keepInstance = Instantiate(keepPrefab, position, Quaternion.identity, transform);
        }
        else
        {
            keepInstance = GameObject.CreatePrimitive(PrimitiveType.Cube);
            keepInstance.transform.SetParent(transform, false);
            keepInstance.transform.position = position;
            keepInstance.transform.localScale = fallbackKeepScale;
        }

        keepInstance.name = "Keep";
        RegisterSpawn(keepInstance, position, false);
    }

    private void BuildWalls(List<List<Vector3>> loops)
    {
        if (loops == null || loops.Count == 0)
        {
            return;
        }

        foreach (var loop in loops)
        {
            if (loop.Count < 2)
            {
                continue;
            }

            float distanceSinceTower = 0f;
            for (int i = 0; i < loop.Count - 1; i++)
            {
                var start = loop[i];
                var end = loop[i + 1];
                Vector3 segment = end - start;
                float segmentLength = segment.magnitude;
                if (segmentLength < Mathf.Epsilon)
                {
                    continue;
                }

                Vector3 direction = segment / segmentLength;
                int segmentCount = Mathf.Max(1, Mathf.RoundToInt(segmentLength / Mathf.Max(1f, wallSegmentSpacing)));
                float step = segmentLength / segmentCount;

                for (int s = 0; s < segmentCount; s++)
                {
                    float t = (s + 0.5f) / segmentCount;
                    Vector3 pos = Vector3.Lerp(start, end, t);
                    PlaceWallSegment(pos, direction);

                    distanceSinceTower += step;
                    if (distanceSinceTower >= Mathf.Max(wallSegmentSpacing, towerInterval))
                    {
                        PlaceTower(pos, direction);
                        distanceSinceTower = 0f;
                    }
                }
            }
        }
    }

    private void PlaceWallSegment(Vector3 position, Vector3 forward)
    {
        GameObject segmentInstance;
        if (wallSegmentPrefab != null)
        {
            segmentInstance = Instantiate(wallSegmentPrefab, position, Quaternion.LookRotation(forward, Vector3.up), transform);
        }
        else
        {
            segmentInstance = GameObject.CreatePrimitive(PrimitiveType.Cube);
            segmentInstance.transform.SetParent(transform, false);
            segmentInstance.transform.position = position;
            segmentInstance.transform.rotation = Quaternion.LookRotation(forward, Vector3.up);
            segmentInstance.transform.localScale = fallbackWallScale;
        }

        segmentInstance.name = "Wall Segment";
        RegisterSpawn(segmentInstance, position, false);
    }

    private void PlaceTower(Vector3 position, Vector3 forward)
    {
        GameObject towerInstance;
        if (wallTowerPrefab != null)
        {
            towerInstance = Instantiate(wallTowerPrefab, position, Quaternion.LookRotation(forward, Vector3.up), transform);
        }
        else
        {
            towerInstance = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            towerInstance.transform.SetParent(transform, false);
            towerInstance.transform.position = position;
            towerInstance.transform.localScale = new Vector3(fallbackWallScale.x * 0.75f, fallbackWallScale.y * 1.5f, fallbackWallScale.x * 0.75f);
        }

        towerInstance.name = "Wall Tower";
        RegisterSpawn(towerInstance, position, false);
    }

    private void PopulateInterior(List<Vector3> samples, Vector3 keepPosition)
    {
        if (samples == null || samples.Count == 0)
        {
            return;
        }

        var shuffledSamples = samples.OrderBy(_ => _random.Next()).ToList();
        int placed = 0;

        foreach (var sample in shuffledSamples)
        {
            if (placed >= maxHouses)
            {
                break;
            }

            Vector3 planar = sample;
            planar.y = keepPosition.y;
            float distanceToKeep = Vector3.Distance(new Vector3(planar.x, 0f, planar.z), new Vector3(keepPosition.x, 0f, keepPosition.z));
            if (distanceToKeep < keepClearRadius)
            {
                continue;
            }

            var jitter = new Vector2((float)(_random.NextDouble() * 2.0 - 1.0) * houseJitter,
                                     (float)(_random.NextDouble() * 2.0 - 1.0) * houseJitter);
            planar.x += jitter.x;
            planar.z += jitter.y;

            float jitteredDistance = Vector3.Distance(new Vector3(planar.x, 0f, planar.z), new Vector3(keepPosition.x, 0f, keepPosition.z));
            if (jitteredDistance < keepClearRadius)
            {
                continue;
            }

            if (!IsPositionFarEnough(planar))
            {
                continue;
            }

            if (!NavMesh.SamplePosition(planar, out var hit, 5f, NavMesh.AllAreas))
            {
                continue;
            }

            if (!IsPositionFarEnough(hit.position))
            {
                continue;
            }

            Vector3 forward = (keepPosition - hit.position).sqrMagnitude > 0.1f
                ? (keepPosition - hit.position).normalized
                : Vector3.forward;
            float rotationNoise = ((float)_random.NextDouble() * 2f - 1f) * houseRotationJitter;
            Quaternion rotation = Quaternion.AngleAxis(rotationNoise, Vector3.up) * Quaternion.LookRotation(-forward, Vector3.up);

            PlaceHouse(hit.position, rotation);
            placed++;
        }
    }

    private bool IsPositionFarEnough(Vector3 candidate)
    {
        Vector2 candidate2D = new Vector2(candidate.x, candidate.z);
        for (int i = 0; i < _housePositions.Count; i++)
        {
            var existing = _housePositions[i];
            Vector2 existing2D = new Vector2(existing.x, existing.z);
            if (Vector2.Distance(candidate2D, existing2D) < houseSpacing)
            {
                return false;
            }
        }

        return true;
    }

    private void PlaceHouse(Vector3 position, Quaternion rotation)
    {
        GameObject instance = null;
        if (housePrefabs != null && housePrefabs.Count > 0)
        {
            var prefab = housePrefabs[_random.Next(housePrefabs.Count)];
            if (prefab != null)
            {
                instance = Instantiate(prefab, position, rotation, transform);
            }
        }

        if (instance == null)
        {
            instance = GameObject.CreatePrimitive(PrimitiveType.Cube);
            instance.transform.SetParent(transform, false);
            instance.transform.position = position;
            instance.transform.rotation = rotation;
            instance.transform.localScale = fallbackHouseScale;
        }

        instance.name = "House";
        RegisterSpawn(instance, position, true);
    }

    private void RegisterSpawn(GameObject go, Vector3 planarPosition, bool trackSpacing)
    {
        _spawnedObjects.Add(go);
        if (trackSpacing)
        {
            _housePositions.Add(planarPosition);
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawDebugGizmos || _cachedTopology == null)
        {
            return;
        }

        Gizmos.color = Color.yellow;
        if (_cachedTopology.BoundaryLoops != null)
        {
            foreach (var loop in _cachedTopology.BoundaryLoops)
            {
                for (int i = 0; i < loop.Count - 1; i++)
                {
                    Gizmos.DrawLine(loop[i], loop[i + 1]);
                }
            }
        }

        Gizmos.color = Color.cyan;
        if (_cachedTopology.InteriorSamples != null)
        {
            foreach (var sample in _cachedTopology.InteriorSamples)
            {
                Gizmos.DrawSphere(sample, 0.5f);
            }
        }

        Gizmos.color = Color.magenta;
        Gizmos.DrawWireSphere(_cachedTopology.NavMeshCentroid, keepClearRadius);
    }

    #region Helper types

    private class NavMeshTopologyData
    {
        public Vector3 NavMeshCentroid;
        public List<List<Vector3>> BoundaryLoops;
        public List<Vector3> InteriorSamples;
    }

    private struct EdgeKey : IEquatable<EdgeKey>
    {
        public readonly int V0;
        public readonly int V1;

        public EdgeKey(int a, int b)
        {
            if (a < b)
            {
                V0 = a;
                V1 = b;
            }
            else
            {
                V0 = b;
                V1 = a;
            }
        }

        public bool Equals(EdgeKey other)
        {
            return V0 == other.V0 && V1 == other.V1;
        }

        public override bool Equals(object obj)
        {
            return obj is EdgeKey other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return (V0 * 397) ^ V1;
            }
        }
    }

    private class EdgeRecord
    {
        public int Usage;
        public int Start;
        public int End;
    }

    #endregion
}
