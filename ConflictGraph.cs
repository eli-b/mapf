using System;
using System.Collections.Generic;

namespace mapf;

public class ConflictGraph
{
    private bool[,] _g;
    private int _numOfNodes;
    private int _numOfEdges;

    public enum MinVertexCover : sbyte
    {
        NOT_SET = -1
    }

    public ConflictGraph(int numOfAgents)
    {
        _numOfEdges = 0;
        _numOfNodes = 0;
        _g = new bool[numOfAgents, numOfAgents];
        for (int i = 0; i< numOfAgents; i++)  // FIXME: Not necessary.
        {
            for (int j = 0; j < numOfAgents; j++)
                _g[i, j] = false;
        }
    }

    public ConflictGraph(ConflictGraph other)
    {
        _numOfEdges = other._numOfEdges;
        _numOfNodes = other._numOfNodes;
        _g = new bool[other._g.GetLength(0), other._g.GetLength(1)];
        Array.Copy(other._g, _g, _g.Length);
    }

    public void Add(int agentAId, int agentBId)
    {
        if (!_g[agentAId, agentBId])
        {
            _g[agentAId, agentBId] = true;
            _g[agentBId, agentAId] = true;
            _numOfEdges++;
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="prevMVC"></param>
    /// <returns>The size of the 2-approximate minimum vertex cover</returns>
    public int ApproximateMinimumVertexCover(int prevMVC = (int)MinVertexCover.NOT_SET)
    {
        if (_numOfEdges < 2)
            return _numOfEdges;

        HashSet<int> approximateMinCover = [];
        for (int i = 0; i < _g.GetLength(0) - 1; i++)
        {
            if (approximateMinCover.Contains(i)) // Node i already in the cover - all its edges are already covered.
                continue;

            for (int j = i + 1; j < _g.GetLength(1); j++)
            {
                if (_g[i, j])
                {
                    if (approximateMinCover.Contains(j) == false)
                    {
                        approximateMinCover.Add(i);
                        approximateMinCover.Add(j);
                        break; // All of node i's edges are now covered.
                    }
                }
            }
        }

        return approximateMinCover.Count;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="prevMVC"></param>
    /// <returns>The size of the minimum vertex cover</returns>
    public int MinimumVertexCover(int prevMVC = (int) MinVertexCover.NOT_SET)
    {
        if (_numOfEdges < 2)
            return _numOfEdges;

        // compute number of nodes that have edges
        _numOfNodes = 0;
        for (int i = 0; i < _g.GetLength(0); i++)
        {
            for (int j = 0; j < _g.GetLength(1); j++)
            {
                if (_g[i,j])
                {
                    _numOfNodes++;
                    break;
                }
            }
        }

        if (prevMVC == (int) MinVertexCover.NOT_SET) // root node of CBS tree, or any node on whose parent we decided not to compute this heuristic
            for (int i = 1; i < _numOfNodes; i++)
                if (KVertexCover(this, i))
                    return i;

        // Either a single (meta-)agent's path was replanned under a new constraint,
        // or two (meta-)agents were merged and their combined path is planned under infinite
        // implicit constraints not to collide between themselves.
        // In the first case, all edges between the agent and other agents in the
        // cardinal-conflict graph could appear, disappear, or stay the same. So the old min
        // vertex cover either stays the same, or now has to include this agent, or now doesn't
        // need to include this agent.
        // In the second case, edges between the agents are now deleted, edges between each
        // one of them and other agents are now either replaced with edges to the new combined
        // agent or deleted, and new edges between the combined agent and other agents might need
        // to be deleted. So we either subtract 2 nodes from the old mvc, or one node, or it
        // stays the same, or we now have to add the combined node to the mvc.
        // TODO: add support for merge actions that don't cause a restart by checking the prevMVC-2 option if the last
        //       action was a merge.
        if (KVertexCover(this, prevMVC - 1))
            return prevMVC - 1;
        else if (KVertexCover(this, prevMVC))
            return prevMVC;
        else
            return prevMVC + 1;
    }

    /// <summary>
    /// Return whether there exists a k-vertex (at most) cover solution (an NP-Complete question).
    /// This algorithm theoretically runs in O(2^(k-1)*2*n), or O(2^k*n). It was described in Parameterized
    /// Computational Feasibility (Downey and Fellows, 1995).
    /// The algorithm with the best asymptotic dependence on k was desribed in Improved
    /// Parameterized Upper Bounds for Vertex Cover (Cheng, Kanj and Xia 2006) and has a runtime of
    /// O(1.2738^k + kn). Even that algorithm can only find a vertex cover of size up to ~190 in
    /// reasonable time.
    /// </summary>
    private static bool KVertexCover(ConflictGraph CG, int k, int lastEdgeX = 0)
    {
        if (CG._numOfEdges == 0)
            return true;
        else if (CG._numOfEdges > k * CG._numOfNodes - k) // |E| > K*(|V|-1), there are more edges
            // to cover than the maximum number of edges that could be covered with K vertices
            // (if every vertex chosen for the cover is connected to all other vertices in the graph),
            // so a K vertex cover is impossible
            return false;

        // Choose an edge (u,v) - (this step is actually O(n^2) but the algorithm assumes is done in constant time)
        // TODO: Measure if this part is significant
        Span<int> edge = stackalloc int[2];
        bool found = false;
        for (int i = lastEdgeX; i < CG._g.GetLength(0) - 1 && !found; i++)
        {
            for (int j = i + 1; j < CG._g.GetLength(1) && !found; j++)
            {
                if (CG._g[i, j])
                {
                    edge[0] = i;
                    edge[1] = j;
                    found = true;
                }
            }
        }

        // Recurse over KVertexCover(G-{u}, k-1) and KVertexCover(G-{v}, k-1).
        // If any are true, return true. Else return false.
        for (int i = 0; i < 2; i++)
        {
            ConflictGraph CG_copy = new(CG);  // TODO: This part also costs n^2
            for (int j = 0; j < CG._g.GetLength(0); j++)
            {
                if (CG_copy._g[edge[i], j])
                {
                    CG_copy._g[edge[i], j] = false;
                    CG_copy._g[j, edge[i]] = false;
                    CG_copy._numOfEdges--;
                }
            }
            CG_copy._numOfNodes--;
            if (KVertexCover(CG_copy, k - 1, edge[0]))
                return true;
        }
        return false;
    }
}
