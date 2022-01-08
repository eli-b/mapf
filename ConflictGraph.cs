using System;
using System.Collections.Generic;

namespace mapf;

public class ConflictGraph
{
    public bool[,] G;
    public int numOfNodes;
    public int numOfEdges;

    public enum MinVertexCover : sbyte
    {
        NOT_SET = -1
    }

    public ConflictGraph(int numOfAgents)
    {
        this.numOfEdges = 0;
        this.numOfNodes = 0;
        this.G = new bool[numOfAgents, numOfAgents];
        for (int i = 0; i< numOfAgents; i++)  // FIXME: Not necessary.
        {
            for (int j = 0; j < numOfAgents; j++)
                G[i, j] = false;
        }
    }
    public ConflictGraph(ConflictGraph other)
    {
        this.numOfEdges = other.numOfEdges;
        this.numOfNodes = other.numOfNodes;
        this.G = new bool[other.G.GetLength(0), other.G.GetLength(1)];
        Array.Copy(other.G, G, G.Length);
    }

    public void Add(int agentAId, int agentBId)
    {
        if (!G[agentAId, agentBId])
        {
            G[agentAId, agentBId] = true;
            G[agentBId, agentAId] = true;
            numOfEdges++;
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="prevMVC"></param>
    /// <returns>The size of the 2-approximate minimum vertex cover</returns>
    public int ApproximateMinimumVertexCover(int prevMVC = (int)MinVertexCover.NOT_SET)
    {
        if (this.numOfEdges < 2)
            return this.numOfEdges;

        var approximateMinCover = new HashSet<int>();
        for (int i = 0; i < this.G.GetLength(0) - 1; i++)
        {
            if (approximateMinCover.Contains(i)) // Node i already in the cover - all its edges are already covered.
                continue;

            for (int j = i + 1; j < this.G.GetLength(1); j++)
            {
                if (G[i, j])
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
        if (this.numOfEdges < 2)
            return this.numOfEdges;

        // compute number of nodes that have edges
        this.numOfNodes = 0;
        for (int i = 0; i < this.G.GetLength(0); i++)
        {
            for (int j = 0; j < this.G.GetLength(1); j++)
            {
                if (G[i,j])
                {
                    this.numOfNodes++;
                    break;
                }
            }
        }

        if (prevMVC == (int) MinVertexCover.NOT_SET) // root node of CBS tree, or any node on whose parent we decided not to compute this heuristic
            for (int i = 1; i < this.numOfNodes; i++)
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
        // TODO: add support for merge actions by checking the prevMVC-2 option if the last
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
        if (CG.numOfEdges == 0)
            return true;
        else if (CG.numOfEdges > k * CG.numOfNodes - k) // |E| > K*(|V|-1), there are more edges
            // to cover than the maximum number of edges that could be covered with K vertices
            // (if every vertex chosen for the cover is connected to all other vertices in the graph),
            // so a K vertex cover is impossible
            return false;

        // Choose an edge (u,v) - (this step is actually O(n^2) but the algorithm assumes is done in constant time)
        // TODO: Measure if this part is significant
        int[] edge = new int[2];
        bool found = false;
        for (int i = lastEdgeX; i < CG.G.GetLength(0) - 1 && !found; i++)
        {
            for (int j = i + 1; j < CG.G.GetLength(1) && !found; j++)
            {
                if (CG.G[i, j])
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
            ConflictGraph CG_copy = new ConflictGraph(CG);  // TODO: This part also costs n^2
            for (int j = 0; j < CG.G.GetLength(0); j++)
            {
                if (CG_copy.G[edge[i], j])
                {
                    CG_copy.G[edge[i], j] = false;
                    CG_copy.G[j, edge[i]] = false;
                    CG_copy.numOfEdges--;
                }
            }
            CG_copy.numOfNodes--;
            if (KVertexCover(CG_copy, k - 1, edge[0]))
                return true;
        }
        return false;
        }
}
