using System;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
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
            for (int i = 0; i< numOfAgents; i++)
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

        public int MinimumVertexCover(int prevMVC = (int) MinVertexCover.NOT_SET)
        {
            if (numOfEdges < 2)
                return numOfEdges;

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

            if (prevMVC == (int) MinVertexCover.NOT_SET) // root node of CBS tree
                for (int i = 1; i < this.numOfNodes; i++)
                    if (KVertexCover(this, i))
                        return i;

            // Eiter a single (meta-)agent's path was replanned under a new constraint,
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
        /// Whether there exists a k-vertex cover solution
        /// </summary>
        private static bool KVertexCover(ConflictGraph CG, int k)
        {
            if (CG.numOfEdges == 0)
                return true;
            else if (CG.numOfEdges > k * CG.numOfNodes - k) // not sure
                return false;

            int[] node = new int[2];
            bool flag = true;
            for (int i = 0; i < CG.G.GetLength(0) - 1 && flag; i++) // to find an edge
            {
                for (int j = i + 1; j < CG.G.GetLength(1) && flag; j++)
                {
                    if (CG.G[i, j])
                    {
                        node[0] = i;
                        node[1] = j;
                        flag = false;
                    }
                }
            }
            for (int i = 0; i < 2; i++)
            {
                ConflictGraph CG_copy = new ConflictGraph(CG);
                for (int j = 0; j < CG.G.GetLength(0); j++)
                {
                    if (CG_copy.G[node[i], j])
                    {
                        CG_copy.G[node[i], j] = false;
                        CG_copy.G[j, node[i]] = false;
                        CG_copy.numOfEdges--;
                    }
                }
                CG_copy.numOfNodes--;
                if (KVertexCover(CG_copy, k - 1))
                    return true;
            }
            return false;
         }
      
    }
}
