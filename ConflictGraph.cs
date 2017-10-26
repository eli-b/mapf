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

        public ConflictGraph(int numOfAgents)
        {
            this.numOfEdges = 0;
            this.numOfNodes = 0;
            this.G = new bool[numOfAgents, numOfAgents];
            for(int i = 0; i< numOfAgents; i++)
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
            if(!G[agentAId, agentBId])
            {
                G[agentAId, agentBId] = true;
                G[agentBId, agentAId] = true;
                numOfEdges++;
            }
        }

        public int MaximumVertexCover(int prevMVC = -1)
        {
            if (numOfEdges < 2)
                return numOfEdges;

            // compute number of nodes that have edges
            for (int i = 0; i < this.G.GetLength(0); i++)
            {
                for(int j = 0; j < this.G.GetLength(1); j++)
                {
                    if(G[i,j])
                    {
                        this.numOfNodes++;
                        break;
                    }
                }
            }

            if (prevMVC < 0) // root node of CBS tree
                for (int i = 1; i < this.numOfNodes; i++)
                    if (KVertexCover(this, i))
                        return i;

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
        private bool KVertexCover(ConflictGraph CG, int k)
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
