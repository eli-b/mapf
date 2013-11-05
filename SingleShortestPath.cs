using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experement
{

    /// <summary>
    /// This class forms a wrapper around True_Path_Huristic, which represents
    /// the single shortest path heuristic, precomputed for every agent.
    /// </summary>

    [Serializable]
    class SingleShortestPath : PDB
    {

        /// <summary>
        /// Since this class simply refers via a table-lookup to the globally
        /// available True_Path_Huristic class, we incur no memory.
        /// </summary>
        /// <returns>0 by definition.</returns>

        public override UInt64 estimateSize()
        {
            return (0);
        }

        /// <summary>
        /// The building function for this class doesn't do anything because we
        /// are simply wrapping the functionality of the True_Path_Huristic
        /// class.
        /// </summary>

        public override void build()
        {
        }

        static int[][][] allTileAgentHuristics;
        static int size_X;
        static int size_Y;
        public static bool[][] grid;
        static bool allowDiagonalMove;

        public static void init(int i_size_X, int i_size_Y, Agent[] allAgents, bool diagonal, bool[][] i_grid)
        {
            size_X = i_size_X;
            size_Y = i_size_Y;
            grid = i_grid;
            allowDiagonalMove = diagonal;
            allTileAgentHuristics = new int[allAgents.Length][][];
            for (int i = 0; i < allAgents.Length; i++)
            {
                int a = allAgents[i].Goal_X;
                int b = allAgents[i].Goal_Y;
                allTileAgentHuristics[i] = setHuristicsForTile(a, b);
            }
        }
        public static void clear()
        {
            allTileAgentHuristics = null;
            grid = null;
        }
        private static int[][] setHuristicsForTile(int X, int Y)
        {
            int a;
            int b;
            int val;
            int[][] ans = new int[size_X][];
            for (int i = 0; i < size_X; i++)
            {
                ans[i] = new int[size_Y];
                for (int j = 0; j < size_Y; j++)
                {
                    ans[i][j] = -1;
                }
            }
            Queue openList = new Queue();
            HashSet<int> closedList = new HashSet<int>();
            openList.Enqueue(X);
            openList.Enqueue(Y);
            openList.Enqueue(0);
            closedList.Add(X * size_X + Y);
            while (openList.Count > 0)
            {
                a = (int)openList.Dequeue();
                b = (int)openList.Dequeue();
                val = (int)openList.Dequeue();
                ans[a][b] = val;
                if (isValidTile(a - 1, b) && !closedList.Contains((a - 1) * size_X + b))
                {
                    insertToOpenList(a - 1, b, val + 1, openList);
                    closedList.Add((a - 1) * size_X + b);
                }


                if (isValidTile(a, b + 1) && !closedList.Contains(a * size_X + b + 1))
                {
                    insertToOpenList(a, b + 1, val + 1, openList);
                    closedList.Add(a * size_X + b + 1);
                }

                if (isValidTile(a + 1, b) && !closedList.Contains((a + 1) * size_X + b))
                {
                    insertToOpenList(a + 1, b, val + 1, openList);
                    closedList.Add((a + 1) * size_X + b);
                }

                if (isValidTile(a, b - 1) && !closedList.Contains(a * size_X + b - 1))
                {
                    insertToOpenList(a, b - 1, val + 1, openList);
                    closedList.Add(a * size_X + b - 1);
                }

                if (allowDiagonalMove)
                {
                    if (isValidTile(a - 1, b + 1) && !closedList.Contains((a - 1) * size_X + b + 1))
                    {
                        insertToOpenList(a - 1, b + 1, val + 1, openList);
                        closedList.Add((a - 1) * size_X + b + 1);
                    }

                    if (isValidTile(a + 1, b + 1) && !closedList.Contains((a + 1) * size_X + b + 1))
                    {
                        insertToOpenList(a + 1, b + 1, val + 1, openList);
                        closedList.Add((a + 1) * size_X + b + 1);
                    }

                    if (isValidTile(a + 1, b - 1) && !closedList.Contains((a + 1) * size_X + b - 1))
                    {
                        insertToOpenList(a + 1, b - 1, val + 1, openList);
                        closedList.Add((a + 1) * size_X + b - 1);
                    }

                    if (isValidTile(a - 1, b - 1) && !closedList.Contains((a - 1) * size_X + b - 1))
                    {
                        insertToOpenList(a - 1, b - 1, val + 1, openList);
                        closedList.Add((a - 1) * size_X + b - 1);
                    }
                }
            }
            return ans;
        }

        private static void insertToOpenList(int X, int Y, int val, Queue OL)
        {
            OL.Enqueue(X);
            OL.Enqueue(Y);
            OL.Enqueue(val);
        }
        public static int getHuristic(int agent, int X, int Y)
        {
            return allTileAgentHuristics[agent][X][Y];
        }
        static public bool isValidTile(int X, int Y)
        {
            if (X < 0 || X >= size_X || Y < 0 || Y >= size_Y)
                return false;
            return !grid[X][Y];
        }
        public static int getGrisSize()
        {
            return size_X;
        }












        /// <summary>
        /// Returns the heuristic estimate.
        /// </summary>
        /// <param name="s">The current state.</param>
        /// <returns>The PDB entry for the given state.</returns>

        public override uint h(WorldState s)
        {
            uint nHeuristic = 0;
            foreach (var a in m_vAgents)
            {
                nHeuristic += (uint)this.m_Problem.GetSingleAgentShortestPath(s.allAgentsState[a].agent.agentNum,
                    s.allAgentsState[a].pos_X, s.allAgentsState[a].pos_Y);
            }
            return (nHeuristic);
        }
    }
}
