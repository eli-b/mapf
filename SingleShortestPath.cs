using System;
using System.Collections;
using System.Collections.Generic;

namespace CPF_experiment
{

    /// <summary>
    /// This class forms a wrapper around True_Path_Heuristic, which represents
    /// the single shortest path heuristic, precomputed for every agent.
    /// </summary>

    [Serializable]
    class SingleShortestPath : PDB
    {
        /// <summary>
        /// Since this class simply refers via a table-lookup to the globally
        /// available True_Path_Heuristic class, we incur no memory.
        /// </summary>
        /// <returns>0 by definition.</returns>
        public override UInt64 estimateSize()
        {
            return 0;
        }

        /// <summary>
        /// The building function for this class doesn't do anything because we
        /// are simply wrapping the functionality of the True_Path_Heuristic
        /// class.
        /// </summary>
        public override void build() {}

        static int[][][] allTileAgentHeuristics;
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
            allTileAgentHeuristics = new int[allAgents.Length][][];
            for (int i = 0; i < allAgents.Length; i++)
            {
                int a = allAgents[i].Goal_X;
                int b = allAgents[i].Goal_Y;
                allTileAgentHeuristics[i] = setHeuristicsForTile(a, b);
            }
        }
        public static void clear()
        {
            allTileAgentHeuristics = null;
            grid = null;
        }
        private static int[][] setHeuristicsForTile(int X, int Y)
        {
            int a;
            int b;
            int val;
            int last_direction;
            if (allowDiagonalMove == false)
                last_direction = Move.LAST_NON_DIAG_MOVE;
            else
                last_direction = Move.LAST_REAL_MOVE;
            int[][] ans = new int[size_X][];
            for (int i = 0; i < size_X; i++)
            {
                ans[i] = new int[size_Y];
                for (int j = 0; j < size_Y; j++)
                {
                    ans[i][j] = -1;
                }
            }
            var openList = new Queue<Tuple<int,int,int>>();
            HashSet<int> closedList = new HashSet<int>();
            openList.Enqueue(new Tuple<int,int,int>(X, Y, 0));
            closedList.Add(X * size_X + Y);
            while (openList.Count > 0)
            {
                var tuple = openList.Dequeue();
                a = tuple.Item1;
                b = tuple.Item2;
                val = tuple.Item3;
                ans[a][b] = val;
                
                // Expand tile:
                for (int direction = Move.FIRST_NON_WAIT; direction <= last_direction; ++direction)
                // Assuming directions that aren't Wait are consecutive
                {
                    int new_a = a + Move.directionToDeltas[direction,0];
                    int new_b = b + Move.directionToDeltas[direction, 1];
                    if (isValidTile(new_a, new_b) && !closedList.Contains(new_a * size_X + new_b))
                    {
                        openList.Enqueue(new Tuple<int, int, int>(new_a, new_b, val + 1));
                        closedList.Add(new_a * size_X + new_b);
                    }
                }
            }
            return ans;
        }

        public static int getHeuristic(int agent, int X, int Y)
        {
            return allTileAgentHeuristics[agent][X][Y];
        }

        static public bool isValidTile(int X, int Y)
        {
            if (X < 0 || X >= size_X || Y < 0 || Y >= size_Y)
                return false;
            return !grid[X][Y];
        }

        public static int getGridSize()
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
            return nHeuristic;
        }
    }
}
