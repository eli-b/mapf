using System;
using System.Collections;
using System.Collections.Generic;

namespace CPF_experiment
{
    /// <summary>
    /// This class forms a wrapper around m_Problem.GetSingleAgentShortestPath().
    /// It represents the single shortest path heuristic, precomputed for every agent.
    /// </summary>

    [Serializable]
    class SumIndividualCosts : PDB
    {
        /// <summary>
        /// Since this class simply refers via a table-lookup to the globally
        /// available m_Problem.GetSingleAgentShortestPath class, we incur no memory.
        /// </summary>
        /// <returns>0 by definition.</returns>
        public override UInt64 estimateSize()
        {
            return 0;
        }

        /// <summary>
        /// The building function for this class doesn't do anything because we
        /// are simply wrapping the functionality of the m_Problem.GetSingleAgentShortestPath
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
                int a = allAgents[i].Goal.x;
                int b = allAgents[i].Goal.y;
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
            var openList = new Queue<Tuple<Move, int>>();
            HashSet<int> closedList = new HashSet<int>();
            openList.Enqueue(new Tuple<Move, int>(new Move(X, Y, Move.Direction.NO_DIRECTION), 0));
            closedList.Add(X * size_X + Y);
            while (openList.Count > 0)
            {
                var tuple = openList.Dequeue();
                Move move = tuple.Item1;
                val = tuple.Item2;
                ans[move.x][move.y] = val;
                
                // Expand tile:
                foreach (Move nextMove in move.GetNextMoves(allowDiagonalMove))
                {
                    if (isValidTile(nextMove.x, nextMove.y) && !closedList.Contains(nextMove.x * size_X + nextMove.y))
                    {
                        openList.Enqueue(new Tuple<Move, int>(nextMove, val + 1));
                        closedList.Add(nextMove.x * size_X + nextMove.y);
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
            foreach (AgentState state in s.allAgentsState)
            {
                nHeuristic += (uint)this.m_Problem.GetSingleAgentShortestPath(state);
            }
            return nHeuristic;
        }

        public override string ToString()
        {
            return "SIC";
        }
    }
}
