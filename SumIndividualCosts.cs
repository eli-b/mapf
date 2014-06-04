using System;
using System.IO;
using System.Linq;

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

        public static uint h(WorldState s, ProblemInstance instance)
        {
            return (uint) s.allAgentsState.Sum(state => instance.GetSingleAgentShortestPath(state));
        }

        public override string ToString()
        {
            return "SIC";
        }

        /// <summary>
        /// Prints header of statistics of a single run to the given output. 
        /// </summary>
        public override void OutputStatisticsHeader(TextWriter output) { }

        /// <summary>
        /// Prints statistics of a single run to the given output.
        /// </summary>
        public override void OutputStatistics(TextWriter output) { }

        public override int NumStatsColumns
        {
            get
            {
                return 0;
            }
        }

        /// <summary>
        /// Clears statistics.
        /// </summary>
        public override void ClearStatistics() { }

        public override void ClearAccumulatedStatistics() { }
        public override void AccumulateStatistics() { }
        public override void OutputAccumulatedStatistics(TextWriter output) { }
    }
}
