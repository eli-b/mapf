﻿using System;
using System.IO;
using System.Linq;

namespace mapf
{
    /// <summary>
    /// This class forms a wrapper around problem.GetSingleAgentOptimalCost().
    /// It represents the single shortest path heuristic, precomputed for every agent.
    /// </summary>
    [Serializable]
    class SumIndividualCosts : PDB
    {
        /// <summary>
        /// Since this class simply refers via a table-lookup to the globally
        /// available problem.GetSingleAgentOptimalCost class, we incur no memory.
        /// </summary>
        /// <returns>0 by definition.</returns>
        public override UInt64 estimateSize()
        {
            return 0;
        }

        /// <summary>
        /// The building function for this class doesn't do anything because we
        /// are simply wrapping the functionality of the problem.GetSingleAgentOptimalCost
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
                nHeuristic += (uint)this.problem.GetSingleAgentOptimalCost(state);
            }
            return nHeuristic;
        }

        public static uint h(WorldState s, ProblemInstance instance)
        {
            return (uint) s.allAgentsState.Sum(state => instance.GetSingleAgentOptimalCost(state));
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

    /// <summary>
    /// This class forms a wrapper around problem.GetSingleAgentOptimalCost().
    /// It represents the single shortest path heuristic for makespan, precomputed for every agent.
    /// </summary>
    [Serializable]
    class MaxIndividualCosts : PDB
    {
        /// <summary>
        /// Since this class simply refers via a table-lookup to the globally
        /// available problem.GetSingleAgentOptimalCost class, we incur no memory.
        /// </summary>
        /// <returns>0 by definition.</returns>
        public override UInt64 estimateSize()
        {
            return 0;
        }

        /// <summary>
        /// The building function for this class doesn't do anything because we
        /// are simply wrapping the functionality of the problem.GetSingleAgentOptimalCost
        /// class.
        /// </summary>
        public override void build() { }

        /// <summary>
        /// Returns the heuristic estimate.
        /// </summary>
        /// <param name="s">The current state.</param>
        /// <returns>The PDB entry for the given state.</returns>
        public override uint h(WorldState s)
        {
            uint maxHeuristic = 0;
            foreach (AgentState state in s.allAgentsState)
            {
                uint heuristic = (uint)this.problem.GetSingleAgentOptimalCost(state);
                if (heuristic > maxHeuristic)
                {
                    maxHeuristic = heuristic;
                }
            }
            return maxHeuristic;
        }

        public static uint h(WorldState s, ProblemInstance instance)
        {
            return (uint)s.allAgentsState.Max(state => instance.GetSingleAgentOptimalCost(state));
        }

        public override string ToString()
        {
            return "MIC";
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
