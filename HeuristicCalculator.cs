using System.Collections.Generic;
using System.IO;

namespace CPF_experiment
{
    public interface HeuristicCalculator
    {
        /// <summary>Returns the heuristic estimate.</summary>
        /// <param name="s">The current state.</param>
        uint h(WorldState s);

        /// <summary>
        /// Initializes the pattern database by storing references to the
        /// problem instance and also the subset of agents that the pattern
        /// database pertains to.
        /// </summary>
        /// <param name="pi">The problem instance.</param>
        /// <param name="vAgents">The agents that the pattern database should keep track of.</param>
        void init(ProblemInstance pi, List<uint> vAgents);

        /// <summary>
        /// Prints header of statistics of a single run to the given output. 
        /// </summary>
        void OutputStatisticsHeader(TextWriter output);

        /// <summary>
        /// Prints statistics of a single run to the given output.
        /// </summary>
        void OutputStatistics(TextWriter output);

        /// <summary>
        /// To fill out them out when an algorithm isn't run
        /// </summary>
        int NumStatsColumns { get; }

        /// <summary>
        /// Clears statistics.
        /// </summary>
        void ClearStatistics();
    }

    public interface LazyHeuristic : HeuristicCalculator
    {
        /// <summary>Returns the heuristic estimate.</summary>
        /// <param name="s">The current state.</param>
        /// <param name="target">The lowest target estimate to return, if possible.</param>
        uint h(WorldState s, int target);
    }
}
