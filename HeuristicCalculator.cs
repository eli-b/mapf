using System.Collections.Generic;
using System.IO;

namespace CPF_experiment
{
    public interface HeuristicCalculator : IAccumulatingStatisticsCsvWriter
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
    }

    public interface LazyHeuristic : HeuristicCalculator
    {
        /// <summary>Returns the heuristic estimate.</summary>
        /// <param name="s">The current state.</param>
        /// <param name="target">The lowest target estimate to return, if possible.</param>
        uint h(WorldState s, int target);
    }
}
