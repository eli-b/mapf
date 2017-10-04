using System.Collections.Generic;
using System.IO;

namespace CPF_experiment
{
    public interface IHeuristicCalculator : IAccumulatingStatisticsCsvWriter
    {
        /// <summary>Returns the heuristic estimate.</summary>
        /// <param name="s">The current state.</param>
        uint h(WorldState s);

        /// <summary>
        /// Initializes the pattern databasea by storing references to the
        /// problem instance and also the subset of agents that the pattern
        /// database pertains to.
        /// </summary>
        /// <param name="pi">The problem instance.</param>
        /// <param name="vAgents">The agents that the pattern database should keep track of.</param>
        void init(ProblemInstance pi, List<uint> vAgents);
    }

    public interface ILazyHeuristic : IHeuristicCalculator
    {
        /// <summary>Returns the heuristic estimate. Used when a low level generated nodes cap is needed.</summary>
        /// <param name="s">The current state.</param>
        /// <param name="target">The lowest target estimate to return, if possible.</param>
        /// <param name="effectiveBranchingFactor">The branching factor so far of the A* search we're serving.</param>
        uint h(WorldState s, int target, float effectiveBranchingFactor);

        /// <summary>Returns the heuristic estimate. Used when a time cap is needed.</summary>
        /// <param name="s">The current state.</param>
        /// <param name="target">The lowest target estimate to return, if possible.</param>
        /// <param name="effectiveBranchingFactor">Ignored. Kept only to make the number of parameters different from the previous method.</param>
        /// <param name="millisCap">Stop the search when the process' total millisecond count reaches the cap.</param>
        /// <param name="resume">Whether to resume the last search. Assumes last search was from the same node</param>
        uint h(WorldState s, int target, float effectiveBranchingFactor, int millisCap, bool resume);
    }
}
