using System.Collections.Generic;

namespace mapf;

public interface IHeuristicCalculator<in State> : IAccumulatingStatisticsCsvWriter
{
    /// <summary>Returns the heuristic estimate.</summary>
    /// <param name="s">The current state.</param>
    uint h(State s);

    /// <summary>
    /// Initializes the pattern database by storing references to the
    /// problem instance and also the subset of agents that the pattern
    /// database pertains to.
    /// </summary>
    /// <param name="pi">The problem instance.</param>
    /// <param name="agentsToConsider">The agents that the heuristic should keep track of.</param>
    void Init(ProblemInstance pi, List<uint> agentsToConsider);

    /// <summary>
    /// Return the name of the heuristic, useful for outputing results.
    /// This is needed because ToString() is provided by the "object" class so it can't be used
    /// to force this behavior to be implemented;
    /// </summary>
    /// <returns>The name of the solver</returns>
    string GetName();
}

public interface ILazyHeuristic<in State> : IHeuristicCalculator<State>
{
    /// <summary>
    /// Returns the heuristic estimate. Stops estimation once the target estimate is reached.
    /// </summary>
    /// <param name="s">The current state.</param>
    /// <param name="target">The lowest target estimate to return, if possible.</param>
    uint h(State s, int target);
}

public interface IBoundedLazyHeuristic<in State> : IHeuristicCalculator<State>
{
    /// <summary>
    /// Returns the heuristic estimate. Stops estimation once the target estimate is reached.
    /// Used when a low level generated nodes cap is needed.
    /// </summary>
    /// <param name="s">The current state.</param>
    /// <param name="target">The lowest target estimate to return, if possible.</param>
    /// <param name="effectiveBranchingFactor">The branching factor so far of the A* search we're serving.</param>
    uint h(State s, int target, float effectiveBranchingFactor);

    /// <summary>Returns the heuristic estimate. Used when a time cap is needed.</summary>
    /// <param name="s">The current state.</param>
    /// <param name="target">The lowest target estimate to return, if possible.</param>
    /// <param name="effectiveBranchingFactor">Ignored. Kept only to make the number of parameters different from the previous method.</param>
    /// <param name="millisCap">Stop the search when the process' total millisecond count reaches the cap.</param>
    /// <param name="resume">Whether to resume the last search. Assumes last search was from the same node</param>
    uint h(State s, int target, float effectiveBranchingFactor, int millisCap, bool resume);
}
