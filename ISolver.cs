using System.Collections.Generic;

namespace mapf;

public interface ISolver : IStatisticsCsvWriter
{
    /// <summary>
    /// Return the name of the solver, useful for outputing results.
    /// </summary>
    /// <returns>The name of the solver</returns>
    string GetName();

    /// <summary>
    /// Solves the instance that was set by a call to Setup()
    /// </summary>
    /// <returns>true if a solution was found, false otherwise</returns>
    bool Solve();

    /// <summary>
    /// Setup the relevant data structures for a run.
    /// </summary>
    /// <param name="problemInstance"></param>
    /// <param name="runner"></param>
    void Setup(ProblemInstance problemInstance, Run runner);

    /// <summary>
    /// Clears the relevant data structures and variables to free memory usage.
    /// </summary>
    void Clear();

    /// <summary>
    /// Returns the found plan, or null if no plan was found.
    /// </summary>
    /// <returns></returns>
    Plan GetPlan();

    /// <summary>
    /// Returns the cost of the solution found, or error codes otherwise.
    /// </summary>
    int GetSolutionCost();

    /// <summary>
    /// Gets the delta of (actual solution cost - first state heuristics)
    /// </summary>
    int GetSolutionDepth();

    long GetMemoryUsed();

    // Not all algorithms have meaningful stats for expanded/generated, but many do
    int GetExpanded();
    int GetGenerated();
}

public interface IConflictReporting
{
    /// <summary>
    /// </summary>
    /// <returns>Map each external agent to the number of conflicts with their path the solution has</returns>
    Dictionary<int, int> GetExternalConflictCounts();
    /// <summary>
    /// </summary>
    /// <returns>Map each external agent to a list of times the solution has a conflict with theirs</returns>
    Dictionary<int, List<int>> GetConflictTimes();
}

public interface IIndependenceDetectionSolver : ISolver, IConflictReporting, IAccumulatingStatisticsCsvWriter
{
    /// <summary>
    /// For new groups
    /// </summary>
    /// <param name="problemInstance"></param>
    /// <param name="runner"></param>
    /// <param name="CAT"></param>
    /// <param name="parentGroup1Cost"></param>
    /// <param name="parentGroup2Cost"></param>
    /// <param name="parentGroup1Size"></param>
    void Setup(ProblemInstance problemInstance, Run runner, ConflictAvoidanceTable CAT,
                int parentGroup1Cost, int parentGroup2Cost, int parentGroup1Size);

    /// <summary>
    /// For replanning groups to resolve a conflict
    /// </summary>
    /// <param name="problemInstance"></param>
    /// <param name="runner"></param>
    /// <param name="CAT"></param>
    /// <param name="targetCost">/// </param>
    /// <param name="illegalMoves"></param>
    void Setup(ProblemInstance problemInstance, Run runner, ConflictAvoidanceTable CAT,
                int targetCost, ISet<TimedMove> illegalMoves);
    int[] GetSingleCosts();

    int GetAccumulatedExpanded();
    int GetAccumulatedGenerated();
}

public interface ICbsSolver : ISolver, IConflictReporting, IAccumulatingStatisticsCsvWriter
{
    /// <summary>
    /// 
    /// </summary>
    /// <param name="problemInstance"></param>
    /// <param name="minTimeStep">Used mostly to force constraints to have an effect</param>
    /// <param name="runner"></param>
    /// <param name="CAT"></param>
    /// <param name="constraints"></param>
    /// <param name="positiveConstraints"></param>
    /// <param name="minCost">
    /// Goal nodes with a lower cost aren't considered a goal.
    /// This can be used to speed up the search!
    /// </param>
    /// <param name="maxCost">If known, can speed up the search (no surplus nodes would be generated)</param>
    /// <param name="mdd">Optional MDD of cost minCost=maxCost</param>
    void Setup(ProblemInstance problemInstance, int minTimeStep, Run runner,
                ConflictAvoidanceTable CAT, ISet<CbsConstraint> constraints, ISet<CbsConstraint> positiveConstraints,
                int minCost, int maxCost, MDD mdd);
    SinglePlan[] GetSinglePlans();
    int[] GetSingleCosts();

    int GetAccumulatedExpanded();
    int GetAccumulatedGenerated();
}

public interface IMStarSolver : ICbsSolver
{
    // Just does the appropriate thing when it's under M-Star
}

public interface IHeuristicSolver<in State>
{
    /// <summary>
    /// Get the heuristic
    /// </summary>
    /// <returns></returns>
    IHeuristicCalculator<State> GetHeuristic();
}
