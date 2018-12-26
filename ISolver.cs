using System.Collections.Generic;

namespace CPF_experiment
{
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
        /// <returns></returns>
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
        int GetMaxGroupSize();

        int GetExpanded();
        int GetGenerated();
    }

    public interface ICbsSolver : ISolver, IAccumulatingStatisticsCsvWriter
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="problemInstance"></param>
        /// <param name="minTimeStep"></param>
        /// <param name="runner"></param>
        /// <param name="minCost">
        /// Goal nodes with a lower cost aren't considered a goal.
        /// This can be used to improve the heuristic estimate!
        /// </param>
        void Setup(ProblemInstance problemInstance, int minTimeStep, Run runner, int minCost);
        SinglePlan[] GetSinglePlans();
        int[] GetSingleCosts();
        Dictionary<int, int> GetExternalConflictCounts();
        Dictionary<int, List<int>> GetConflictTimes();

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
}
