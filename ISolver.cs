using System;
using System.IO;

namespace CPF_experiment
{
    public interface ISolver : IStatisticsCsvWriter
    {
        /// <summary>
        /// Return the name of the solver, useful for outputing results.
        /// </summary>
        /// <returns>The name of the solver</returns>
        String GetName();

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
        /// Set the heuristic
        /// </summary>
        /// <param name="heuristic"></param>
        void SetHeuristic(HeuristicCalculator heuristic);

        HeuristicCalculator GetHeuristic();

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
        void Setup(ProblemInstance problemInstance, int minDepth, Run runner);
        SinglePlan[] GetSinglePlans();
        int[] GetSingleCosts();

        int GetAccumulatedExpanded();
        int GetAccumulatedGenerated();
    }
}
