using System;
using System.IO;

namespace CPF_experiment
{
    public interface ISolver
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
        /// Returns the goal state if it was found. Otherwise returns null.
        /// </summary>
        WorldState GetGoal();

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
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        void OutputStatistics(TextWriter output);

        /// <summary>
        /// Gets the delta of (actual solution cost - first state heuristics)
        /// </summary>
        int GetSolutionDepth();

        int GetNodesPassedPruningCounter(); // What does this mean?
        long GetMemoryUsed();
        int GetHighLevelExpanded();
        int GetHighLevelGenerated();
        int GetLowLevelExpanded();
        int GetLowLevelGenerated();
        int GetMaxGroupSize();
    }

    public interface ICbsSolver : ISolver
    {
        void Setup(ProblemInstance problemInstance, int minDepth, Run runner);
        SinglePlan[] GetSinglePlans();
    }
}
