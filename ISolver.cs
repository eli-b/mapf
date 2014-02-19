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
        /// <param name="runner"></param>
        /// <returns></returns>
        bool Solve(Run runner);


        /// <summary>
        /// Setup the relevant data structures for a run.
        /// </summary>
        void Setup(ProblemInstance problemInstance);


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
        /// gets the delta of (actual solution cost - first state heuristics)
        /// </summary>
        /// <returns></returns>
        int getSolutionDepth();

        int getNodesPassedPruningCounter();
        long getMemoryUsed();
        int getHighLevelExpanded();
        int getHighLevelGenerated();
        int getLowLevelExpanded();
        int getLowLevelGenerated();
        int getMaxGroupSize();

    }

    public interface IDnCSolver : ISolver
    {
        void Setup(ProblemInstance problemInstance, int minDepth);
        SinglePlan[] getSinglePlans();
    }
}
