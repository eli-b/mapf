using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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
        /// Setup the relevant datastructures for a run.
        /// </summary>
        void Setup(ProblemInstance problemInstance);


        /// <summary>
        /// Clears the relevant datastructures and variables to free memory usage.
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
        /// grts the delta of (actual solution cost - first state huristics)
        /// </summary>
        /// <returns></returns>
        int getSolutionDepth();

        int getNodesPassedPruningCounter();
        long getMemuryUsed();
        int getHighLeveExpanded();
        int getHighLeveGenerated();
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
