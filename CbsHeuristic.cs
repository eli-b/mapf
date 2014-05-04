using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// TODO: This class can actually be generalized to SolverHeuristic and be used as a brute-force estimator.
    ///       The only CBS things in it are the targetCost, the Debug.Assert that the root costs exactly like SIC's
    ///       estimate, and the statistics.
    /// </summary>
    class CbsHeuristic : HeuristicCalculator
    {
        protected CBS_LocalConflicts cbs;
        protected ProblemInstance instance;
        protected Run runner;
        protected bool validate;

        protected bool reportSolution;
        protected int minAboveSic;

        protected double totalRuntime;
        protected double totalImprovement;
        protected int nCalls;

        protected int highLevelExpanded;
        protected int highLevelGenerated;
        protected int lowLevelExpanded;
        protected int lowLevelGenerated;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="cbs">The underlying CBS to use</param>
        /// <param name="runner"></param>
        /// <param name="reportSolution">
        /// Whether to store the solution found by CBS in the node.
        /// This should greatly speed up searches.</param>
        /// <param name="minAboveSic">
        /// The minimum increment by which to beat SIC's estimate, if possible.
        /// Larger values would cause each call to the heuristic to take longer, but make it return better estimates.
        /// </param>
        public CbsHeuristic(CBS_LocalConflicts cbs, Run runner, bool reportSolution = false, int minAboveSic = 1, bool validate = false)
        {
            this.cbs = cbs;
            this.runner = runner;

            this.reportSolution = reportSolution;
            minAboveSic = Math.Max(minAboveSic, 1);
            this.minAboveSic = minAboveSic;

            this.validate = validate;
        }

        /// <summary>
        /// 
        /// Assumes g of node was already calculated!
        /// </summary>
        /// <param name="s"></param>
        /// <returns></returns>
        public uint h(WorldState s)
        {
            uint sicEstimate = this.cbs.GetHeuristic().h(s);
            if (sicEstimate == 0)
                return 0;

            double start = Process.GetCurrentProcess().TotalProcessorTime.TotalMilliseconds;

            // Calc the h:
            ProblemInstance sAsProblemInstance = s.ToProblemInstance(this.instance);
            this.cbs.Setup(sAsProblemInstance, s.makespan, this.runner); // s isn't the goal (checked earlier in the method), so we must search at least until the next depth. This forces must conds to be upheld when dealing with A*+OD nodes.

            if (this.cbs.openList.Count > 0)
                Debug.Assert(((CbsNode)this.cbs.openList.Peek()).totalCost - s.g == (int)this.cbs.GetHeuristic().h(s),
                    "Total cost of CBS root not same as SIC + g");

            this.cbs.targetCost = s.g + (int)sicEstimate + this.minAboveSic; // Ariel's idea - using SIC directly here to calc the target
            // CBS gets an explicitly partially solved state - the agents' g may be greater than zero.
            // So the cost CBS is going to calc is not of this node but of the initial problem instance,
            // this is accounted for later too.
            // (Notice node usually has a (possibly very wrong) h set already - copied from the parent)

            bool solved = this.cbs.Solve();

            if (solved && this.reportSolution)
            {
                s.SetSolution(this.cbs.GetPlan());
                s.SetGoalCost(this.cbs.totalCost);
            }

            double end = Process.GetCurrentProcess().TotalProcessorTime.TotalMilliseconds;
            this.totalRuntime += end - start;
            this.nCalls++;

            this.highLevelExpanded += this.cbs.GetHighLevelExpanded();
            this.highLevelGenerated += this.cbs.GetHighLevelGenerated();
            this.lowLevelExpanded += this.cbs.GetLowLevelExpanded();
            this.lowLevelGenerated += this.cbs.GetLowLevelGenerated();

            if (this.cbs.totalCost < 0) // A timeout is legitimately possible if very little time was left to begin with,
                                        // and a no solution failure may theoretically be possible too.
                return this.cbs.GetHeuristic().h(s);

            Debug.Assert(this.cbs.totalCost > s.g, "CBS total cost is smaller than starting problem's initial cost."); // > and not >= because we already checked in the beginning that s isn't the goal.

            uint cbsEstimate = (uint)(this.cbs.totalCost - s.g);

            this.totalImprovement += cbsEstimate - sicEstimate;

            if (validate && nCalls > 180)
            {
                // Brute-force validation of admissability of estimate:
                var sic = this.cbs.GetHeuristic();
                var epeastarsic = new AStarWithPartialExpansion(sic);
                epeastarsic.Setup(sAsProblemInstance, s.makespan, runner);
                bool epeastarsicSolved = epeastarsic.Solve();
                if (epeastarsicSolved)
                    Debug.Assert(epeastarsic.totalCost - s.g >= this.cbs.totalCost - s.g, "Inadmissable!!");
            }

            // Discounting the moves the agents did before we started solving
            // (This is easier than making a copy of each AgentState just to zero its lastMove.time)
            return cbsEstimate;
        }

        /// <summary>
        /// Part of the HeuristicCalculator interface. Irrelevant since each WorldState is a new ProblemInstance for us
        /// </summary>
        /// <param name="pi"></param>
        /// <param name="vAgents">Only passed to the underlying heuristic. TODO: Consider using in h() too.</param>
        public void init(ProblemInstance pi, List<uint> vAgents)
        {
            this.cbs.GetHeuristic().init(pi, vAgents); // Doesn't do anything special in SIC, but notice anyway that we're not init'ing with the PI that we're going to solve
            this.instance = pi;
            //this.cbs.instance = pi;

            this.highLevelExpanded = 0;
            this.highLevelGenerated = 0;
            this.lowLevelExpanded = 0;
            this.lowLevelGenerated = 0;

            this.totalRuntime = 0;
            this.totalImprovement = 0;
            this.nCalls = 0;
        }

        public override string ToString()
        {
            return "CBSH(" + this.reportSolution + ", " + this.minAboveSic + ")";
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write("\"" + this.ToString() + " Expanded (HL)\"");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write("\"" + this.ToString() + " Generated (HL)\"");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write("\"" + this.ToString() + " Expanded (LL)\"");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write("\"" + this.ToString() + " Generated (LL)\"");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write("\"" + this + " Averge Runtime\"");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write("\"" + this + " Averge Improvement Over SIC\"");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write("\"" + this + " Num Calls\"");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Total Expanded Nodes (High-Level): {1}", this, this.highLevelExpanded);
            Console.WriteLine("{0} Total Generated Nodes (High-Level): {1}", this, this.highLevelGenerated);
            Console.WriteLine("{0} Total Expanded Nodes (Low-Level): {1}", this, this.lowLevelExpanded);
            Console.WriteLine("{0} Total Generated Nodes (Low-Level): {1}", this, this.lowLevelGenerated);

            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.lowLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.lowLevelGenerated + Run.RESULTS_DELIMITER);

            double averageRunTime = this.totalRuntime / this.nCalls;
            double averageImprovement = this.totalImprovement / this.nCalls;

            Console.WriteLine("{0} Average Runtime: {1}", this, averageRunTime);
            Console.WriteLine("{0} Average Improvement over SIC: {1}", this, averageImprovement);
            Console.WriteLine("{0} Num Calls: {1}", this, this.nCalls);

            output.Write(averageRunTime + Run.RESULTS_DELIMITER);
            output.Write(averageImprovement + Run.RESULTS_DELIMITER);
            output.Write(this.nCalls + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 7;
            }
        }

        /// <summary>
        /// Clears statistics.
        /// </summary>
        public void ClearStatistics()
        {
            this.totalRuntime = 0;
            this.nCalls = 0;
        }
    }
}