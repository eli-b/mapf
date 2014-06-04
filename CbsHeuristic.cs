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
        protected List<uint> vAgents;
        public Run runner; // FIXME: Hack!
        protected bool validate;

        protected bool reportSolution;
        protected int minAboveSic;

        protected double totalRuntime;
        protected int totalImprovement;
        protected int nCalls;
        protected int highLevelExpanded;
        protected int highLevelGenerated;
        protected int lowLevelExpanded;
        protected int lowLevelGenerated;
        protected double accTotalRuntime;
        protected int accTotalImprovement;
        protected int accNCalls;
        protected int accHighLevelExpanded;
        protected int accHighLevelGenerated;
        protected int accLowLevelExpanded;
        protected int accLowLevelGenerated;

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
            this.minAboveSic = Math.Max(minAboveSic, 1);

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
            int sicEstimate = (int) SumIndividualCosts.h(s, this.instance);
            if (sicEstimate == 0)
                return 0;
            int targetCost = s.g + sicEstimate + this.minAboveSic; // Ariel's idea - using SIC directly here to calc the target
            // CBS gets an explicitly partially solved state - the agents' g may be greater than zero.
            // So the cost CBS is going to calc is not of this node but of the initial problem instance,
            // this is accounted for later too.
            // (Notice node usually has a (possibly very wrong) h set already - copied from the parent)
            return this.h(s, targetCost, sicEstimate);
        }

        /// <summary>
        /// 
        /// Assumes g of node was already calculated and h isn't zero.
        /// </summary>
        /// <param name="s"></param>
        /// <param name="targetCost"></param>
        /// <param name="sicEstimate">For a debug assertion and a statistic</param>
        /// <returns></returns>
        protected uint h(WorldState s, int targetCost, uint sicEstimate)
        {
            double start = Process.GetCurrentProcess().TotalProcessorTime.TotalMilliseconds;

            // Calc the h:
            ProblemInstance sAsProblemInstance = s.ToProblemInstance(this.instance);
            this.cbs.Setup(sAsProblemInstance, s.makespan, this.runner); // s isn't the goal (checked earlier in the method), so we must search at least until the next depth. This forces must conds to be upheld when dealing with A*+OD nodes.

            if (this.cbs.openList.Count > 0)
                Debug.Assert(((CbsNode)this.cbs.openList.Peek()).totalCost - s.g == (int)sicEstimate,
                    "Total cost of CBS root not same as SIC + g");

            this.cbs.targetCost = targetCost;

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
            // Discounting the moves the agents did before we started solving
            // (This is easier than making a copy of each AgentState just to zero its lastMove.time)

            this.totalImprovement += (int)(cbsEstimate - s.h); // Not computing difference from SIC to not over-count, since a node can be improved twice.
            
            if (validate)
            {
                // Brute-force validation of admissability of estimate:
                var sic = new SumIndividualCosts();
                sic.init(this.instance, this.vAgents);
                var epeastarsic = new AStarWithPartialExpansion(sic);
                epeastarsic.Setup(sAsProblemInstance, s.makespan, runner);
                bool epeastarsicSolved = epeastarsic.Solve();
                if (epeastarsicSolved)
                    Debug.Assert(epeastarsic.totalCost - s.g >= this.cbs.totalCost - s.g, "Inadmissable!!");
            }

            return cbsEstimate;
        }

        /// <summary>
        /// Part of the HeuristicCalculator interface.
        /// </summary>
        /// <param name="pi"></param>
        /// <param name="vAgents">Only passed to the underlying heuristic. TODO: Consider using in h() too.</param>
        public void init(ProblemInstance pi, List<uint> vAgents)
        {
            this.cbs.GetHeuristic().init(pi, vAgents); // Doesn't do anything special in SIC, but notice anyway that we're not init'ing with the PI that we're going to solve
            this.instance = pi;
            this.vAgents = vAgents;

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
            return "CBSH(" + this.reportSolution + " " + this.minAboveSic + ")";
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Total Expanded (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Total Generated (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Total Expanded (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Total Generated (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Average Expanded (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Average Generated (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Average Expanded (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Average Generated (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this + " Average Runtime");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this + " Average Improvement Achieved");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this + " Num Calls");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public void OutputStatistics(TextWriter output)
        {
            if (this.nCalls == 0) // No stats
            {
                for (int i=0; i<this.NumStatsColumns ; ++i)
                    output.Write(Run.RESULTS_DELIMITER);
                return;
            }

            // TODO: Make IAccumulatingStatisticsCsvWriter emit its statistics into an object, and AccumulateStatistics()
            //       receive it and increment it. This way we'll be able to preserve the entire statistics from the CBS
            //       calls, and not just the node counts.
            Console.WriteLine("{0} Total Expanded Nodes (High-Level): {1}", this, this.highLevelExpanded);
            Console.WriteLine("{0} Total Generated Nodes (High-Level): {1}", this, this.highLevelGenerated);
            Console.WriteLine("{0} Total Expanded Nodes (Low-Level): {1}", this, this.lowLevelExpanded);
            Console.WriteLine("{0} Total Generated Nodes (Low-Level): {1}", this, this.lowLevelGenerated);
            Console.WriteLine("{0} Average Expanded Nodes (High-Level): {1}", this, this.highLevelExpanded / this.nCalls);
            Console.WriteLine("{0} Average Generated Nodes (High-Level): {1}", this, this.highLevelGenerated / this.nCalls);
            Console.WriteLine("{0} Average Expanded Nodes (Low-Level): {1}", this, this.lowLevelExpanded / this.nCalls);
            Console.WriteLine("{0} Average Generated Nodes (Low-Level): {1}", this, this.lowLevelGenerated / this.nCalls);

            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.lowLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.lowLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelExpanded / this.nCalls + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated / this.nCalls + Run.RESULTS_DELIMITER);
            output.Write(this.lowLevelExpanded / this.nCalls + Run.RESULTS_DELIMITER);
            output.Write(this.lowLevelGenerated / this.nCalls + Run.RESULTS_DELIMITER);

            double averageRunTime = this.totalRuntime / this.nCalls;
            double averageImprovement = ((double)this.totalImprovement) / this.nCalls;

            Console.WriteLine("{0} Average Runtime: {1}", this, averageRunTime);
            Console.WriteLine("{0} Average Improvement achieved: {1}", this, averageImprovement);
            Console.WriteLine("{0} Num Calls: {1}", this, this.nCalls);

            output.Write(averageRunTime + Run.RESULTS_DELIMITER);
            output.Write(averageImprovement + Run.RESULTS_DELIMITER);
            output.Write(this.nCalls + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 11;
            }
        }

        /// <summary>
        /// Clears statistics.
        /// </summary>
        public void ClearStatistics()
        {
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 0;
            this.lowLevelExpanded = 0;
            this.lowLevelGenerated = 0;
            this.totalImprovement = 0;
            this.totalRuntime = 0;
            this.nCalls = 0;
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.accTotalRuntime = 0;
            this.accTotalImprovement = 0;
            this.accNCalls = 0;
            this.accHighLevelExpanded = 0;
            this.accHighLevelGenerated = 0;
            this.accLowLevelExpanded = 0;
            this.accLowLevelGenerated = 0;
        }

        public virtual void AccumulateStatistics()
        {
            this.accTotalRuntime += this.totalRuntime;
            this.accTotalImprovement += this.totalImprovement;
            this.accNCalls += this.nCalls;
            this.accHighLevelExpanded += this.highLevelExpanded;
            this.accHighLevelGenerated += this.accHighLevelGenerated;
            this.accLowLevelExpanded += this.lowLevelExpanded;
            this.accLowLevelGenerated += this.lowLevelGenerated;
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Total Expanded Nodes (High-Level): {1}", this, this.accHighLevelExpanded);
            Console.WriteLine("{0} Total Generated Nodes (High-Level): {1}", this, this.accHighLevelGenerated);
            Console.WriteLine("{0} Total Expanded Nodes (Low-Level): {1}", this, this.accLowLevelExpanded);
            Console.WriteLine("{0} Total Generated Nodes (Low-Level): {1}", this, this.accLowLevelGenerated);
            Console.WriteLine("{0} Average Expanded Nodes (High-Level): {1}", this, this.accHighLevelExpanded / this.accNCalls);
            Console.WriteLine("{0} Average Generated Nodes (High-Level): {1}", this, this.accHighLevelGenerated / this.accNCalls);
            Console.WriteLine("{0} Average Expanded Nodes (Low-Level): {1}", this, this.accLowLevelExpanded / this.accNCalls);
            Console.WriteLine("{0} Average Generated Nodes (Low-Level): {1}", this, this.accLowLevelGenerated / this.accNCalls);

            output.Write(this.accHighLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accHighLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.accLowLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accLowLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.accHighLevelExpanded / this.accNCalls + Run.RESULTS_DELIMITER);
            output.Write(this.accHighLevelGenerated / this.accNCalls + Run.RESULTS_DELIMITER);
            output.Write(this.accLowLevelExpanded / this.accNCalls + Run.RESULTS_DELIMITER);
            output.Write(this.accLowLevelGenerated / this.accNCalls + Run.RESULTS_DELIMITER);

            double averageRunTime = this.accTotalRuntime / this.accNCalls;
            double averageImprovement = this.accTotalImprovement / this.accNCalls;

            Console.WriteLine("{0} Average Runtime: {1}", this, averageRunTime);
            Console.WriteLine("{0} Average Improvement Achieved: {1}", this, averageImprovement);
            Console.WriteLine("{0} Num Calls: {1}", this, this.accNCalls);

            output.Write(averageRunTime + Run.RESULTS_DELIMITER);
            output.Write(averageImprovement + Run.RESULTS_DELIMITER);
            output.Write(this.accNCalls + Run.RESULTS_DELIMITER);            
        }
    }

    class DyanamicLazyCbsh : CbsHeuristic, LazyHeuristic
    {
        public DyanamicLazyCbsh(CBS_LocalConflicts cbs, Run runner, bool reportSolution = false, bool validate = false)
            : base(cbs, runner, reportSolution, -1, validate) {}

        /// <summary>
        /// Assumes g of node was already calculated.
        /// </summary>
        /// <param name="s"></param>
        /// <param name="target"></param>
        /// <returns></returns>
        public uint h(WorldState s, int targetH)
        {
            uint sicEstimate = SumIndividualCosts.h(s, this.instance);
            if (sicEstimate == 0)
                return 0;
            return this.h(s, s.g + targetH, sicEstimate);
        }

        public override string ToString()
        {
            return "DynamicLazyCBSH(" + this.reportSolution + ")";
        }
    }
}
