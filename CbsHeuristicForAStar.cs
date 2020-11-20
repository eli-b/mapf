using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace mapf
{
    /// <summary>
    /// Runs a bounded CBS to get an lower estimate of the cost from a given A* node to the goal.
    /// TODO: This class can actually be generalized to SolverHeuristic and be used as a brute-force estimator.
    ///       The only CBS things in it are the targetCost, the Trace.Assert that the root costs exactly like SIC's
    ///       estimate, and the statistics.
    /// </summary>
    class CbsHeuristicForAStar : IHeuristicCalculator<WorldState>
    {
        protected CBS cbs;
        protected ProblemInstance instance;
        protected List<uint> agentsToConsider;
        protected Run runner;
        protected bool validate;

        protected bool reportSolution;
        protected int minAboveSic;

        protected double totalRuntime;
        protected int totalImprovement;
        protected int nCalls;
        protected int nodesSolved;
        protected double accTotalRuntime;
        protected int accTotalImprovement;
        protected int accNCalls;
        protected int accNodesSolved;

        /// <summary>
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
        /// <param name="validate"></param>
        public CbsHeuristicForAStar(CBS cbs, Run runner, bool reportSolution = false,
                            int minAboveSic = 1, bool validate = false)
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
            int sicEstimate;
            if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                sicEstimate = (int) SumIndividualCosts.h(s, this.instance);
            else if (Constants.costFunction == Constants.CostFunction.MAKESPAN || Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                sicEstimate = (int) MaxIndividualCosts.h(s, this.instance);
            else
                throw new Exception($"Unsupported cost function {Constants.costFunction}");
            if (sicEstimate == 0)  // Only the goal has an estimate of zero
                return 0;
            int targetCost = s.g + sicEstimate + this.minAboveSic; // Ariel's idea - using SIC directly here to calc the target
            // CBS gets an explicitly partially solved state - the agents' g may be greater than zero.
            // So the cost CBS is going to calc is not of this node but of the initial problem instance,
            // this is accounted for later too.
            // (Notice node usually has a (possibly too low) h set already - inherited from the parent)
            return this.h(s, targetCost, sicEstimate);
        }

        /// <summary>
        /// Computes a heuristic by running a bounded CBS search from the given node.
        /// Assumes g of node was already calculated and h isn't zero.
        /// </summary>
        /// <param name="s"></param>
        /// <param name="targetCost">Stop when the target cost is reached</param>
        /// <param name="sicEstimate">For a debug assertion.</param>
        /// <param name="lowLevelGeneratedCap">The number of low level nodes to generate</param>
        /// <param name="milliCap">The process total millisecond count to stop at</param>
        /// <param name="resume">Whether to resume the last search instead of solving the given node. Assumes the last search was from the same node as the given node.</param>
        /// <returns></returns>
        protected uint h(WorldState s, int targetCost, int sicEstimate=-1, int lowLevelGeneratedCap=-1,
                         int milliCap=int.MaxValue, bool resume=false)
        {
            double start = this.runner.ElapsedMilliseconds();

            ProblemInstance sAsProblemInstance;
            if (resume == false)
            {
                this.cbs.Clear();
                sAsProblemInstance = s.ToProblemInstance(this.instance);
                this.cbs.Setup(sAsProblemInstance,
                               Math.Max(s.makespan,  // This forces must-constraints to be upheld when dealing with A*+OD nodes,
                                                     // at the cost of forcing every agent to move when a goal could be found earlier with all must constraints upheld.
                                        s.minGoalTimeStep), // No point in finding shallower goal nodes
                               this.runner, null, null, null);
                
                if (this.cbs.openList.Count > 0 && this.cbs.topMost)
                {
                    if (sicEstimate == -1)
                    {
                        if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                            sicEstimate = (int) SumIndividualCosts.h(s, this.instance);
                        else if (Constants.costFunction == Constants.CostFunction.MAKESPAN || Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                            sicEstimate = (int) MaxIndividualCosts.h(s, this.instance);
                        else
                            throw new Exception($"Unsupported cost function {Constants.costFunction}");
                    }

                    Trace.Assert(((CbsNode)this.cbs.openList.Peek()).g - s.g == (int)sicEstimate,
                                    "Total cost of CBS root not same as SIC + g");
                    // Notice we're subtracting s.g, not sAsProblemInstance.g.
                    // Must constraints we put may have forced some moves,
                    // and we shouldn't count them as part of the estimate.
                }
            }
            else
                sAsProblemInstance = this.cbs.GetProblemInstance();

            if (lowLevelGeneratedCap == -1)
            {
                // Rough estimate of the branching factor:
                lowLevelGeneratedCap = (int) Math.Pow(Constants.NUM_ALLOWED_DIRECTIONS, this.instance.agents.Length);
            }

            // Calc the h:
            this.cbs.targetF = targetCost;
            this.cbs.milliCap = milliCap;
            this.cbs.lowLevelGeneratedCap = lowLevelGeneratedCap;

            bool solved = this.cbs.Solve();

            if (solved && this.reportSolution)
            {
                // We're always going to find a proper goal since we respected the node's minDepth
                s.SetSolution(this.cbs.GetSinglePlans());
                s.SetGoalCost(this.cbs.solutionCost); // We have to do it explicitly.
                // We can't just change the node's g to g + cbs.g and its h to zero
                // because approaches like BPMX or maximazing PDBs might "fix" the h back.
                // So instead h is bumped to its maximum value when this method returns.
                s.SetSingleCosts(this.cbs.GetSingleCosts());
                this.nodesSolved++;
            }

            double end = this.runner.ElapsedMilliseconds();
            this.totalRuntime += end - start;
            this.nCalls++;

            this.cbs.AccumulateStatistics();
            this.cbs.ClearStatistics();

            if (this.cbs.solutionCost < 0) // A timeout is legitimately possible if very little time was left to begin with,
                                           // and a no solution failure may theoretically be possible too.
            {
                if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                    return SumIndividualCosts.h(s, this.instance);
                else if (Constants.costFunction == Constants.CostFunction.MAKESPAN || Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                    return MaxIndividualCosts.h(s, this.instance);
                else
                    throw new Exception($"Unsupported cost function {Constants.costFunction}");
            }

            Trace.Assert(this.cbs.solutionCost >= s.g,
                         $"CBS total cost {this.cbs.solutionCost} is smaller than starting problem's initial cost {s.g}."); // = is allowed since even though this isn't a goal node (otherwise this function won't be called),
                                                                                                                         // a non-goal node can have h==0 if a minimum depth is specified, and all agents have reached their
                                                                                                                         // goal in this node, but the depth isn't large enough.

            uint cbsEstimate = (uint)(this.cbs.solutionCost - s.g);
            // Discounting the moves the agents did before we started solving
            // (This is easier than making a copy of each AgentState just to zero its lastMove.time)

            this.totalImprovement += (int)(cbsEstimate - s.h); // Not computing difference from SIC to not over-count, since a node can be improved twice.
                                                               // Can be negative if the base heuristic was improved by:
                                                               // - Partial expansion
                                                               // - BPMX
            
            if (validate)
            {
                // Brute-force validation of admissibility of estimate:
                IHeuristicCalculator<WorldState> heuristic;
                if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                    heuristic = new SumIndividualCosts();
                else if (Constants.costFunction == Constants.CostFunction.MAKESPAN || Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                    heuristic = new MaxIndividualCosts();
                else
                    throw new Exception($"Unsupported cost function {Constants.costFunction}");
                
                heuristic.Init(this.instance, this.agentsToConsider);
                var epeastarsic = new EPEA_Star(heuristic);
                epeastarsic.Setup(sAsProblemInstance, s.makespan, runner);
                bool epeastarsicSolved = epeastarsic.Solve();
                if (epeastarsicSolved)
                    Trace.Assert(epeastarsic.totalCost - s.g >= this.cbs.solutionCost - s.g, "Inadmissible!!");
            }

            return cbsEstimate;
        }

        /// <summary>
        /// Part of the HeuristicCalculator interface.
        /// </summary>
        /// <param name="pi"></param>
        /// <param name="agentsToConsider">Only passed to the underlying heuristic. TODO: Consider using in h() too.</param>
        public void Init(ProblemInstance pi, List<uint> agentsToConsider)
        {
            this.instance = pi;
            this.agentsToConsider = agentsToConsider;

            this.cbs.ClearAccumulatedStatistics();

            this.totalRuntime = 0;
            this.totalImprovement = 0;
            this.nCalls = 0;
        }

        public override string ToString()
        {
            return $"CBS Heutistic({this.reportSolution} {this.minAboveSic})";
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            this.cbs.OutputStatisticsHeader(output);

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
            output.Write(this + " Nodes Solved");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this + " Num Calls");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public void OutputStatistics(TextWriter output)
        {
            this.cbs.OutputAccumulatedStatistics(output);

            if (this.nCalls == 0) // No stats
            {
                for (int i = 0; i < this.NumStatsColumns - this.cbs.NumStatsColumns; ++i)
                    output.Write(Run.RESULTS_DELIMITER);
                return;
            }

            // TODO: Make IAccumulatingStatisticsCsvWriter emit its statistics into an object, and AccumulateStatistics()
            //       receive it and increment it. This way we'll be able to preserve the entire statistics from the CBS
            //       calls, and not just the node counts.
            Console.WriteLine("{0} Average Expanded Nodes (High-Level): {1}", this, this.cbs.GetAccumulatedExpanded() / this.nCalls);
            Console.WriteLine("{0} Average Generated Nodes (High-Level): {1}", this, this.cbs.GetAccumulatedGenerated() / this.nCalls);
            Console.WriteLine("{0} Average Expanded Nodes (Low-Level): {1}", this, this.cbs.GetLowLevelExpanded() / this.nCalls);
            Console.WriteLine("{0} Average Generated Nodes (Low-Level): {1}", this, this.cbs.GetLowLevelGenerated() / this.nCalls);

            output.Write(this.cbs.GetAccumulatedExpanded() / this.nCalls + Run.RESULTS_DELIMITER);
            output.Write(this.cbs.GetAccumulatedGenerated() / this.nCalls + Run.RESULTS_DELIMITER);
            output.Write(this.cbs.GetLowLevelExpanded() / this.nCalls + Run.RESULTS_DELIMITER);
            output.Write(this.cbs.GetLowLevelGenerated() / this.nCalls + Run.RESULTS_DELIMITER);

            double averageRunTime = this.totalRuntime / this.nCalls;
            double averageImprovement = ((double)this.totalImprovement) / this.nCalls;

            Console.WriteLine("{0} Average Runtime: {1}", this, averageRunTime);
            Console.WriteLine("{0} Average Improvement achieved: {1}", this, averageImprovement);
            Console.WriteLine("{0} Nodes Solved: {1}", this, this.nodesSolved);
            Console.WriteLine("{0} Num Calls: {1}", this, this.nCalls);

            output.Write(averageRunTime + Run.RESULTS_DELIMITER);
            output.Write(averageImprovement + Run.RESULTS_DELIMITER);
            output.Write(this.nodesSolved + Run.RESULTS_DELIMITER);
            output.Write(this.nCalls + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 8 + this.cbs.NumStatsColumns;
            }
        }

        /// <summary>
        /// Clears statistics.
        /// </summary>
        public void ClearStatistics()
        {
            //this.cbs.ClearStatistics(); // Already done after each CBS run
            this.totalImprovement = 0;
            this.totalRuntime = 0;
            this.nodesSolved = 0;
            this.nCalls = 0;
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.cbs.ClearAccumulatedStatistics();
            this.accTotalRuntime = 0;
            this.accTotalImprovement = 0;
            this.accNodesSolved = 0;
            this.accNCalls = 0;
        }

        public virtual void AccumulateStatistics()
        {
            //this.cbs.AccumulateStatistics(); // Already done after each CBS run
            this.accTotalRuntime += this.totalRuntime;
            this.accTotalImprovement += this.totalImprovement;
            this.accNodesSolved += this.nodesSolved;
            this.accNCalls += this.nCalls;
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            this.cbs.OutputAccumulatedStatistics(output);

            double accAverageExpandedHigh = 0;
            double accAverageGeneratedHigh = 0;
            double accAverageExpandedLow = 0;
            double accAverageGeneratedLow = 0;
            double accAverageRunTime = 0;
            double accAverageImprovement = 0;

            if (this.nCalls != 0) // Stats are available
            {
                accAverageExpandedHigh = (double)this.cbs.GetAccumulatedExpanded() / this.nCalls;
                accAverageGeneratedHigh = (double)this.cbs.GetAccumulatedGenerated() / this.nCalls;
                accAverageExpandedLow = (double)this.cbs.GetLowLevelExpanded() / this.nCalls;
                accAverageGeneratedLow = (double)this.cbs.GetLowLevelGenerated() / this.nCalls;
                accAverageRunTime = this.accTotalRuntime / this.accNCalls;
                accAverageImprovement = (double)this.accTotalImprovement / this.accNCalls;
            }

            Console.WriteLine("{0} Accumulated Average Expanded Nodes (High-Level): {1}", this, accAverageExpandedHigh);
            Console.WriteLine("{0} Accumulated Average Generated Nodes (High-Level): {1}", this, accAverageGeneratedHigh);
            Console.WriteLine("{0} Accumulated Average Expanded Nodes (Low-Level): {1}", this, accAverageExpandedLow);
            Console.WriteLine("{0} Accumulated Average Generated Nodes (Low-Level): {1}", this, accAverageGeneratedLow);

            output.Write(accAverageExpandedHigh + Run.RESULTS_DELIMITER);
            output.Write(accAverageGeneratedHigh + Run.RESULTS_DELIMITER);
            output.Write(accAverageExpandedLow + Run.RESULTS_DELIMITER);
            output.Write(accAverageGeneratedLow + Run.RESULTS_DELIMITER);

            Console.WriteLine("{0} Average Runtime: {1}", this, accAverageRunTime);
            Console.WriteLine("{0} Average Improvement Achieved: {1}", this, accAverageImprovement);
            Console.WriteLine("{0} Nodes Solved: {1}", this, this.accNodesSolved);
            Console.WriteLine("{0} Num Calls: {1}", this, this.accNCalls);

            output.Write(accAverageRunTime + Run.RESULTS_DELIMITER);
            output.Write(accAverageImprovement + Run.RESULTS_DELIMITER);
            output.Write(this.accNodesSolved + Run.RESULTS_DELIMITER);
            output.Write(this.accNCalls + Run.RESULTS_DELIMITER);            
        }

        public string GetName()
        {
            return this.ToString();
        }
    }

    class DyanamicLazyCbsHeuristicForAStar : CbsHeuristicForAStar, IBoundedLazyHeuristic<WorldState>
    {
        public DyanamicLazyCbsHeuristicForAStar(CBS cbs, Run runner, bool reportSolution = false, bool validate = false)
            : base(cbs, runner, reportSolution, -1, validate) {}

        /// <summary>
        /// Assumes g of node was already calculated.
        /// </summary>
        /// <param name="s"></param>
        /// <param name="targetH"></param>
        /// <param name="effectiveBranchingFactor"></param>
        /// <returns></returns>
        public uint h(WorldState s, int targetH, float effectiveBranchingFactor)
        {
            // No need to check if SIC is zero because this heuristic is run after SIC was already computed, not instead of it.
            int lowLevelGeneratedCap = (int) Math.Round(effectiveBranchingFactor * this.instance.agents.Length); // Cap of B_of_AStar * K,
                                                                                                                 // because CBS low level nodes are of one agent only so they're about k times cheaper to work with
            return base.h(s, s.g + targetH, -1, lowLevelGeneratedCap);
        }

        /// <summary>
        /// Assumes g of node was already calculated.
        /// </summary>
        /// <param name="s"></param>
        /// <param name="targetH"></param>
        /// <param name="effectiveBranchingFactor"></param>
        /// <param name="millisCap"></param>
        /// <param name="resume"></param>
        /// <returns></returns>
        public uint h(WorldState s, int targetH, float effectiveBranchingFactor, int millisCap, bool resume)
        {
            // No need to check if SIC is zero because this heuristic is run after SIC was already computed, not instead of it.
            return this.h(s, s.g + targetH, -1, int.MaxValue, millisCap, resume);
        }

        public override string ToString()
        {
            return $"DynamicLazyCBSH({this.cbs})";
        }
    }
}
