using System;
using System.IO;

namespace CPF_experiment
{
    public class DynamicRationalLazyOpenList : OpenList
    {
        public LazyHeuristic expensive;
        protected int lastF;
        protected int successes;
        protected int failures;
        protected int skips;
        protected int accSuccesses;
        protected int accFailures;
        protected int accSkips;
        protected static readonly double movingAverageFactor = 0.4;
        /// <summary>
        /// Probability that the expensive heuristic is helpful
        /// </summary>
        protected double Ph;
        protected double accPh;

        public DynamicRationalLazyOpenList(ISolver user, LazyHeuristic expensive)
            : base(user)
        {
            this.expensive = expensive;
            this.ClearPrivateStatistics();
        }

        protected void ClearPrivateStatistics()
        {
            this.successes = 0;
            this.failures = 0;
            this.skips = 0;
            this.Ph = 1; // Initial (hopeful) estimate - this give the heuristic enough time to run in the beginning
        }

        public override string ToString()
        {
            return "DynamicRationalLazyOpenList(alpha="+ DynamicRationalLazyOpenList.movingAverageFactor + ")/" + this.expensive;
        }

        public override IBinaryHeapItem Remove()
        {
            WorldState node;

            if (base.Count == 1) // Can happen more than once with the same node if partial expansion pushes it back into the open list, leading to multiple zero-timeout runs.
            {
                node = (WorldState)base.Remove();
                if (node.GoalTest() == true || // Can't improve the h of the goal
                    ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds() > Constants.MAX_TIME) // No time to continue improving H. // FIXME: Hack!
                    return node;
                int expensiveEstimate = (int)this.expensive.h(node, int.MaxValue, -1, 0, false); // I'm willing to pay the overhead to instantly solve problems of depth zero.
                node.h = Math.Max(node.h, expensiveEstimate);
            }
            else
            {
                // There are alternatives to the lowest cost node in the open list, try to postpone expansion of it:
                float branchingFactor = ((float)this.user.GetGenerated() - 1) / this.user.GetExpanded();
                if (this.user is ClassicAStar) // HACK!! Just until I copy this method to the interface and implement it everywhere.
                    branchingFactor = ((ClassicAStar)this.user).GetEffectiveBranchingFactor();
                const double binaryHeapTau = 0.000073359375; // milliseconds. From empirical experiments with this infra.
                double logN = Math.Log(this.heap.Count, 2); // Removals from the queue cost practically zero.
                double overhead = 0.023 * ((WorldState)this.Peek()).allAgentsState.Length; // Empirical lowest estimate. The cost of a zero-timeout CBSH run wasn't simply linear with the number of agents for some reason.

                while (true)
                {
                    WorldState next;
                    node = (WorldState)base.Remove();

                    if (node.GoalTest() == true || // Can't improve the h of the goal
                        ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds() > Constants.MAX_TIME) // No time to continue improving H. // FIXME: Hack!
                    {
                        if (node.g + node.h < this.lastF) // This can happen if the last removed node had many runs of the expensive heuristic, which this node didn't yet have.
                            node.h = this.lastF - node.g; // Just so we don't throw an inconsistency exception
                        break;
                    }

                    next = (WorldState)base.Peek();
                    int targetH = next.g + next.h + 1 - node.g;
                    // RLA* formula (Tolpin et al.):
                    //if (branchingFactor >= 1)
                    //    this.Ph = 1 - 1 / Math.Pow(branchingFactor, node.makespan); // TODO: Exponential average or something.
                    //else
                    //    this.Ph = 0; // Must expand all children
                    //Console.WriteLine("{0} {1} {2} {3} {4}", branchingFactor, node.makespan, Math.Pow(branchingFactor, node.makespan), 1 / Math.Pow(branchingFactor, node.makespan), Ph);
                    double denom = 1 - this.Ph * branchingFactor; // In our variant it's not enough to know that we want to run the expensive heuristic, we still need the formula to know how long to run it.
                    if (denom <= 0)
                    {
                        denom = 0.001; // Very small but not negative
                    }
                    double millisCap = binaryHeapTau * this.Ph * (branchingFactor + 1) * logN / denom;
                    bool success = false;
                    //Console.WriteLine("b is {2} logn is {3} Cap is {0} denom is {4} Ph is {5}, expected overhead is {1}", millisCap, overhead, branchingFactor, logN, denom, Ph);
                    if (millisCap > overhead || // Worth running the expensive heuristic
                        node.g + node.h < lastF) // Must improve the heuristic estimate to be consistent
                    {
                        int expensiveEstimate = (int)this.expensive.h(node, targetH, -1, (int)(((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds() + millisCap), false);
                        if (expensiveEstimate > node.h) // Node may have inherited a better estimate from its parent
                            node.h = expensiveEstimate; // Not necessarily a success, we later check if the increment was large enough.
                        
                        success = node.CompareTo(next) == 1; // node is not the smallest F anymore - re-insert it into the open list
                        if (success)
                            this.successes++;
                        else
                        {
                            // Give the heuristic another chance, with a double cap.
                            expensiveEstimate = (int)this.expensive.h(node, targetH, -1, (int)(((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds() + 2 * millisCap), true);
                            if (expensiveEstimate > node.h) // Node may have inherited a better estimate from its parent or from partial expansion
                                node.h = expensiveEstimate; // Not necessarily a success, we later check if the increment was large enough.

                            success = node.CompareTo(next) == 1; // node is not the smallest F anymore - re-insert it into the open list
                            if (success)
                                this.successes++;
                            else
                                this.failures++;
                        }
                        this.Ph = (1 - DynamicRationalLazyOpenList.movingAverageFactor) * this.Ph + DynamicRationalLazyOpenList.movingAverageFactor * this.successes / (this.successes + this.failures); // This is only necessary for the first runs so the Ph won't jitter too much
                    }
                    else
                        this.skips++;

                    if (success || node.g + node.h < lastF) // Never be inconsistent - don't return nodes with lower F than before. Try searching the node again.
                    {
                        this.Add(node);
                    }
                    else
                    {
                        //node.cbsState = null; // Not clearing all that memory because we might have to reopen the node.

                        // Node is still less than or equal to all items in the open list (and its h would look consistent to A*).
                        // This can be because of many reasons:
                        // - The search ended because of a timeout
                        // - The search ended because a goal was found (good!)
                        // - The search ended because the CBS search was too costly
                        break;
                    }
                }
            }
            this.lastF = node.g + node.h;
            return node;
        }

        public override void OutputStatisticsHeader(TextWriter output)
        {
            base.OutputStatisticsHeader(output);

            output.Write(this.ToString() + " successes");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " failures");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Ph");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " skips");
            output.Write(Run.RESULTS_DELIMITER);

            this.expensive.OutputStatisticsHeader(output);
        }

        public override void OutputStatistics(TextWriter output)
        {
            base.OutputStatistics(output);

            Console.WriteLine(this.ToString() + " Heuristic Function Successes: {0}", this.successes);
            Console.WriteLine(this.ToString() + " Heuristic Function Failures: {0}", this.failures);
            Console.WriteLine(this.ToString() + " Ph: {0}", this.Ph);
            Console.WriteLine(this.ToString() + " Skips: {0}", this.skips);

            output.Write(this.successes + Run.RESULTS_DELIMITER);
            output.Write(this.failures + Run.RESULTS_DELIMITER);
            output.Write(this.Ph + Run.RESULTS_DELIMITER);
            output.Write(this.skips + Run.RESULTS_DELIMITER);

            this.expensive.OutputStatistics(output);
        }

        public override int NumStatsColumns
        {
            get
            {
                return 4 + base.NumStatsColumns + this.expensive.NumStatsColumns;
            }
        }

        public override void ClearStatistics()
        {
            base.ClearStatistics();

            this.ClearPrivateStatistics();

            this.expensive.ClearStatistics();
        }

        public override void ClearAccumulatedStatistics()
        {
            base.ClearAccumulatedStatistics();

            this.accSuccesses = 0;
            this.accFailures = 0;
            this.accSkips = 0;
            this.accPh = 0;

            this.expensive.ClearAccumulatedStatistics();
        }

        public override void AccumulateStatistics()
        {
            base.AccumulateStatistics();

            this.accSuccesses += this.successes;
            this.accFailures += this.failures;
            this.accSkips += this.skips;
            this.accPh += this.Ph;

            this.expensive.AccumulateStatistics();
        }

        public override void OutputAccumulatedStatistics(TextWriter output)
        {
            base.OutputAccumulatedStatistics(output);

            Console.WriteLine(this.ToString() + " Accumulated Heuristic Function Successes: {0}", this.accSuccesses);
            Console.WriteLine(this.ToString() + " Accumulated Heuristic Function Failures: {0}", this.accFailures);
            Console.WriteLine(this.ToString() + " Accumulated Ph: {0}", this.accPh);
            Console.WriteLine(this.ToString() + " Accumulated Skips: {0}", this.accSkips);

            output.Write(this.accSuccesses + Run.RESULTS_DELIMITER);
            output.Write(this.accFailures + Run.RESULTS_DELIMITER);
            output.Write(this.accPh + Run.RESULTS_DELIMITER);
            output.Write(this.accSkips + Run.RESULTS_DELIMITER);

            this.expensive.OutputAccumulatedStatistics(output);
        }
    }
}
