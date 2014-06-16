using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace CPF_experiment
{
    public class DynamicRationalLazyOpenList : OpenList
    {
        public LazyHeuristic expensive;
        protected int lastF;
        protected int skips;
        protected int accSkips;
        protected double[][] capData;
        protected double[][] accCapData;
        protected static readonly double MOVING_AVERAGE_FACTOR = 1;
        protected static readonly int SUCCESS_IND = 0;
        protected static readonly int FAILURE_IND = 1;
        protected static readonly int PH_IND = 2;
        protected static readonly int NUM_CAPS = 27;
        private double expandStartTime;
        protected double sumExpandTimes;
        protected int numExpands;
        protected double accSumExpandTimes;
        protected int accNumExpands;
        /// <summary>
        /// Probability that the expensive heuristic is helpful
        /// </summary>

        public DynamicRationalLazyOpenList(ISolver user, LazyHeuristic expensive)
            : base(user)
        {
            this.expensive = expensive;
            this.ClearPrivateStatistics();
        }

        protected void ClearPrivateStatistics()
        {
            this.lastF = -1;

            this.numExpands = 0;
            this.sumExpandTimes = 0;
            this.skips = 0;
            this.capData = new double[DynamicRationalLazyOpenList.NUM_CAPS][];
            this.accCapData = new double[DynamicRationalLazyOpenList.NUM_CAPS][];
            for (int i = 0; i < DynamicRationalLazyOpenList.NUM_CAPS; ++i)
            {
                this.capData[i] = new double[3] { 0, 0, 0 };
                this.accCapData[i] = new double[3] { 0, 0, 0 };
            }
        }

        public override string ToString()
        {
            return "DynamicRationalLazyOpenList(alpha=" + DynamicRationalLazyOpenList.MOVING_AVERAGE_FACTOR + 
                   " num_caps=" + DynamicRationalLazyOpenList.NUM_CAPS + ")/" + this.expensive;
        }

        public override IBinaryHeapItem Remove()
        {
            WorldState node;

            if (this.lastF != -1)
            {
                this.numExpands++;
                double expandFinishTime = ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds();
                this.sumExpandTimes += expandFinishTime - this.expandStartTime;
            }

            if (base.Count == 1) // Can happen more than once with the same node if partial expansion pushes it back into the open list, leading to multiple zero-timeout runs.
            {
                node = (WorldState)base.Remove();
                if (node.GoalTest() == true || // Can't improve the h of the goal
                    ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds() > Constants.MAX_TIME) // No time to continue improving H. // FIXME: Hack!
                    goto finish;
                int expensiveEstimate = (int)this.expensive.h(node, int.MaxValue, -1, 0, false); // I'm willing to pay the overhead to instantly solve problems of depth zero.
                node.h = Math.Max(node.h, expensiveEstimate);
                goto finish;
            }

            // There are alternatives to the lowest cost node in the open list, try to postpone expansion of it:
            float branchingFactor = ((ClassicAStar)this.user).GetEffectiveBranchingFactor(); // We know the solver is an A* variant.
            const double binaryHeapTau = 0.073359375; // microseconds. From empirical experiments with this infra on my computer.
            double logN = Math.Log(this.heap.Count, 2); // Removals from the queue cost practically zero.
            double t0 = binaryHeapTau * logN;
            double overhead = 0.023 * ((WorldState)this.Peek()).allAgentsState.Length; // in milliseconds. Empirical lowest estimate. The cost of a zero-timeout CBSH run wasn't simply linear with the number of agents for some reason.

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
                // DRLA* calculation (derived from Tolpin et al. RLA* formula):
                double tExpand = (this.sumExpandTimes / this.numExpands) * 1000; // in microseconds
                double[] expectedRegret = new double[DynamicRationalLazyOpenList.NUM_CAPS]; // in microseconds
                double lowestRegret = double.MaxValue;
                int selectedCapExponent = -1;

                //double tempStartTime = ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds();

                // Compute the expected regret from choosing each cap, and finding cap that gives the lowest expected regret
                for (int i=0 ; i < DynamicRationalLazyOpenList.NUM_CAPS ; ++i)
                {
                    expectedRegret[i] = 0;
                    double cap = 0; // To quiet the compiler
                    for (int j=0 ; j <= i; ++j)
                    {
                        cap = 1 << j; // microseconds
                        double pThatThisCapWouldBeTheFirstCapToSucceed;
                        if (j != 0)
                            pThatThisCapWouldBeTheFirstCapToSucceed = this.capData[j][DynamicRationalLazyOpenList.PH_IND] - this.capData[j - 1][DynamicRationalLazyOpenList.PH_IND]; // The PH_IND points to the probability that the given cap would succeed at all, including the probability that it would've worked for lower caps.
                        else
                            pThatThisCapWouldBeTheFirstCapToSucceed = this.capData[j][DynamicRationalLazyOpenList.PH_IND];
                        expectedRegret[i] += (0.75 * cap/*cap*/ + t0 - Math.Min(0.75 * cap/*cap*/ + t0, tExpand)) * pThatThisCapWouldBeTheFirstCapToSucceed;  // The cap is an upper bound on t2. A closer estimate would be 0.75*cap (since 0.5*cap is the previous cap).
                    }
                    for (int j=i+1 ; j < DynamicRationalLazyOpenList.NUM_CAPS; ++j)
                    {
                        double heuristicOnlyHelpfulAfter = 1 << j; // microseconds
                        double pThatThisCapWouldBeTheFirstCapToSucceed = this.capData[j][DynamicRationalLazyOpenList.PH_IND] - this.capData[j - 1][DynamicRationalLazyOpenList.PH_IND];
                        expectedRegret[i] += (cap + tExpand - Math.Min(0.75 * heuristicOnlyHelpfulAfter/*heuristicOnlyHelpfulAfter*/ + t0, tExpand)) * pThatThisCapWouldBeTheFirstCapToSucceed;
                    }
                    // The case where even the highest cap fails:
                    int last = DynamicRationalLazyOpenList.NUM_CAPS - 1;
                    expectedRegret[i] += (1 << last) * (1 - this.capData[last][DynamicRationalLazyOpenList.PH_IND]);

                    if (expectedRegret[i] < lowestRegret)
                    {
                        lowestRegret = expectedRegret[i];
                        selectedCapExponent = i;
                    }
                }

                // Force the heuristic to be tried if it's never been tried before at a cap that's one above Texpand.
                // This is needed because we initially believe all caps are useless, so the caps that are too short won't be tried
                // and fail. This has the added benefit that after running, we'll have an informed belief for all lower caps and possibly
                // all higher caps too.
                int nextCapAboveTExpand = (int)Math.Ceiling(Math.Log(tExpand, 2));
                if (this.capData[nextCapAboveTExpand][DynamicRationalLazyOpenList.SUCCESS_IND] +
                    this.capData[nextCapAboveTExpand][DynamicRationalLazyOpenList.FAILURE_IND] == 0) // This cap was never tried before, and it might be good.
                {
                    selectedCapExponent = nextCapAboveTExpand;
                } // TODO: Put the entire matrix calculation above into the else of this clause.

                double millisCap = ((double)(1 << selectedCapExponent)) / 1000;

                //double tempFinishTime = ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds();
                //Console.WriteLine("calculations time:{0}", tempFinishTime - tempStartTime);
                //Console.WriteLine("millisCap:{0}", millisCap);
                //Console.WriteLine("Texpand:{0}={1}/{2}", tExpand, this.sumExpandTimes * 1000, this.numExpands);
                //Console.WriteLine("T0:{0}", t0);
                //for (int i=0 ; i<DynamicRationalLazyOpenList.NUM_CAPS ; ++i)
                    //Console.WriteLine("Ph for cap {0}:{1}={2}/{3}", ((double)(1 << i)) / 1000, this.capData[i][2], this.capData[i][0], this.capData[i][0] + this.capData[i][1]);
                //for (int i = 0; i < DynamicRationalLazyOpenList.NUM_CAPS; ++i)
                    //Console.WriteLine("Expected regret for cap {0}:{1}", ((double)(1 << i)) / 1000, expectedRegret[i]/1000.0);

                bool success = false;
                if (millisCap > overhead || // Worth running the expensive heuristic
                    node.g + node.h < lastF) // Must improve the heuristic estimate to be consistent
                {
                    double expensiveCallStartTime = ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds();
                    int expensiveEstimate = (int)this.expensive.h(node, targetH, -1, (int)(expensiveCallStartTime + millisCap), false);
                    double expensiveCallTotalTime = ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds() - expensiveCallStartTime;

                    if (expensiveEstimate > node.h) // Node may have inherited a better estimate from its parent so this check is necessary for failues
                        node.h = expensiveEstimate; // If this wasn't a success, the assignment here serves to force the next heuristic call
                                                    // to search deeper, since we're always consistent.

                    success = node.CompareTo(next) == 1; // Node is not the smallest F anymore - re-insert it into the open list
                    if (success || node.GoalTest())
                    {
                        int lowestCapThatWouldHaveWorked = (int)Math.Ceiling(Math.Log(expensiveCallTotalTime * 1000, 2));
                        //Console.WriteLine("Lowest cap that would have worked:{0}", (1<<lowestCapThatWouldHaveWorked)/1000.0);
                        for (int j = 0; j < lowestCapThatWouldHaveWorked; ++j)
                        {
                            this.capData[j][DynamicRationalLazyOpenList.FAILURE_IND] += 1;
                            this.capData[j][DynamicRationalLazyOpenList.PH_IND] = (1 - DynamicRationalLazyOpenList.MOVING_AVERAGE_FACTOR) * this.capData[j][DynamicRationalLazyOpenList.PH_IND] +
                                                                                  DynamicRationalLazyOpenList.MOVING_AVERAGE_FACTOR * this.capData[j][DynamicRationalLazyOpenList.SUCCESS_IND] / (this.capData[j][DynamicRationalLazyOpenList.SUCCESS_IND] + this.capData[j][DynamicRationalLazyOpenList.FAILURE_IND]);
                        }
                        for (int j = lowestCapThatWouldHaveWorked; j < DynamicRationalLazyOpenList.NUM_CAPS; ++j)
                        {
                            this.capData[j][DynamicRationalLazyOpenList.SUCCESS_IND] += 1;
                            this.capData[j][DynamicRationalLazyOpenList.PH_IND] = (1 - DynamicRationalLazyOpenList.MOVING_AVERAGE_FACTOR) * this.capData[j][DynamicRationalLazyOpenList.PH_IND] +
                                                                                  DynamicRationalLazyOpenList.MOVING_AVERAGE_FACTOR * this.capData[j][DynamicRationalLazyOpenList.SUCCESS_IND] / (this.capData[j][DynamicRationalLazyOpenList.SUCCESS_IND] + this.capData[j][DynamicRationalLazyOpenList.FAILURE_IND]);
                        }
                    }
                    else
                    {
                        for (int j = 0 ; j <= selectedCapExponent ; ++j)
                        {
                            this.capData[j][DynamicRationalLazyOpenList.FAILURE_IND] += 1;
                            this.capData[j][DynamicRationalLazyOpenList.PH_IND] = (1 - DynamicRationalLazyOpenList.MOVING_AVERAGE_FACTOR) * this.capData[j][DynamicRationalLazyOpenList.PH_IND] +
                                                                                  DynamicRationalLazyOpenList.MOVING_AVERAGE_FACTOR * this.capData[j][DynamicRationalLazyOpenList.SUCCESS_IND] / (this.capData[j][DynamicRationalLazyOpenList.SUCCESS_IND] + this.capData[j][DynamicRationalLazyOpenList.FAILURE_IND]); // This is only necessary for the first runs so the Ph won't jitter too much.
                        }
                        // No info on whether a larger cap would have worked.
                    }
                }
                else
                    this.skips++;

                if (success || node.g + node.h < lastF) // Never be inconsistent - don't return nodes with lower F than before. Try searching the node again.
                {
                    this.Add(node);
                }
                else
                {
                    // Node is still less than or equal to all items in the open list (and its h would look consistent to A*).
                    // This can be because of many reasons:
                    // - The search ended because of a timeout
                    // - The search ended because a goal was found (good!)
                    // - The search ended because the CBS search was too costly
                    break;
                }
            }

            finish:
            this.lastF = node.g + node.h;
            this.expandStartTime = ((DyanamicLazyCbsh)this.expensive).runner.ElapsedMilliseconds();
            return node;
        }

        public override void OutputStatisticsHeader(TextWriter output)
        {
            base.OutputStatisticsHeader(output);

            output.Write(this.ToString() + " Skips");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Average Texpand (ms)");
            output.Write(Run.RESULTS_DELIMITER);

            this.expensive.OutputStatisticsHeader(output);
        }

        public override void OutputStatistics(TextWriter output)
        {
            base.OutputStatistics(output);

            double Texpand;
            if (this.numExpands != 0)
                Texpand = this.sumExpandTimes / this.numExpands;
            else
                Texpand = -1;

            Console.WriteLine(this.ToString() + " Skips: {0}", this.skips);
            Console.WriteLine(this.ToString() + " Average Texpand (ms): {0}", Texpand);

            output.Write(this.skips + Run.RESULTS_DELIMITER);
            output.Write(Texpand + Run.RESULTS_DELIMITER);

            this.expensive.OutputStatistics(output);
        }

        public override int NumStatsColumns
        {
            get
            {
                return 2 + base.NumStatsColumns + this.expensive.NumStatsColumns;
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

            this.accSkips = 0;

            this.expensive.ClearAccumulatedStatistics();
        }

        public override void AccumulateStatistics()
        {
            base.AccumulateStatistics();

            this.accSkips += this.skips;
            this.accNumExpands += this.numExpands;
            this.accSumExpandTimes += this.sumExpandTimes;

            this.expensive.AccumulateStatistics();
        }

        public override void OutputAccumulatedStatistics(TextWriter output)
        {
            base.OutputAccumulatedStatistics(output);

            double accTexpand;
            if (this.numExpands != 0)
                accTexpand = this.accSumExpandTimes / this.accNumExpands;
            else
                accTexpand = -1;

            Console.WriteLine(this.ToString() + " Accumulated Skips: {0}", this.accSkips);
            Console.WriteLine(this.ToString() + " Accumulated Texpand (ms): {0}", accTexpand);

            output.Write(this.accSkips + Run.RESULTS_DELIMITER);
            output.Write(accTexpand + Run.RESULTS_DELIMITER);

            this.expensive.OutputAccumulatedStatistics(output);
        }
    }
}
