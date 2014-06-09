using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    public class DynamicLazyOpenList : OpenList
    {
        public LazyHeuristic expensive;
        protected int lastF;

        public DynamicLazyOpenList(ISolver user, LazyHeuristic expensive)
            : base(user)
        {
            this.expensive = expensive;
        }

        public override string ToString()
        {
            return "DynamicLazyOpenList/" + this.expensive;
        }

        public override IBinaryHeapItem Remove()
        {
            WorldState node;
            if (base.Count < 2)
            {
                node = (WorldState)base.Remove(); // Throws if Count == 0
                this.lastF = node.g + node.h;
                return node;
            }

            // There are alternatives to the lowest cost node in the open list, try to postpone expansion of it:
            float branchingFactor = ((float)this.user.GetGenerated() - 1) / this.user.GetExpanded();
            if (this.user is ClassicAStar) // HACK!! Just until I copy this method to the interface and implement it everywhere.
                branchingFactor = ((ClassicAStar)this.user).GetEffectiveBranchingFactor();

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
                // No matter if we tried reaching the same targetH before.
                // We can actually improve the estimate again if the search hit the node generation thershold last time.
                // The only node we can't improve the estimate for is a (generalized) goal node, and that's handled earlier
                int expensiveEstimate = (int)this.expensive.h(node, targetH, branchingFactor);
                node.h = Math.Max(node.h, expensiveEstimate); // Node may have inherited a better estimate from its parent
                
                if (node.CompareTo(next) == 1 || // node is not the smallest F anymore - re-insert into open list
                    node.g + node.h < lastF) // Never be inconsistent - don't return nodes with lower F than before. Try searching the node again.
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
            this.lastF = node.g + node.h;
            return node;
        }

        public override void OutputStatisticsHeader(TextWriter output)
        {
            base.OutputStatisticsHeader(output);

            this.expensive.OutputStatisticsHeader(output);
        }

        public override void OutputStatistics(TextWriter output)
        {
            base.OutputStatistics(output);

            this.expensive.OutputStatistics(output);
        }

        public override int NumStatsColumns
        {
            get
            {
                return base.NumStatsColumns + this.expensive.NumStatsColumns;
            }
        }

        public override void ClearStatistics()
        {
            base.ClearStatistics();

            this.expensive.ClearStatistics();
        }

        public override void ClearAccumulatedStatistics()
        {
            base.ClearAccumulatedStatistics();

            this.expensive.ClearAccumulatedStatistics();
        }

        public override void AccumulateStatistics()
        {
            base.AccumulateStatistics();

            this.expensive.AccumulateStatistics();
        }

        public override void OutputAccumulatedStatistics(TextWriter output)
        {
            base.OutputAccumulatedStatistics(output);

            this.expensive.OutputAccumulatedStatistics(output);
        }
    }
}
