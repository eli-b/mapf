using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    /// <summary>
    /// This open list invokes an expensive-to-compute heuristic on items before they're removed,
    /// potentially pushing them back into itself if their F value increased above the next item in
    /// the list.
    /// The heuristic is given the "target value" - the minimum estimate that would cause an item to
    /// be pushed back, and is allowed to stop once this estimate was reached.
    /// </summary>
    /// <typeparam name="Item"></typeparam>
    public class DynamicLazyOpenList<Item> : OpenList<Item> where Item: IBinaryHeapItem, IHeuristicSearchNode
    {
        public ILazyHeuristic<Item> expensive;
        public Run runner;
        protected int lastF;
        protected int nodesPushedBack;
        protected int accNodesPushedBack;
        public bool debug;

        public DynamicLazyOpenList(ISolver user, ILazyHeuristic<Item> expensive)
            : base(user)
        {
            this.expensive = expensive;
            this.ClearStatistics();
            this.accNodesPushedBack = 0;
            this.debug = false;
        }

        public override string GetName()
        {
            return $"Dynamic Lazy Open List with Heuristic {this.expensive.GetName()}";
        }

        public override string ToString()
        {
            return $"DynamicLazyOpenList/{this.expensive}";
        }

        public override Item Remove()
        {
            Item node;
            if (base.Count < 2)
            {
                // No need to run the expensive heuristic - it can't push back a node over another.
                Debug.WriteLine("Fewer than 2 nodes in the open list - not applying the heuristic");
                node = base.Remove(); // Throws if Count == 0
                this.lastF = node.f;
                return node;
            }
            // There are alternatives to the lowest cost node in the open list, try to postpone expansion of it:

            while (true)
            {
                node = base.Remove();

                if (node.GoalTest() == true || // Can't improve the h of the goal
                    node.hBonus > 0 || // Already computed the expensive heuristic
                    this.runner.ElapsedMilliseconds() > Constants.MAX_TIME) // No time to continue improving H.
                    break;

                var next = base.Peek();
                int targetH = node.GetTargetH(next.f + 1);  // Don't assume f = g + h (but do assume integer costs)
                int expensiveEstimate = (int)this.expensive.h(node, targetH);
                if (node.h < expensiveEstimate) // Node may have inherited a better estimate from its parent
                {
                    node.hBonus += expensiveEstimate - node.h;
                    node.h = expensiveEstimate;
                }
                
                if (node.CompareTo(next) == 1) // node is not the smallest F anymore - re-insert into open list
                {
                    this.Add(node);
                    this.nodesPushedBack++;
                    if (this.debug)
                        Debug.Print("Pushing back the node into the open list with an increased h.");
                }
                else
                {
                    // Node is still less than or equal to all items in the open list (and its h would look consistent to A*).
                    // This can be because of many reasons:
                    // - The heuristic couldn't increase the node's estimate enough
                    //    - Sometimes that's because a goal was found!
                    // - Computing the heuristic ended because of a timeout
                    // - Computing the heuristic ended because it was otherwise too costly
                    break;
                }
            }
            this.lastF = node.f;
            return node;
        }

        public override void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Nodes Pushed Back");
            output.Write(Run.RESULTS_DELIMITER);

            base.OutputStatisticsHeader(output);

            this.expensive.OutputStatisticsHeader(output);
        }

        public override void OutputStatistics(TextWriter output)
        {
            Console.WriteLine($"Nodes Pushed Back: {this.nodesPushedBack}");

            output.Write(this.nodesPushedBack + Run.RESULTS_DELIMITER);

            base.OutputStatistics(output);

            this.expensive.OutputStatistics(output);
        }

        public override int NumStatsColumns
        {
            get
            {
                return base.NumStatsColumns + this.expensive.NumStatsColumns + 1;
            }
        }

        public override void ClearStatistics()
        {
            base.ClearStatistics();

            this.expensive.ClearStatistics();

            this.nodesPushedBack = 0;
        }

        public override void ClearAccumulatedStatistics()
        {
            base.ClearAccumulatedStatistics();

            this.expensive.ClearAccumulatedStatistics();

            this.accNodesPushedBack = 0;
        }

        public override void AccumulateStatistics()
        {
            base.AccumulateStatistics();

            this.expensive.AccumulateStatistics();

            this.accNodesPushedBack += this.nodesPushedBack;
        }

        public override void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine($"{this} Accumulated Nodes Pushed Back: {this.accNodesPushedBack}");

            output.Write(this.accNodesPushedBack + Run.RESULTS_DELIMITER);

            base.OutputAccumulatedStatistics(output);

            this.expensive.OutputAccumulatedStatistics(output);
        }
    }
}
