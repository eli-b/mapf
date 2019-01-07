using System.Collections.Generic;
using System.Linq;
using System.IO;
using System;

namespace CPF_experiment
{
    class AStarWithPartialExpansion : ClassicAStar 
    {
        protected int expandedFullStates;
        protected int accExpandedFullStates;

        public AStarWithPartialExpansion(IHeuristicCalculator<WorldState> heuristic = null, bool mstar = false, bool mstarShuffle = false)
            : base(heuristic, mstar, mstarShuffle) { }

        override protected WorldState CreateSearchRoot(int minDepth = -1, int minCost = -1)
        {
            return new WorldStateForPartialExpansion(this.instance.m_vAgents, minDepth, minCost);
        }

        protected override WorldState CreateSearchNode(WorldState from)
        {
            return new WorldStateForPartialExpansion((WorldStateForPartialExpansion)from);
        }

        override public string GetName() { return "EPE" + base.GetName(); }

        public override void Setup(ProblemInstance problemInstance, int minDepth, Run runner, int minCost = -1, int maxCost = int.MaxValue)
        {
            base.Setup(problemInstance, minDepth, runner, minCost, maxCost);
            this.expandedFullStates = 0;
        }

        public override void Expand(WorldState nodeP)
        {
            var node = (WorldStateForPartialExpansion)nodeP;

            bool wasAlreadyExpanded = true;

            if (node.IsAlreadyExpanded() == false)
            {
                node.calcSingleAgentDeltaFs(instance, this.IsValid);
                expandedFullStates++;
                node.alreadyExpanded = true;
                wasAlreadyExpanded = false;
                //node.hBonus = 0; // Locking any hbonus that doesn't come from partial expansion
                node.targetDeltaF = 0; // Assuming a consistent heuristic (as done in the paper), the min delta F is zero.
                node.remainingDeltaF = node.targetDeltaF; // Just for the following hasChildrenForCurrentDeltaF call.
                while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() == false) // DeltaF=0 may not be possible if all agents have obstacles between their location and the goal
                {
                    node.targetDeltaF++;
                    node.remainingDeltaF = node.targetDeltaF;
                }
                if (node.hasMoreChildren() == false) // Node has no possible children at all
                {
                    node.Clear();
                    return;
                }
            }
                
            //Debug.Print("Expanding node " + node);

            // If this node was already expanded, notice its h was updated, so the deltaF refers to its original H

            base.Expand(node);

            if (node.IsAlreadyExpanded() == false)
            {
                // Node was cleared during expansion.
                // It's unnecessary and unsafe to continue to prepare it for the next partial expansion.
                return;
                // TODO: Is there a prettier way to do this?
            }

            //if (wasAlreadyExpanded)
            //{
            //    // Only doing it after expansion so that the children get the higher h
            //    node.h -= node.targetDeltaF; // This way we retain any BPMX or other h boosts, allowing the new targetDeltaF to fully add to the base h
            //    node.hBonus -= node.targetDeltaF;
            //}
            // FIXME: Why is this commented out? It was the only use of wasAlreadyExpanded, so if
            //        removing the above is correct, also remove wasAlreadyExpanded.

            node.targetDeltaF++; // This delta F was exhausted
            node.remainingDeltaF = node.targetDeltaF;

            while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() == false)
            {
                node.targetDeltaF++;
                node.remainingDeltaF = node.targetDeltaF; // Just for the following hasChildrenForCurrentDeltaF call.
            }

            if (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() && node.h + node.g + node.targetDeltaF <= this.maxSolutionCost)
            {
                // Increment H before re-insertion into open list
                //int sicEstimate = (int)SumIndividualCosts.h(node, this.instance); // Re-compute even if the heuristic used is SIC since this may be a second expansion
                //if (node.h < sicEstimate + node.targetDeltaF)
                //if (node.h < node.h + node.targetDeltaF)
                //{
                //    // Assuming the heuristic used doesn't give a lower estimate than SIC for each and every one of the node's children,
                //    // (an ok assumption since SIC is quite basic, no heuristic we use is ever worse than it)
                //    // then the current target deltaF is really exhausted, since the deltaG is always correct,
                //    // and the deltaH predicted by SIC is less than or equal to the finalDeltaH.
                //    // So if the heuristic gives the same estimate as SIC for this node
                //    // (and that mainly happens when SIC happens to give a perfect estimate),
                //    // we can increment the node's h to SIC+targetDeltaH

                //    //int newH = sicEstimate + node.targetDeltaF;
                //    int newH = node.h + node.targetDeltaF;
                //    node.hBonus += newH - node.h;
                //    node.h = newH;
                //}
                
                // Re-insert node into open list
                openList.Add(node);
                if (this.debug)
                    Console.WriteLine("Re-inserting the node into the open list with higher h");
            }
            else
                node.Clear();
        }

        protected override List<WorldState> ExpandOneAgent(List<WorldState> intermediateNodes, int agentIndex)
        {
            // Expand the agent
            List<WorldState> generated = base.ExpandOneAgent(intermediateNodes, agentIndex);
            
            // Update target F
            foreach (WorldState simpleLookingNode in generated) {
                var node = (WorldStateForPartialExpansion)simpleLookingNode;
                node.UpdateRemainingDeltaF(agentIndex);
            }

            // Prune nodes that can't get to the target F - even before their real H is calculated!
            generated = generated.Where<WorldState>(
                node => ((WorldStateForPartialExpansion)node).remainingDeltaF != ushort.MaxValue && // last move was good
                        ((WorldStateForPartialExpansion)node).hasChildrenForCurrentDeltaF(agentIndex+1)
                                                                    ).ToList<WorldState>();

            return generated;
        }

        protected override bool ProcessGeneratedNode(WorldState currentNode)
        {
            bool ret = base.ProcessGeneratedNode(currentNode);

            var node = (WorldStateForPartialExpansion)currentNode;
            node.ClearExpansionData();
            return ret;
        }

        public override void OutputStatisticsHeader(TextWriter output)
        {
            base.OutputStatisticsHeader(output);
            output.Write(this.ToString() + " Expanded Full States");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public override void OutputStatistics(TextWriter output)
        {
            base.OutputStatistics(output);

            Console.WriteLine("Expanded Full States: {0}", this.expandedFullStates);

            output.Write(this.expandedFullStates + Run.RESULTS_DELIMITER);
        }

        public override int NumStatsColumns
        {
            get
            {
                return 1 + base.NumStatsColumns;
            }
        }

        public override void ClearAccumulatedStatistics()
        {
            base.ClearAccumulatedStatistics();

            this.accExpandedFullStates = 0;
        }

        public override void AccumulateStatistics()
        {
            base.AccumulateStatistics();

            this.accExpandedFullStates += this.expandedFullStates;
        }

        public override void OutputAccumulatedStatistics(TextWriter output)
        {
            base.OutputAccumulatedStatistics(output);

            Console.WriteLine(this.ToString() + " Accumulated Total Expanded Full States (Low-Level): {0}", this.accExpandedFullStates);

            output.Write(this.accExpandedFullStates + Run.RESULTS_DELIMITER);
        }

        public override float GetEffectiveBranchingFactor()
        {
            return ((float)this.GetGenerated() - 1) / this.expandedFullStates;
        }
    }
}
