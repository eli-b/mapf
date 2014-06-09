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

        public AStarWithPartialExpansion(HeuristicCalculator heuristic = null)
            : base(heuristic) { }

        override protected WorldState CreateSearchRoot(int minDepth = -1)
        {
            WorldStateForPartialExpansion root = new WorldStateForPartialExpansion(this.instance.m_vAgents, minDepth);
            return root;
        }

        protected override WorldState CreateSearchNode(WorldState from)
        {
            return new WorldStateForPartialExpansion((WorldStateForPartialExpansion)from);
        }

        override public string GetName() { return "EPEA*"; }

        public override void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            base.Setup(problemInstance, minDepth, runner);
            this.expandedFullStates = 0;
        }

        public override void Expand(WorldState nodeP)
        {
            var node = (WorldStateForPartialExpansion)nodeP;

            if (node.isAlreadyExpanded() == false)
            {
                node.calcSingleAgentDeltaFs(instance, this.IsValid);
                expandedFullStates++;
                node.alreadyExpanded = true;
                node.targetDeltaF = 0; // Assuming a consistent heuristic (as done in the paper), the min delta F is zero.
                node.remainingDeltaF = node.targetDeltaF; // Just for the hasChildrenForCurrentDeltaF call.
                while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() == false) // DeltaF=0 may not be possible if all agents have obstacles between their location and the goal
                    node.targetDeltaF++;
            }
            //Debug.Print("Expanding node " + node);

            // If this node was already expanded, notice its h was updated, so the deltaF refers to its original H

            node.remainingDeltaF = node.targetDeltaF;

            base.Expand(node);

            node.targetDeltaF++;

            while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() == false)
                node.targetDeltaF++;

            if (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() && node.h + node.g + node.targetDeltaF <= this.maxCost)
            {
                // Increment H before re-insertion into open list
                int sicEstimate = (int)SumIndividualCosts.h(node, this.instance); // Re-compute even if the heuristic used is SIC since this may be a second expansion
                if (node.h < sicEstimate + node.targetDeltaF)
                {
                    // Assuming the heuristic used doesn't give a lower estimate than SIC for each and every one of the node's children,
                    // (an ok assumption since SIC is quite basic, no heuristic we use is ever worse than it)
                    // then the current target deltaF is really exhausted, since the deltaG is always correct,
                    // and the deltaH predicted by SIC is less than or equal to the finalDeltaH.
                    // So if the heuristic gives the same estimate as SIC for this node
                    // (and that mainly happens when SIC happens to give a perfect estimate),
                    // we can increment the node's h to SIC+targetDeltaH
                    node.h = sicEstimate + node.targetDeltaF;
                }
                
                // Re-insert node into open list
                openList.Add(node);
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
                node => ((WorldStateForPartialExpansion)node).remainingDeltaF != byte.MaxValue && // last move was good
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
