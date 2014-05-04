using System.Collections.Generic;
using System.Linq;
using System.IO;
using System;

namespace CPF_experiment
{
    class AStarWithPartialExpansion : ClassicAStar 
    {
        public AStarWithPartialExpansion(HeuristicCalculator heuristic = null)
            : base(heuristic) { }

        override protected WorldState CreateSearchRoot()
        {
            WorldStateForPartialExpansion root = new WorldStateForPartialExpansion(this.instance.m_vAgents);
            return root;
        }

        protected override WorldState CreateSearchNode(WorldState from)
        {
            return new WorldStateForPartialExpansion((WorldStateForPartialExpansion)from);
        }

        override public string GetName() { return "EPEA*"; }

        public override void Expand(WorldState nodeP)
        {
            var node = (WorldStateForPartialExpansion)nodeP;

            node.calcSingleAgentDeltaFs(instance);

            if (node.isAlreadyExpanded() == false)
            {
                expandedFullStates++;
                node.alreadyExpanded = true;
                node.targetDeltaF = 0; // Assuming a consistent heuristic (as done in the paper), the min delta F is zero.
            }
            //Debug.Print("Expanding node " + node);

            node.remainingDeltaF = node.targetDeltaF;

            base.Expand(node);

            node.targetDeltaF++;

            while (node.hasMoreChildren() && node.hasChildrenForCurrentF() == false)
                node.targetDeltaF++;

            if (node.hasMoreChildren() && node.hasChildrenForCurrentF() && node.h + node.g + node.targetDeltaF <= this.maxCost)
                openList.Add(node);
            return;
        }

        protected override List<WorldState> ExpandOneAgent(List<WorldState> intermediateNodes, int agentIndex)
        {
            // Prune nodes that can't get to the target F
            foreach (WorldState node in intermediateNodes)
                ((WorldStateForPartialExpansion)node).calcSingleAgentDeltaFs(this.instance);
            intermediateNodes = intermediateNodes.Where<WorldState>(
                node => ((WorldStateForPartialExpansion)node).remainingDeltaF != byte.MaxValue && // last move was good
                        ((WorldStateForPartialExpansion)node).hasChildrenForCurrentF(agentIndex)
                                                                    ).ToList<WorldState>();
            // Expand the agent
            List<WorldState> generated = base.ExpandOneAgent(intermediateNodes, agentIndex);
            
            // Update target F
            foreach (WorldState simpleLookingNode in generated) {
                var node = (WorldStateForPartialExpansion)simpleLookingNode;
                node.UpdateRemainingFChange(agentIndex);
            }

            return generated;
        }

        protected override bool ProcessGeneratedNode(WorldState simpleLookingNode)
        {
            var currentNode = (WorldStateForPartialExpansion)simpleLookingNode;
            if (currentNode.remainingDeltaF != 0) // No more moves to do, remaining delta F must be zero
                return false;

 	        return base.ProcessGeneratedNode(currentNode);
        }

        public override void OutputStatisticsHeader(TextWriter output)
        {
            base.OutputStatisticsHeader(output);
            output.Write(this.ToString() + " Expanded Full States (LL)");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public override void OutputStatistics(TextWriter output)
        {
            base.OutputStatistics(output);

            Console.WriteLine("Expanded Full States (Low-Level): {0}", this.expandedFullStates);

            output.Write(this.expandedFullStates + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 1 + base.NumStatsColumns;
            }
        }
    }
}
