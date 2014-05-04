using System.Collections.Generic;
using System.Linq;
using System.IO;
using System;

namespace CPF_experiment
{
    class AStarWithPartialExpansion : ClassicAStar 
    {
        protected int expandedFullStates;

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

            if (node.isAlreadyExpanded() == false)
            {
                node.calcSingleAgentDeltaFs(instance);
                expandedFullStates++;
                node.alreadyExpanded = true;
                node.targetDeltaF = 0; // Assuming a consistent heuristic (as done in the paper), the min delta F is zero.
                node.remainingDeltaF = node.targetDeltaF; // Just for the hasChildrenForCurrentDeltaF call.
                while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() == false) // DeltaF=0 may not be possible if all nodes have obstacles between their location and the goal
                    node.targetDeltaF++;
            }
            //Debug.Print("Expanding node " + node);

            node.remainingDeltaF = node.targetDeltaF;

            base.Expand(node);

            node.targetDeltaF++;

            while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() == false)
                node.targetDeltaF++;

            if (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() && node.h + node.g + node.targetDeltaF <= this.maxCost)
                openList.Add(node);
            return;
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
