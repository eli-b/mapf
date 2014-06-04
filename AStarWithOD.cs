using System;
using System.Collections.Generic;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// A* implementation with Standley's operator decomposition (OD). See AAAI 2010 paper by Trevor Scott Standley on Cooperative Pathfinding.
    /// </summary>
    public class AStarWithOD : ClassicAStar
    {
        protected int expandedFullStates;
        protected int accExpandedFullStates;

        public AStarWithOD(HeuristicCalculator heuristic = null)
            : base(heuristic) { }

        override protected WorldState CreateSearchRoot(int minDepth = -1)
        {
            return new WorldStateWithOD(this.instance.m_vAgents, minDepth);
        }

        protected override WorldState CreateSearchNode(WorldState from)
        {
            return new WorldStateWithOD((WorldStateWithOD)from);
        }

        override public string GetName() { return "A*+OD"; }

        public override void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            base.Setup(problemInstance, minDepth, runner);
            this.expandedFullStates = 0;
        }

        protected bool alreadyExpanded;

        // Problem 59 of 3 agents and 9 obstacles is a good example of why
        // equivalence over different times is difficult to get right
        // with A*+OD (but possible).

        /// <summary>
        /// Expand a given node. This includes:
        /// - Generating all possible children
        /// - Inserting them to OPEN
        /// - Insert the generated nodes to the hashtable of nodes, currently implmented together with the closed list.
        /// </summary>
        public override void Expand(WorldState node)
        {
            if (((WorldStateWithOD)node).agentTurn == 0)
                expandedFullStates++;
            this.alreadyExpanded = false;
            base.Expand(node);
        }

        protected override List<WorldState> ExpandOneAgent(List<WorldState> intermediateNodes, int agentIndex)
        {
            if (this.alreadyExpanded == true)  // Necessary because after expansion, the generated nodes have an incremented agentTurn that once again equals agentIndex
                                               // and because it's possible that a node may have valid children that aren't already in the closed list
                return intermediateNodes; // Do nothing to this agent

            WorldStateWithOD parent = (WorldStateWithOD)intermediateNodes[0];

            if (agentIndex < parent.agentTurn)
                return intermediateNodes; // Do nothing to this agent

            var generated = base.ExpandOneAgent(intermediateNodes, agentIndex);

            int childAgentTurn = ((parent.agentTurn + 1) % (this.instance.m_vAgents.Length));
            foreach (var node in generated)
            {
                WorldStateWithOD childNode = (WorldStateWithOD)node;

                childNode.agentTurn = childAgentTurn;

                // Makespan increases only if this is the move of the first agent
                if (parent.agentTurn != 0)
                    childNode.makespan--; // Cancel the increment in base
            }

            this.alreadyExpanded = true;
            return generated;
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

            Console.WriteLine("Total Expanded Full States: {0}", this.expandedFullStates);

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

            Console.WriteLine("Total Expanded Full States (Low-Level): {0}", this.accExpandedFullStates);

            output.Write(this.accExpandedFullStates + Run.RESULTS_DELIMITER);
        }
    }
}
