using System.Collections.Generic;
using System.Linq;
using System.IO;

namespace CPF_experiment
{
    /// <summary>
    /// Only keeps generated nodes that have the same f as the parent.
    /// The parent is then re-inserted with the lowest f from its discarded children
    /// (Yoshizumi, Miura, and Ishida 2000)
    /// </summary>
    class AStarWithPartialExpansionBasic : ClassicAStar
    {
        int generatedAndDiscarded;
        bool hasMoreSuccessors;
        /// <summary>
        /// The lowest discarded F
        /// </summary>
        public int nextFvalue;
        public int currentFTarget;

        public AStarWithPartialExpansionBasic(HeuristicCalculator heuristic = null)
            : base(heuristic) { }

        override protected WorldState CreateSearchRoot()
        {
            return new WorldStateForPartialExpansion(this.instance.m_vAgents); // Consider using a WorldStateForBasicPartialExpansion that only has the isAlreadyExpanded stuff
        }

        public override void Setup(ProblemInstance problemInstance, Run runner) 
        { 
            base.Setup(problemInstance, runner);
            generatedAndDiscarded = 0;
        }

        override public string GetName() { return "(B)PEA*"; }

        protected override WorldState CreateSearchNode(WorldState from)
        {
            return new WorldStateForPartialExpansion((WorldStateForPartialExpansion)from);
        }

        override public void Expand(WorldState simpleLookingNode)
        {
            var node = (WorldStateForPartialExpansion)simpleLookingNode;
            //Debug.Print("Expanding node " + node);
            if (!node.isAlreadyExpanded())
            {
                node.alreadyExpanded = true;
                this.expandedFullStates++;
            }

            hasMoreSuccessors = false;
            this.nextFvalue = int.MaxValue;
            this.currentFTarget = node.g + node.h;

            base.Expand(node);


            if (hasMoreSuccessors && this.nextFvalue <= this.maxCost)
            {
                node.h = this.nextFvalue - node.g; // Just to update this node's f value to the desired value.
                                                   // Although you could say that since we exhausted the current F value, if we get to this node again it means the heuristic was off by at least 1
                this.openList.Add(node); // Re-insert to open list with updated F
            }
        }

        /// <summary>
        /// Adds nodes of target F only
        /// </summary>
        /// <param name="currentNode"></param>
        /// <param name="parentNode"></param>
        /// <returns></returns>
        protected override bool ProcessGeneratedNode(WorldState currentNode)
        {
            if (currentNode.h + currentNode.g == this.currentFTarget)
                return base.ProcessGeneratedNode(currentNode);
            else generatedAndDiscarded++; // Notice we don't count the discarded nodes in the genereted count, only here

            if (currentNode.h + currentNode.g > this.currentFTarget)
            {
                this.hasMoreSuccessors = true;
                if (currentNode.h + currentNode.g < this.nextFvalue)
                    this.nextFvalue = (byte)(currentNode.h + currentNode.g);
            }
            return false;
        }

        public override void OutputStatistics(TextWriter output)
        {
            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write(this.generatedAndDiscarded + Run.RESULTS_DELIMITER);
            output.Write(solutionDepth + Run.RESULTS_DELIMITER);
            output.Write(expandedFullStates + Run.RESULTS_DELIMITER);
            output.Write("NA"/*Process.GetCurrentProcess().VirtualMemorySize64*/ + Run.RESULTS_DELIMITER);
            // Isn't there a CSV module in C# instead of fussing with the delimeter everywhere?
        }
    }
    

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

            node.calcSingleAgentFDifferences(instance);

            if (node.isAlreadyExpanded() == false)
            {
                expandedFullStates++;
                node.alreadyExpanded = true;
                node.targetFChange = 0; // Assuming a consistent heuristic (as done in the paper), the min F Change is zero.
            }
            //Debug.Print("Expanding node " + node);

            node.remainingFChange = node.targetFChange;

            base.Expand(node);

            node.targetFChange++;

            while (node.hasMoreChildren() && node.hasChildrenForCurrentF() == false)
                node.targetFChange++;

            if (node.hasMoreChildren() && node.hasChildrenForCurrentF() && node.h + node.g + node.targetFChange <= this.maxCost)
                openList.Add(node);
            return;
        }

        protected override List<WorldState> ExpandOneAgent(List<WorldState> intermediateNodes, int agentIndex)
        {
            // Prune nodes that can't get to the target F
            foreach (WorldState node in intermediateNodes)
                ((WorldStateForPartialExpansion)node).calcSingleAgentFDifferences(this.instance);
            intermediateNodes = intermediateNodes.Where<WorldState>(
                node => ((WorldStateForPartialExpansion)node).remainingFChange != byte.MaxValue && // last move was good
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
            if (currentNode.remainingFChange != 0) // No more moves to do, remaining F change must be zero
                return false;



 	        return base.ProcessGeneratedNode(currentNode);
        }
    }
}
