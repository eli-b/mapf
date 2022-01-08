using System.Collections.Generic;
using System.Linq;
using System.IO;
using System;
using System.Diagnostics;

namespace mapf;

class EPEA_Star : A_Star 
{
    protected int expandedFullStates;
    protected int accExpandedFullStates;

    public EPEA_Star(IHeuristicCalculator<WorldState> heuristic = null, bool mstar = false,
        bool mstarShuffle = false)
        : base(heuristic, mstar, mstarShuffle)
    {
        if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
            Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS
            )
        {
            throw new NotImplementedException("Makespan support isn't implemented at the moment. Use A*+OD for now.");
        }
    }

    override protected WorldState CreateSearchRoot(int minDepth = -1, int minCost = -1, MDDNode mddNode = null)
    {
        var root =  new WorldStateForPartialExpansion(this.instance.agents, minDepth, minCost, mddNode);
        root.sic = (int)SumIndividualCosts.h(root, this.instance);
        return root;
    }

    protected override WorldState CreateSearchNode(WorldState from)
    {
        return new WorldStateForPartialExpansion((WorldStateForPartialExpansion)from);
    }

    override public string GetName() { return "EPE" + base.GetName(); }

    public override void Setup(ProblemInstance problemInstance, int minDepth, Run runner,
                                ConflictAvoidanceTable CAT = null,
                                ISet<CbsConstraint> constraints = null, ISet<CbsConstraint> positiveConstraints = null,
                                int minCost = -1, int maxCost = int.MaxValue, MDD mdd = null)
    {
        base.Setup(problemInstance, minDepth, runner, CAT, constraints, positiveConstraints,
                    minCost, maxCost, mdd);
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
        // If this node was already expanded, notice its h was updated, so the deltaF refers to its original H

        do
        {
            if (debug)
                Debug.WriteLine($"Generating children with target deltaF={node.targetDeltaF}. Actual delta F may be lower because the parent node may have various H boosts.");
            base.Expand(node);
            // base.Expand computes every child's h value from scratch, even though we could compute it incrementally from intermediate nodes

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

            // This node's target delta F was exhausted - increment it until a target delta F with actual children is found
            do
            {
                node.targetDeltaF++;
                node.remainingDeltaF = node.targetDeltaF; // Just for the following hasChildrenForCurrentDeltaF call.
            } while (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() == false);
        } while (node.hasMoreChildren() && node.g + node.sic + node.targetDeltaF <= node.minGoalCost);  // Generate more children immediately if we have a lower bound on the solution depth

        if (node.hasMoreChildren() && node.hasChildrenForCurrentDeltaF() && node.g + node.sic + node.targetDeltaF <= this.maxSolutionCost)
        {
            // Assuming the heuristic used doesn't give a lower estimate than SIC for each and every one of the node's children,
            // (an ok assumption since SIC is quite basic, no heuristic we use is ever worse than it)
            // then the current target deltaF is really exhausted, since the deltaG is always correct,
            // and the deltaH predicted by SIC is less than or equal to the finalDeltaH.
            // So if the heuristic gives the same estimate as SIC for this node
            // (and that mainly happens when SIC happens to give a perfect estimate),
            // we can increment the node's f to g+SIC+targetDeltaH

            // Re-insert node into open list
            openList.Add(node);
            if (this.debug)
            {
                Debug.WriteLine($"Re-inserting node {node.generated} into the open list (with targetDeltaF: {node.targetDeltaF})");
                Debug.WriteLine("");
            }
        }
        else
        {
            node.Clear();
            //TODO: Think about this.surplusNodesAvoided. Can it be correctly incremented?
        }
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
        generated = generated.Where(
            node => ((WorldStateForPartialExpansion)node).remainingDeltaF != ushort.MaxValue && // last move was good
                    ((WorldStateForPartialExpansion)node).hasChildrenForCurrentDeltaF(agentIndex+1)
        ).ToList();

        return generated;
    }

    protected override bool ProcessGeneratedNode(WorldState currentNode)
    {
        bool ret = base.ProcessGeneratedNode(currentNode);

        var node = (WorldStateForPartialExpansion)currentNode;
        node.ClearExpansionData();
        node.sic = (int)SumIndividualCosts.h(node, this.instance);
        // We defer calling the expensive calcSingleAgentDeltaFs to when the node is expanded, which means we might only then find out the node's current
        // target deltaF=0 is not possibe.
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
