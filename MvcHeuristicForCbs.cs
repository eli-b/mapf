using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Diagnostics;

namespace mapf;

class MvcHeuristicForCbs : ILazyHeuristic<CbsNode>
{
    protected int targetClearlyTooHigh;
    protected int targetReached;
    protected int targetNotReached;
    protected int accTargetClearlyTooHigh;
    protected int accTargetReached;
    protected int accTargetNotReached;

    public int NumStatsColumns
    {
        get
        {
            return 3;
        }
    }

    public string GetName()
    {
        return "MVC of Cardinal Conflict Graph Heuristic";
    }

    public void AccumulateStatistics()
    {
        this.accTargetClearlyTooHigh += this.targetClearlyTooHigh;
        this.accTargetReached += this.targetReached;
        this.accTargetNotReached += this.targetNotReached;
    }

    public void ClearAccumulatedStatistics()
    {
        this.accTargetClearlyTooHigh = 0;
        this.accTargetReached = 0;
        this.accTargetNotReached = 0;
    }

    public void ClearStatistics()
    {
        this.targetClearlyTooHigh = 0;
        this.targetReached = 0;
        this.targetNotReached = 0;
    }

    /// <summary>
    /// Compute h value with a minimum vertex cover solver.
    /// Assumes the s.cbs.mergeThreshold is -1.
    /// </summary>
    /// <returns></returns>
    public uint h(CbsNode s)
    {
        return this.h(s, int.MaxValue);
    }

    /// <summary>
    /// Lazy version
    /// </summary>
    /// <param name="s"></param>
    /// <param name="target"></param>
    /// <returns></returns>
    public uint h(CbsNode s, int target)
    {
        Debug.WriteLine($"Computing heuristic estimate for node hash {s.GetHashCode()}");
        if (target != int.MaxValue &&
            (
            (s.prev != null && s.prev.minimumVertexCover != (int) ConflictGraph.MinVertexCover.NOT_SET && 
                                                                    target > s.prev.minimumVertexCover + 1) ||
            (target > s.totalInternalAgentsThatConflict))
            )
        {
            Debug.WriteLine($"Target estimate {target} was too high!");
            this.targetClearlyTooHigh++;
            return 0;  // We can't give a higher estimate than the size of the parent's MVC + 1,
                        // or the max number of vertices in this agent's conflict graph
                        // see comments in ConflictGraph.MinimumVertexCover for details.
                        // 0 just signals we couldn't raise the h enough.
        }

        ConflictGraph CardinallyConflictingAgents = new ConflictGraph(s.singleAgentPlans.Length);
        ISet<int>[] groups = s.GetGroups();

        // Populate the cardinal conflict graph
        foreach (var agentIndex in Enumerable.Range(0, s.singleAgentPlans.Length))
        {
            if (s.conflictTimesPerAgent[agentIndex].Count == 0)
                continue;  // Agent has no conflicts
            bool hasMdd = s.mddNarrownessValues[agentIndex] != null;
            bool canBuildMDD = groups[agentIndex].Count == 1;
            if (canBuildMDD == false)
                continue;  // We aren't going to lookahead just for the heuristic

            foreach (int conflictingAgentNum in s.conflictTimesPerAgent[agentIndex].Keys)
            {
                int conflictingAgentIndex = s.agentNumToIndex[conflictingAgentNum];
                if (conflictingAgentIndex < agentIndex) // check later
                    continue;
                bool otherHasMdd = s.mddNarrownessValues[conflictingAgentIndex] != null;
                bool otherCanBuildMdd = groups[conflictingAgentIndex].Count == 1;
                if (otherCanBuildMdd == false)
                    continue;  // We won't lookahead just for the heuristic

                foreach (int conflictTime in s.conflictTimesPerAgent[agentIndex][conflictingAgentNum])
                {
                    bool otherNarrow;
                    if (otherHasMdd)
                    {
                        otherNarrow = s.DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, agentIndex, groups);
                        if (otherNarrow == false)
                            continue;
                    }
                    else
                        otherNarrow = false;  // Temporarily. Will be computed soon.

                    s.buildMddForAgentWithItsCurrentCost(agentIndex);  // Does nothing if its MDD is already built.
                    hasMdd = true;
                    bool iNarrow = s.DoesAgentHaveNoOtherOption(agentIndex, conflictTime, conflictingAgentIndex, groups);
                    if (iNarrow == false)
                        continue;
                    if (otherHasMdd == false)
                    {
                        s.buildMddForAgentWithItsCurrentCost(conflictingAgentIndex);
                        otherHasMdd = true;
                        otherNarrow = s.DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, agentIndex, groups);
                    }
                    if (otherNarrow) // Cardinal conflict - both agent's MDDs are narrow at this timestep.
                        CardinallyConflictingAgents.Add(agentIndex, conflictingAgentIndex);
                }
            }
        }

        if (s.prev == null || s.prev.minimumVertexCover == (int) ConflictGraph.MinVertexCover.NOT_SET)
            s.minimumVertexCover = CardinallyConflictingAgents.MinimumVertexCover();
        else
            s.minimumVertexCover = CardinallyConflictingAgents.MinimumVertexCover(s.prev.minimumVertexCover);
        // FIXME: The value might be incorrect after a merge operation which wasn't followed by a restart

        if (target != int.MaxValue)
        {
            if (s.minimumVertexCover >= target)
            {
                Debug.WriteLine($"Target estimate {target} reached");
                this.targetReached++;
            }
            else
            {
                Debug.WriteLine($"Target estimate {target} not reached");
                this.targetNotReached++;
            }
        }

        return (uint)s.minimumVertexCover;
    }

    public void Init(ProblemInstance pi, List<uint> agentsToConsider)
    {

    }

    public void OutputAccumulatedStatistics(TextWriter output)
    {
        string name = this.GetName();
        Console.WriteLine($"{name} Accumulated Times Target Estimate Was Clearly Too High: {this.accTargetClearlyTooHigh}");
        Console.WriteLine($"{name} Accumulated Times Target Estimate Was Reached: {this.accTargetReached}");
        Console.WriteLine($"{name} Accumulated Times Target Estimate Was Not Reached: {this.accTargetNotReached}");

        output.Write(this.accTargetClearlyTooHigh + Run.RESULTS_DELIMITER);
        output.Write(this.accTargetReached + Run.RESULTS_DELIMITER);
        output.Write(this.accTargetNotReached + Run.RESULTS_DELIMITER);
    }

    public void OutputStatistics(TextWriter output)
    {
        string name = this.GetName();
        Console.WriteLine($"{name} times target estimate was clearly too high: {this.targetClearlyTooHigh}");
        Console.WriteLine($"{name} times target estimate was reached: {this.targetReached}");
        Console.WriteLine($"{name} times target estimate was not reached: {this.targetNotReached}");

        output.Write(this.targetClearlyTooHigh + Run.RESULTS_DELIMITER);
        output.Write(this.targetReached + Run.RESULTS_DELIMITER);
        output.Write(this.targetNotReached + Run.RESULTS_DELIMITER);
    }

    public void OutputStatisticsHeader(TextWriter output)
    {
        string name = this.GetName();
        output.Write($"{name} Times Target Estimate Was Too High");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write($"{name} Times Target Estimate Was Reached");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write($"{name} Times Target Estimate Was Not Reached");
        output.Write(Run.RESULTS_DELIMITER);
    }
}
