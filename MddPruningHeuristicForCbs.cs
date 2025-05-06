using System;
using System.Collections.Generic;
using System.IO;

namespace mapf;

class MddPruningHeuristicForCbs : ILazyHeuristic<CbsNode>
{
    public MddPruningHeuristicForCbs(bool ignoreConstraints = false)
    {
        _ignoreConstraints = ignoreConstraints;
    }

    private int _pruningSuccesses;
    private int _pruningFailures;
    private int _cacheHits;
    private int _targetTooHigh;
    private int _accPruningSuccesses;
    private int _accPruningFailures;
    private int _accCacheHits;
    private int _accTargetTooHigh;

    private bool _ignoreConstraints;
    private ProblemInstance _instance;

    /// <summary>
    /// Maps a pair of agents and their costs to whether there's a solution with those costs
    /// </summary>
    public Dictionary<(int agentAIndex, int agentBIndex, int agentACost, int agentBCost), ushort> cache;
    public int NumStatsColumns => 4;
    public string GetName()
    {
        return "MDD Pruning Heuristic";
    }

    public void AccumulateStatistics()
    {
        _accPruningSuccesses += _pruningSuccesses;
        _accPruningFailures += _pruningFailures;
        _accCacheHits += _cacheHits;
        _accTargetTooHigh += _targetTooHigh;
    }

    public void ClearAccumulatedStatistics()
    {
        _accPruningSuccesses = 0;
        _accPruningFailures = 0;
        _accCacheHits = 0;
        _accTargetTooHigh = 0;
    }

    public void ClearStatistics()
    {
        _pruningSuccesses = 0;
        _pruningFailures = 0;
        _cacheHits = 0;
        _targetTooHigh = 0;
    }

    /// <summary>
    /// Returns 0 if the conflict is solvable with the current costs, 1 otherwise. If unsure, returns 0;
    /// Currently avoiding building a k-agent MDD.
    /// TODO: This became largely irrelevant after we've discovered cardinal conflicts, but
    /// now that we're always building all MDDs, it might be worth it to try to sync the MDDs
    /// for nodes with no cardinal conflicts. We might find they're still unsolvable
    /// with the current costs.
    /// </summary>
    /// <param name="s"></param>
    /// <param name="ignoreConstraints">
    /// Ignore constraints. This allows the results for the given costs to be used elsewhere in
    /// the CBS tree, but can cause under-estimates and make building the MDD a little slower.
    /// </param>
    /// <returns>
    /// If after syncing and pruning the MDDs at the current costs for the two conflicting agents
    /// the whole MDD was pruned, indicating these agents can't be solved at the current cost,
    /// returns 1. Else returns 0
    /// </returns>
    public uint h(CbsNode s)
    {
        var agentIndicesAndCosts = (s.Conflict.agentAIndex, s.Conflict.agentBIndex,
                s.SingleAgentCosts[s.Conflict.agentAIndex], s.SingleAgentCosts[s.Conflict.agentBIndex]);
        if (_ignoreConstraints)
        {
            if (cache.ContainsKey(agentIndicesAndCosts))
            {
                _cacheHits++;
                return cache[agentIndicesAndCosts];
            }
        }
        // TODO

        if (s.GoalTest())
        {
            return 0;
        }

        if (s.H > 1)
        {
            return 1;  // We can't raise the heuristic more than that
        }

        if (s.GetGroupSize(s.Conflict.agentAIndex) > 1 || s.GetGroupSize(s.Conflict.agentBIndex) > 1)
        {
            return 0; // Without saving the result, as it's just a cop-out
        }

        int maxCost = Math.Max(s.SingleAgentCosts[s.Conflict.agentAIndex],
                                s.SingleAgentCosts[s.Conflict.agentBIndex]);
        // Building MDDs for the conflicting agents. We can't keep them because we're
        // destructively syncing them later (the first one, at least).
        var mddA = new MDD(s.Conflict.agentAIndex, _instance.agents[s.Conflict.agentAIndex].agent.agentNum,
                            _instance.agents[s.Conflict.agentAIndex].lastMove,
                            s.SingleAgentCosts[s.Conflict.agentAIndex], maxCost,
                            _instance.GetNumOfAgents(), _instance, _ignoreConstraints);
        var mddB = new MDD(s.Conflict.agentBIndex, _instance.agents[s.Conflict.agentBIndex].agent.agentNum,
                            _instance.agents[s.Conflict.agentBIndex].lastMove,
                            s.SingleAgentCosts[s.Conflict.agentBIndex], maxCost,
                            _instance.GetNumOfAgents(), _instance, _ignoreConstraints);
        s.CBS.MDDsBuilt += 2;
        MDD.PruningDone ans = mddA.SyncMDDs(mddB, checkTriples: false).Item1;
        if (ans == MDD.PruningDone.EVERYTHING)
        {
            if (_ignoreConstraints)
                cache.Add(agentIndicesAndCosts, 1);
            _pruningSuccesses++;
            return 1;
        }
        else
        {
            if (_ignoreConstraints)
                cache.Add(agentIndicesAndCosts, 0);
            _pruningFailures++;
            return 0;
        }
    }

    /// <summary>
    /// Lazy version
    /// </summary>
    /// <param name="s"></param>
    /// <param name="target"></param>
    /// <returns></returns>
    public uint h(CbsNode s, int target)
    {
        if (s.G + 1 < target)
        {
            _targetTooHigh++;
            return 0;  // Currently we can only give an estimate of 1
        }
        return h(s);
    }

    public void Init(ProblemInstance pi, List<uint> agentsToConsider)
    {
        _instance = pi;
    }

    public void OutputAccumulatedStatistics(TextWriter output)
    {
        string name = GetName();
        Console.WriteLine($"{name} Accumulated Pruning Successes (High-Level): {_accPruningSuccesses}");
        Console.WriteLine($"{name} Accumulated Pruning Failures (High-Level): {_accPruningFailures}");
        Console.WriteLine($"{name} Accumulated Cache Hits (High-Level): {_accCacheHits}");
        Console.WriteLine($"{name} Accumulated Times Target Estimate Was Too High (High-Level): {_accTargetTooHigh}");

        output.Write(_accPruningSuccesses + Run.RESULTS_DELIMITER);
        output.Write(_accPruningFailures + Run.RESULTS_DELIMITER);
        output.Write(_accCacheHits + Run.RESULTS_DELIMITER);
        output.Write(_accTargetTooHigh + Run.RESULTS_DELIMITER);
    }

    public void OutputStatistics(TextWriter output)
    {
        string name = GetName();
        Console.WriteLine($"{name} Pruning successes (High-Level): {_pruningSuccesses}");
        Console.WriteLine($"{name} Pruning failures (High-Level): {_pruningFailures}");
        Console.WriteLine($"{name} Cache hits (High-Level): {_cacheHits}");
        Console.WriteLine($"{name} Times Target Estimate was Too High (High-Level): {_targetTooHigh}");

        output.Write(_pruningSuccesses + Run.RESULTS_DELIMITER);
        output.Write(_pruningFailures + Run.RESULTS_DELIMITER);
        output.Write(_cacheHits + Run.RESULTS_DELIMITER);
        output.Write(_targetTooHigh + Run.RESULTS_DELIMITER);
    }

    public void OutputStatisticsHeader(TextWriter output)
    {
        string name = GetName();
        output.Write($"{name} Pruning Successes (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write($"{name} Pruning Failures (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write($"{name} Cache Hits (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write($"{name} Times Target Estimate Was Too High");
        output.Write(Run.RESULTS_DELIMITER);
    }
}
