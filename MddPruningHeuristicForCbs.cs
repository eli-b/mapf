using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace mapf
{
    class MddPruningHeuristicForCbs : ILazyHeuristic<CbsNode>
    {
        public MddPruningHeuristicForCbs(bool ignoreConstraints = false)
        {
            this.ignoreConstraints = ignoreConstraints;
        }

        protected int pruningSuccesses;
        protected int pruningFailures;
        protected int cacheHits;
        protected int targetTooHigh;
        protected int accPruningSuccesses;
        protected int accPruningFailures;
        protected int accCacheHits;
        protected int accTargetTooHigh;

        protected bool ignoreConstraints;
        protected ProblemInstance instance;

        /// <summary>
        /// Maps a pair of agents and their costs to whether there's a solution with those costs
        /// </summary>
        public Dictionary<(int agentAIndex, int agentBIndex, int agentACost, int agentBCost), ushort> cache;

        public int NumStatsColumns
        {
            get
            {
                return 4;
            }
        }

        public string GetName()
        {
            return "MDD Pruning Heuristic";
        }

        public void AccumulateStatistics()
        {
            this.accPruningSuccesses += this.pruningSuccesses;
            this.accPruningFailures += this.pruningFailures;
            this.accCacheHits += this.cacheHits;
            this.accTargetTooHigh += this.targetTooHigh;
        }

        public void ClearAccumulatedStatistics()
        {
            this.accPruningSuccesses = 0;
            this.accPruningFailures = 0;
            this.accCacheHits = 0;
            this.accTargetTooHigh = 0;
        }

        public void ClearStatistics()
        {
            this.pruningSuccesses = 0;
            this.pruningFailures = 0;
            this.cacheHits = 0;
            this.targetTooHigh = 0;
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
            var agentIndicesAndCosts = (s.conflict.agentAIndex, s.conflict.agentBIndex,
                    s.allSingleAgentCosts[s.conflict.agentAIndex], s.allSingleAgentCosts[s.conflict.agentBIndex]);
            if (this.ignoreConstraints)
            {
                if (this.cache.ContainsKey(agentIndicesAndCosts))
                {
                    this.cacheHits++;
                    return this.cache[agentIndicesAndCosts];
                }
            }
            // TODO

            if (s.GoalTest())
            {
                return 0;
            }

            if (s.h > 1)
            {
                return 1;  // We can't raise the heuristic more than that
            }

            if (s.GetGroupSize(s.conflict.agentAIndex) > 1 || s.GetGroupSize(s.conflict.agentBIndex) > 1)
            {
                return 0; // Without saving the result, as it's just a cop-out
            }

            int maxCost = Math.Max(s.allSingleAgentCosts[s.conflict.agentAIndex],
                                   s.allSingleAgentCosts[s.conflict.agentBIndex]);
            // Building MDDs for the conflicting agents. We can't keep them because we're
            // destructively syncing them later (the first one, at least).
            var mddA = new MDD(s.conflict.agentAIndex, this.instance.agents[s.conflict.agentAIndex].agent.agentNum,
                                this.instance.agents[s.conflict.agentAIndex].lastMove,
                                s.allSingleAgentCosts[s.conflict.agentAIndex], maxCost,
                                this.instance.GetNumOfAgents(), this.instance, this.ignoreConstraints);
            var mddB = new MDD(s.conflict.agentBIndex, this.instance.agents[s.conflict.agentBIndex].agent.agentNum,
                               this.instance.agents[s.conflict.agentBIndex].lastMove,
                               s.allSingleAgentCosts[s.conflict.agentBIndex], maxCost,
                               this.instance.GetNumOfAgents(), this.instance, this.ignoreConstraints);
            s.cbs.mddsBuilt += 2;
            (MDD.PruningDone ans, int stat) = mddA.SyncMDDs(mddB, checkTriples: false);
            if (ans == MDD.PruningDone.EVERYTHING)
            {
                if (this.ignoreConstraints)
                    this.cache.Add(agentIndicesAndCosts, 1);
                this.pruningSuccesses++;
                return 1;
            }
            else
            {
                if (this.ignoreConstraints)
                    this.cache.Add(agentIndicesAndCosts, 0);
                this.pruningFailures++;
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
            if (s.g + 1 < target)
            {
                this.targetTooHigh++;
                return 0;  // Currently we can only give an estimate of 1
            }
            return this.h(s);
        }

        public void Init(ProblemInstance pi, List<uint> agentsToConsider)
        {
            this.instance = pi;
        }

        public void OutputAccumulatedStatistics(TextWriter output)
        {
            string name = this.GetName();
            Console.WriteLine($"{name} Accumulated Pruning Successes (High-Level): {this.accPruningSuccesses}");
            Console.WriteLine($"{name} Accumulated Pruning Failures (High-Level): {this.accPruningFailures}");
            Console.WriteLine($"{name} Accumulated Cache Hits (High-Level): {this.accCacheHits}");
            Console.WriteLine($"{name} Accumulated Times Target Estimate Was Too High (High-Level): {this.accTargetTooHigh}");

            output.Write(this.accPruningSuccesses + Run.RESULTS_DELIMITER);
            output.Write(this.accPruningFailures + Run.RESULTS_DELIMITER);
            output.Write(this.accCacheHits + Run.RESULTS_DELIMITER);
            output.Write(this.accTargetTooHigh + Run.RESULTS_DELIMITER);
        }

        public void OutputStatistics(TextWriter output)
        {
            string name = this.GetName();
            Console.WriteLine($"{name} Pruning successes (High-Level): {this.pruningSuccesses}");
            Console.WriteLine($"{name} Pruning failures (High-Level): {this.pruningFailures}");
            Console.WriteLine($"{name} Cache hits (High-Level): {this.cacheHits}");
            Console.WriteLine($"{name} Times Target Estimate was Too High (High-Level): {this.targetTooHigh}");

            output.Write(this.pruningSuccesses + Run.RESULTS_DELIMITER);
            output.Write(this.pruningFailures + Run.RESULTS_DELIMITER);
            output.Write(this.cacheHits + Run.RESULTS_DELIMITER);
            output.Write(this.targetTooHigh + Run.RESULTS_DELIMITER);
        }

        public void OutputStatisticsHeader(TextWriter output)
        {
            string name = this.GetName();
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
}
