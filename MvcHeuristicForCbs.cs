using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace CPF_experiment
{
    class MvcHeuristicForCbs : ILazyHeuristic<CbsNode>
    {
        protected int targetTooHigh;
        protected int accTargetTooHigh;

        public int NumStatsColumns
        {
            get
            {
                return 1;
            }
        }

        public string GetName()
        {
            return "MVC of Cardinal Conflict Graph Heuristic";
        }

        public void AccumulateStatistics()
        {
            this.accTargetTooHigh += this.targetTooHigh;
        }

        public void ClearAccumulatedStatistics()
        {
            this.accTargetTooHigh = 0;
        }

        public void ClearStatistics()
        {
            this.targetTooHigh = 0;
        }

        /// <summary>
        /// Compute h value with a minimum vertex cover solver.
        /// Assumes the s.cbs.mergeThreshold is -1.
        /// </summary>
        /// <returns></returns>
        public uint h(CbsNode s)
        {
            s.buildAllMDDs();

            ConflictGraph CardinallyConflictingAgents = new ConflictGraph(s.allSingleAgentPlans.Length);
            ISet<int>[] groups = s.GetGroups();

            // Populate the cardinal conflict graph
            foreach (var agentIndex in Enumerable.Range(0, s.allSingleAgentPlans.Length))
            {
                if (s.conflictTimesPerAgent[agentIndex].Count == 0)
                    continue;  // Agent has no conflicts

                foreach (int conflictingAgentNum in s.conflictTimesPerAgent[agentIndex].Keys)
                {
                    int conflictingAgentIndex = s.agentNumToIndex[conflictingAgentNum];
                    if (conflictingAgentIndex < agentIndex) // check later
                        continue;

                    foreach (int conflictTime in s.conflictTimesPerAgent[agentIndex][conflictingAgentNum])
                    {
                        bool iNarrow = s.DoesAgentHaveNoOtherOption(agentIndex, conflictTime, conflictingAgentIndex, groups);
                        bool jNarrow = s.DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, agentIndex, groups);
                        if (iNarrow && jNarrow) // Cardinal conflict
                            CardinallyConflictingAgents.Add(agentIndex, conflictingAgentIndex);
                    }
                }
            }

            if (s.prev == null)
                s.minimumVertexCover = CardinallyConflictingAgents.MinimumVertexCover();
            else
                s.minimumVertexCover = CardinallyConflictingAgents.MinimumVertexCover(s.prev.minimumVertexCover);
            if (s.h < s.minimumVertexCover)
            {
                s.hBonus = s.minimumVertexCover - s.h;
                s.h = s.minimumVertexCover;
            }
            return (uint)s.h;
        }

        /// <summary>
        /// Lazy version
        /// </summary>
        /// <param name="s"></param>
        /// <param name="target"></param>
        /// <returns></returns>
        public uint h(CbsNode s, int target)
        {
            if (s.prev != null && target > s.prev.minimumVertexCover + 1)
            {
                this.targetTooHigh++;
                return 0;  // We can't give a higher estimate than the size of the parent's MVC + 1,
                           // see comments in ConflictGraph.MinimumVertexCover for details.
                           // 0 just signals we couldn't raise the h enough.
            }
            return this.h(s);
        }

        public void init(ProblemInstance pi, List<uint> vAgents)
        {

        }

        public void OutputAccumulatedStatistics(TextWriter output)
        {
            string name = this.GetName();
            Console.WriteLine($"{name} Accumulated Times Target Estimate Was Too High (High-Level): {this.accTargetTooHigh}");

            output.Write(this.accTargetTooHigh + Run.RESULTS_DELIMITER);
        }

        public void OutputStatistics(TextWriter output)
        {
            string name = this.GetName();
            Console.WriteLine($"{name} Times Target Estimate was Too High (High-Level): {this.targetTooHigh}");

            output.Write(this.targetTooHigh + Run.RESULTS_DELIMITER);
        }

        public void OutputStatisticsHeader(TextWriter output)
        {
            string name = this.GetName();
            output.Write($"{name} Times Target Estimate Was Too High");
            output.Write(Run.RESULTS_DELIMITER);
        }
    }
}
