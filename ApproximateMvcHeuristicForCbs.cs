using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace CPF_experiment
{
    /// <summary>
    /// Uses the half the size of the 2-approximation of the MVC of the cardinal conflict graph as a
    /// heuristic. Builds fewer MDDs when the lazy method is used, and, of course, runs faster than
    /// brute-forcing the MVC.
    /// </summary>
    class ApproximateMvcHeuristicForCbs : ILazyHeuristic<CbsNode>
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
            return "Approximate MVC of Cardinal Conflict Graph Heuristic";
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
        /// Compute h value with an approximate minimum vertex cover solver.
        /// This heuristic is bounded by the maximal matching of the graph.
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

            s.minimumVertexCover = CardinallyConflictingAgents.ApproximateMinimumVertexCover() / 2;  // The approximation is always even.
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
            var vertexCover = new HashSet<int>();
            ISet<int>[] groups = s.GetGroups();
            int targetTimes2 = 2 * target;

            // Populate the cardinal conflict graph
            foreach (var agentIndex in Enumerable.Range(0, s.allSingleAgentPlans.Length))
            {
                if (s.conflictTimesPerAgent[agentIndex].Count == 0)
                    continue;  // Agent has no conflicts
                if (vertexCover.Contains(agentIndex))  // All its edges are already covered
                    continue;
                s.buildMddForAgentWithItsCurrentCost(agentIndex);

                bool largeEnough = false;
                foreach (int conflictingAgentNum in s.conflictTimesPerAgent[agentIndex].Keys)
                {
                    int conflictingAgentIndex = s.agentNumToIndex[conflictingAgentNum];
                    if (conflictingAgentIndex < agentIndex) // check later
                        continue;
                    s.buildMddForAgentWithItsCurrentCost(conflictingAgentIndex);  // TODO: Only build it if the width of the already-built MDD
                                                                                  // at the time of a conflict is 1.

                    bool addedToVC = false;
                    foreach (int conflictTime in s.conflictTimesPerAgent[agentIndex][conflictingAgentNum])
                    {
                        bool iNarrow = s.DoesAgentHaveNoOtherOption(agentIndex, conflictTime, conflictingAgentIndex, groups);
                        bool jNarrow = s.DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, agentIndex, groups);
                        if (iNarrow && jNarrow) // Cardinal conflict
                        {
                            vertexCover.Add(agentIndex);
                            vertexCover.Add(conflictingAgentIndex);
                            addedToVC = true;
                            largeEnough = vertexCover.Count >= targetTimes2;
                            break;
                        }
                    }
                    if (addedToVC)
                        break;
                }
                if (largeEnough)
                    break;
            }

            s.minimumVertexCover = vertexCover.Count / 2;  // The approximation is always even.
            if (s.h < s.minimumVertexCover)
            {
                s.hBonus = s.minimumVertexCover - s.h;
                s.h = s.minimumVertexCover;
            }
            return (uint)s.h;
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
