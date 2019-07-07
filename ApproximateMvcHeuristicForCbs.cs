using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Diagnostics;

namespace mapf
{
    /// <summary>
    /// Uses the half the size of the 2-approximation of the MVC of the cardinal conflict graph as a
    /// heuristic. Builds fewer MDDs when the lazy method is used, and, of course, runs faster than
    /// brute-forcing the MVC.
    /// </summary>
    class ApproximateMvcHeuristicForCbs : ILazyHeuristic<CbsNode>
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
            return "Approximate MVC of Cardinal Conflict Graph Heuristic";
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
        /// Compute h value with an approximate minimum vertex cover solver.
        /// This heuristic is bounded by the maximal matching of the graph.
        /// Assumes the s.cbs.mergeThreshold is -1.
        /// </summary>
        /// <returns></returns>
        public uint h(CbsNode s)
        {
            return this.h(s, int.MaxValue);
        }

        /// <summary>
        /// Lazy version - only builds MDDs if it has to, and only computes the heuristic up to the
        /// target value.
        /// </summary>
        /// <param name="s"></param>
        /// <param name="target"></param>
        /// <returns></returns>
        public uint h(CbsNode s, int target)
        {
            Debug.WriteLine($"Computing heuristic estimate for node hash {s.GetHashCode()}");
            if (target != int.MaxValue && target > s.totalInternalAgentsThatConflict)
            {
                Debug.WriteLine($"Target estimate {target} was too high!");
                this.targetClearlyTooHigh++;
                return 0;
            }

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
                bool hasMdd = s.mddNarrownessValues[agentIndex] != null;

                bool largeEnough = false;
                foreach (int conflictingAgentNum in s.conflictTimesPerAgent[agentIndex].Keys)
                {
                    int conflictingAgentIndex = s.agentNumToIndex[conflictingAgentNum];
                    if (conflictingAgentIndex < agentIndex) // check later
                        continue;
                    bool otherHasMdd = s.mddNarrownessValues[conflictingAgentIndex] != null;

                    bool addedToVC = false;
                    foreach (int conflictTime in s.conflictTimesPerAgent[agentIndex][conflictingAgentNum])
                    {
                        if (hasMdd == false)
                        {
                            if (otherHasMdd == false || s.DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, agentIndex, groups))  // Other agent's MDD is narrow at this timestep.
                            {
                                s.buildMddForAgentWithItsCurrentCost(agentIndex);
                                hasMdd = true;
                            }
                            else
                                continue;
                        }
                        bool iNarrow = s.DoesAgentHaveNoOtherOption(agentIndex, conflictTime, conflictingAgentIndex, groups);
                        if (iNarrow == false)
                            continue;
                        if (otherHasMdd == false)
                        {
                            s.buildMddForAgentWithItsCurrentCost(conflictingAgentIndex);
                            otherHasMdd = true;
                        }
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

        public void Init(ProblemInstance pi, List<uint> agents)
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
}
