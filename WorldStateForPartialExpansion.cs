using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;

namespace mapf
{
    class WorldStateForPartialExpansion : WorldState
    {
        public bool alreadyExpanded;
        /// <summary>
        /// Starts at zero, incremented after a node is expanded once. Set on Expand.
        /// </summary>
        public ushort targetDeltaF;
        /// <summary>
        /// Remaining delta F towards targetDeltaF. Reset on Expand.
        /// </summary>
        public ushort remainingDeltaF;
        /// <summary>
        /// For each agent and each direction it can go, the effect of that move on F
        /// byte.MaxValue means this is an illegal move. Only computed on demand.
        /// </summary>
        protected byte[][] singleAgentDeltaFs;
        /// <summary>
        /// Only computed on demand
        /// </summary>
        protected ushort maxDeltaF;
        /// <summary>
        /// Per each agent and delta F, has 1 if that delta F is achievable by moving the agents starting from this one on,
        /// -1 if it isn't, and 0 if we don't know yet.
        /// Only computed on demand
        /// </summary>
        protected sbyte[][] fLookup; 

        /// <summary>
        /// Create a state with the given state for every agent.
        /// </summary>
        /// <param name="allAgentsState"></param>
        /// <param name="minDepth"></param>
        /// <param name="minCost"></param>
        public WorldStateForPartialExpansion(AgentState[] allAgentsState, int minDepth = -1,
                                             int minCost = -1, MDDNode mddNode = null):
            base(allAgentsState, minDepth, minCost, mddNode)
        {
            this.alreadyExpanded = false;
            this.maxDeltaF = 0;
            this.singleAgentDeltaFs = null;
            this.fLookup = null; 
        }

        /// <summary>
        /// Copy constructor
        /// </summary>
        /// <param name="cpy"></param>
        public WorldStateForPartialExpansion(WorldStateForPartialExpansion cpy)
            : base(cpy)
        {
            alreadyExpanded = false; // Creating a new unexpanded node from cpy

            // For intermediate nodes created during expansion (fully expanded nodes have these fields recalculated when they're expanded)
            remainingDeltaF = cpy.remainingDeltaF;
            singleAgentDeltaFs = cpy.singleAgentDeltaFs; // For the UpdateRemainingDeltaF call on temporary nodes.
                                                         // Notice that after an agent is moved its row won't be up-to-date.
            fLookup = cpy.fLookup; // For the hasChildrenForCurrentDeltaF call on temporary nodes.
                                   // Notice that after an agent is moved, all rows up to and including the one of the agent that moved
                                   // won't be up-to-date.
            maxDeltaF = cpy.maxDeltaF; // Not necessarily achievable after some of the agents moved.
            // The above is OK because we won't be using data for agents that already moved.
        }

        /// <summary>
        /// From generated nodes. Allows expansion table to be garbage collected before all generated nodes are expanded.
        /// </summary>
        public void ClearExpansionData()
        {
            this.singleAgentDeltaFs = null;
            this.fLookup = null;
        }

        public override int CompareTo(IBinaryHeapItem other)
        {
            this.h += this.targetDeltaF;
            WorldStateForPartialExpansion otherNode = (WorldStateForPartialExpansion)other;
            otherNode.h += otherNode.targetDeltaF;

            int res = base.CompareTo(other);

            this.h -= this.targetDeltaF;
            otherNode.h -= otherNode.targetDeltaF;

            return res;
        }

        public override string ToString()
        {
            //return base.ToString() + "\nTarget delta F = " + this.targetDeltaF;
            return base.ToString() + " with target delta F = " + this.targetDeltaF;
        }

        public delegate bool ValidityChecker(TimedMove move, IReadOnlyDictionary<TimedMove, int> currentMoves, int makespan, int agentIndex, WorldState node, WorldState intermediateNode);

        private static readonly IReadOnlyDictionary<TimedMove, int> noMoves = new Dictionary<TimedMove, int>();

        /// <summary>
        /// Calculates for each agent and each direction it can go, the effect of that move on F. Illegal moves get byte.MaxValue.
        /// Also calcs maxDeltaF.
        /// Implicitly uses the SIC heuristic.
        /// </summary>
        /// <param name="problem">For GetSingleAgentOptimalCost</param>
        /// <param name="isValid"></param>
        /// <returns></returns>
        public void calcSingleAgentDeltaFs(ProblemInstance problem, ValidityChecker isValid)
        {
            // Init
            this.singleAgentDeltaFs = new byte[allAgentsState.Length][];
            for (int i = 0; i < singleAgentDeltaFs.Length; i++)
            {
                this.singleAgentDeltaFs[i] = new byte[Constants.NUM_ALLOWED_DIRECTIONS];
            }

            int hBefore, hAfter;

            this.maxDeltaF = 0;

            // Set values
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                hBefore = problem.GetSingleAgentOptimalCost(allAgentsState[i]);
                
                int singleAgentMaxLegalDeltaF = -1;

                foreach (TimedMove check in allAgentsState[i].lastMove.GetNextMoves())
                {
                    if (isValid(check, noMoves, this.makespan + 1, i, this, this) == false)
                    {
                         singleAgentDeltaFs[i][(int)check.direction] = byte.MaxValue;
                    }
                    else
                    {
                        hAfter = problem.GetSingleAgentOptimalCost(allAgentsState[i].agent.agentNum, check);

                        if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG)
                        {
                            if (hBefore != 0)
                                singleAgentDeltaFs[i][(int)check.direction] = (byte)(hAfter - hBefore + 1); // h difference + g difference in this specific domain
                            else if (hAfter != 0) // If agent moved from its goal we must count and add all the steps it was stationed at the goal, since they're now part of its g difference
                                singleAgentDeltaFs[i][(int)check.direction] = (byte)(hAfter - hBefore + makespan - allAgentsState[i].arrivalTime + 1);
                            else
                                singleAgentDeltaFs[i][(int)check.direction] = 0; // This is a WAIT move at the goal.
                        }
                        else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
                        {
                            if (hBefore == 0 && hAfter == 0)
                                singleAgentDeltaFs[i][(int)check.direction] = 0; // This is a WAIT move at the goal.
                            else
                                singleAgentDeltaFs[i][(int)check.direction] = (byte)(hAfter - hBefore + 1); // h difference + g difference in this specific domain
                        }
                        singleAgentMaxLegalDeltaF = Math.Max(singleAgentMaxLegalDeltaF, singleAgentDeltaFs[i][(int)check.direction]);
                    }
                }

                if (singleAgentMaxLegalDeltaF == -1) // No legal action for this agent, so no legal children exist for this node
                {
                    this.maxDeltaF = 0; // Can't make it negative without widening the field.
                    break;
                }

                this.maxDeltaF += (byte) singleAgentMaxLegalDeltaF;
            }

            fLookup = new sbyte[allAgentsState.Length][];
            for (int i = 0; i < fLookup.Length; i++)
            {
                fLookup[i] = new sbyte[this.maxDeltaF + 1]; // Towards the last agents most of the row will be wasted (the last one can do delta F of 0 or 1),
                                                            // but it's easier than fiddling with array sizes
            }
        }

        /// <summary>
        /// Returns whether all possible f values were generated from this node already
        /// </summary>
        /// <returns></returns>
        public bool hasMoreChildren()
        {
            return this.targetDeltaF <= this.maxDeltaF;
        }
        
        public bool IsAlreadyExpanded()
        {
            return alreadyExpanded;
        }

        public bool hasChildrenForCurrentDeltaF(int agentNum=0)
        {
            return existsChildForF(agentNum, this.remainingDeltaF);
        }

        /// <summary>
        /// Recursive func. Kind of dynamic programming as it updates the lookup table as it goes to refrain from computing answers twice.
        /// </summary>
        /// <param name="agentNum"></param>
        /// <param name="remainingTargetDeltaF"></param>
        /// <returns></returns>
        protected bool existsChildForF(int agentNum, ushort remainingTargetDeltaF)
        {
            // Stopping conditions:
            if (agentNum == allAgentsState.Length)
            {
                if (remainingTargetDeltaF == 0)
                    return true;
                return false;
            }
            
            if (fLookup[agentNum][remainingTargetDeltaF] != 0) // Answer known (arrays are initialized to zero). TODO: Replace the magic.
            {
                return fLookup[agentNum][remainingTargetDeltaF] == 1; // Return known answer. TODO: Replace the magic
            }

            // Recursive actions:
            for (int direction = 0; direction < Constants.NUM_ALLOWED_DIRECTIONS; direction++)
            {
                if (singleAgentDeltaFs[agentNum][direction] > remainingTargetDeltaF) // Small optimization - no need to make the recursive
                                                                                     // call just to request a negative target from it and
                                                                                     // get false (because we assume the heuristic function
                                                                                     // is consistent)
                    continue;
                if (existsChildForF(agentNum + 1, (byte)(remainingTargetDeltaF - singleAgentDeltaFs[agentNum][direction])))
                {
                    fLookup[agentNum][remainingTargetDeltaF] = 1;
                    return true;
                }
            }
            fLookup[agentNum][remainingTargetDeltaF] = -1;
            return false;
        }

        /// <summary>
        /// An agent was moved between calculating the singleAgentDeltaFs and this call.
        /// Using the data that describes its delta F potential before the move.
        /// </summary>
        /// <param name="agentIndex"></param>
        public void UpdateRemainingDeltaF(int agentIndex) {
            Debug.Assert(this.remainingDeltaF != ushort.MaxValue, "Remaining deltaF is ushort.MaxValue, a reserved value with special meaning. agentIndex=" + agentIndex);

            byte lastMoveDeltaF = this.singleAgentDeltaFs[agentIndex][(int)this.allAgentsState[agentIndex].lastMove.direction];
            if (lastMoveDeltaF != byte.MaxValue && this.remainingDeltaF >= lastMoveDeltaF)
                this.remainingDeltaF -= lastMoveDeltaF;
            else
                this.remainingDeltaF = ushort.MaxValue; // Either because last move was illegal or because the delta F from the last move was more than the entire remaining delta F budget
        }

        /// <summary>
        /// For fully expanded nodes.
        /// Notice ClearExpansionData does a similar thing, but for different reasons.
        /// </summary>
        public override void Clear()
        {
            this.alreadyExpanded = false; // Enables reopening
            // The following info could be reused when reopening the node, saving the time it takes to generate it,
            // but reopening a node is rare in our domain, and the memory saved can be significant
            this.fLookup = null; // Save a lot of memory
            this.singleAgentDeltaFs = null; // Save some more memory
            this.targetDeltaF = 0;
            this.remainingDeltaF = 0;
            //this.h -= this.hBonus; // Reset the h
            //this.hBonus = 0;
        }

        public override int f
        {
            get
            {
                return this.g + this.h + this.targetDeltaF;
            }
        }
    }
}
