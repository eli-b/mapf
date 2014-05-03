using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    class WorldStateForPartialExpansion : WorldState
    {
        public bool alreadyExpanded;
        /// <summary>
        /// Starts at zero, incremented after a node is expanded once. Set on Expand.
        /// </summary>
        public byte targetDeltaF;
        /// <summary>
        /// Remaining delta F towards targetDeltaF. Reset on Expand.
        /// </summary>
        public byte remainingDeltaF;
        /// <summary>
        /// For each agent and each direction it can go, the effect of that move on F
        /// byte.MaxValue means this is an illegal move. Only computed on demand.
        /// </summary>
        protected byte[][] singleAgentDeltaFs;
        /// <summary>
        /// Only computed on demand
        /// </summary>
        protected byte maxDeltaF;
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
        public WorldStateForPartialExpansion(AgentState[] allAgentsState): base(allAgentsState)
        {
            alreadyExpanded = false;
            maxDeltaF = 0;
            singleAgentDeltaFs = null;
            fLookup = null; 
        }

        /// <summary>
        /// Copy constructor
        /// </summary>
        /// <param name="cpy"></param>
        public WorldStateForPartialExpansion(WorldStateForPartialExpansion cpy)
            : base(cpy)
        {
            alreadyExpanded = false; // Creating a new unexpanded node from cpy
            remainingDeltaF = cpy.remainingDeltaF;
            singleAgentDeltaFs = cpy.singleAgentDeltaFs; // Notice that after an agent is moved its row won't be up-to-date.
            fLookup = null; // Notice that after an agent is moved, all rows up to and including the one of the agent that moved won't be up-to-date.
            maxDeltaF = 0; // cpy.maxDeltaF // TODO: Same.
        }

        /// <summary>
        /// Calculates for each agent and each direction it can go, the effect of that move on F. Illegal moves get byte.MaxValue
        /// </summary>
        /// <param name="problem"></param>
        /// <returns></returns>
        public void calcSingleAgentDeltaFs(ProblemInstance problem)
        {
            //init
            this.singleAgentDeltaFs = new byte[allAgentsState.Length][];
            for (int i = 0; i < singleAgentDeltaFs.Length; i++)
            {
                singleAgentDeltaFs[i] = new byte[Constants.NUM_ALLOWED_DIRECTIONS];
            }

            int hBefore, hAfter;

            //set values
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                hBefore = problem.GetSingleAgentShortestPath(allAgentsState[i]);

                foreach (TimedMove check in allAgentsState[i].lastMove.GetNextMoves(Constants.ALLOW_DIAGONAL_MOVE))
                {
                    if (problem.parameters.ContainsKey(Trevor.ILLEGAL_MOVES_KEY))
                    {
                        HashSet<TimedMove> reserved = (HashSet<TimedMove>)(problem.parameters[Trevor.ILLEGAL_MOVES_KEY]);
                        if (check.isColliding(reserved))
                        {
                            singleAgentDeltaFs[i][(int)check.direction] = byte.MaxValue;
                            continue;
                        }
                    }
                    
                    if (problem.IsValidTile(check.x, check.y))
                    {
                        hAfter = problem.GetSingleAgentShortestPath(allAgentsState[i].agent.agentNum, check.x, check.y);

                        if (hBefore != 0)
                            singleAgentDeltaFs[i][(int)check.direction] = (byte)(hAfter - hBefore + 1); // h difference + g difference
                        else if (hAfter != 0) // If agent moved from its goal we must count and add all the steps it was stationed at the goal, since they're now part of its g difference
                            singleAgentDeltaFs[i][(int)check.direction] = (byte)(hAfter - hBefore + makespan - allAgentsState[i].arrivalTime + 1);
                        else
                            singleAgentDeltaFs[i][(int)check.direction] = 0;
                    }
                    else
                    {
                        singleAgentDeltaFs[i][(int)check.direction] = byte.MaxValue;
                    }
                }
            }

            // Sum across all agents of max F difference by a legal move
            this.maxDeltaF = singleAgentDeltaFs.Select<byte[], byte>(
                        agentFDifferences => agentFDifferences.Where<byte>(
                            x => (x < byte.MaxValue)
                        ).Max<byte>()
                   ).Max<byte>();

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
        
        public bool isAlreadyExpanded()
        {
            return alreadyExpanded;
        }

        public bool hasChildrenForCurrentF(int agentNum=0)
        {
            return existsChildForF(agentNum, this.remainingDeltaF);
        }

        /// <summary>
        /// Recursive func. Kind of dynamic programming as it updates the lookup table as it goes to refrain from computing answers twice.
        /// </summary>
        /// <param name="agentNum"></param>
        /// <param name="remainingTargetFChange"></param>
        /// <returns></returns>
        protected bool existsChildForF(int agentNum, byte remainingTargetFChange)
        {
            // Stopping conditions:
            if (agentNum == allAgentsState.Length)
            {
                if (remainingTargetFChange == 0)
                    return true;
                return false;
            }
            
            if (fLookup[agentNum][remainingTargetFChange] != 0) // Answer known (arrays are initialized to zero). TODO: Replace the magic.
            {
                return fLookup[agentNum][remainingTargetFChange] == 1; // Return known answer. TODO: Replace the magic
            }

            // Recursive actions:
            for (int direction = 0; direction < Constants.NUM_ALLOWED_DIRECTIONS; direction++)
            {
                if (singleAgentDeltaFs[agentNum][direction] > remainingTargetFChange) // Small optimization - no need to make the recursive call just to request a negative target from it and get false (because we assume the heuristic function is consistent)
                    continue;
                if (existsChildForF(agentNum + 1, (byte)(remainingTargetFChange - singleAgentDeltaFs[agentNum][direction])))
                {
                    fLookup[agentNum][remainingTargetFChange] = 1;
                    return true;
                }
            }
            fLookup[agentNum][remainingTargetFChange] = -1;
            return false;
        }

        /// <summary>
        /// An agent was moved between calculating the singleAgentDeltaFs and this call. Using the data that describes its delta F potential before the move.
        /// </summary>
        /// <param name="agentIndex"></param>
        public void UpdateRemainingFChange(int agentIndex) {
            byte fChangeFromLastMove = this.singleAgentDeltaFs[agentIndex][(int)this.allAgentsState[agentIndex].lastMove.direction];
            if (fChangeFromLastMove != byte.MaxValue && this.remainingDeltaF >= fChangeFromLastMove)
                this.remainingDeltaF -= fChangeFromLastMove;
            else
                this.remainingDeltaF = byte.MaxValue;
        }
    }
}
