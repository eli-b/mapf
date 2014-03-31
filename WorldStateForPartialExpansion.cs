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
        public byte targetFChange;
        /// <summary>
        /// Remaining F change towards targetFChange. Reset on Expand.
        /// </summary>
        public byte remainingFChange;
        /// <summary>
        /// For each agent and each direction it can go, the effect of that move on F
        /// byte.MaxValue means this is an illegal move. Only computed on demand.
        /// </summary>
        protected byte[][] singleAgentFDifferences;
        /// <summary>
        /// Only computed on demand
        /// </summary>
        protected byte maxFChange;
        /// <summary>
        /// Per each agent and f change, has 1 if that f change is achievable by moving the agents starting from this one on,
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
            maxFChange = 0;
            singleAgentFDifferences = null;
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
            remainingFChange = cpy.remainingFChange;
            singleAgentFDifferences = cpy.singleAgentFDifferences; // Notice that after an agent is moved its row won't be up-to-date.
            fLookup = null; // Notice that after an agent is moved, all rows up to and including the one of the agent that moved won't be up-to-date.
            maxFChange = 0; // cpy.maxFChange // TODO: Same.
        }

        /// <summary>
        /// Calculates for each agent and each direction it can go, the effect of that move on F. Illegal moves get byte.MaxValue
        /// </summary>
        /// <param name="problem"></param>
        /// <returns></returns>
        public void calcSingleAgentFDifferences(ProblemInstance problem)
        {
            //init
            this.singleAgentFDifferences = new byte[allAgentsState.Length][];
            for (int i = 0; i < singleAgentFDifferences.Length; i++)
            {
                singleAgentFDifferences[i] = new byte[Constants.NUM_ALLOWED_DIRECTIONS];
            }

            int hBefore, hAfter;

            //set values
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                hBefore = problem.GetSingleAgentShortestPath(allAgentsState[i]);

                foreach (TimedMove check in allAgentsState[i].last_move.GetNextMoves(Constants.ALLOW_DIAGONAL_MOVE))
                {
                    if (problem.parameters.ContainsKey(Trevor.ILLEGAL_MOVES_KEY))
                    {
                        HashSet<TimedMove> reserved = (HashSet<TimedMove>)(problem.parameters[Trevor.ILLEGAL_MOVES_KEY]);
                        if (check.isColliding(reserved))
                        {
                            singleAgentFDifferences[i][(int)check.direction] = byte.MaxValue;
                            continue;
                        }
                    }
                    
                    if (problem.isValidTile(check.x, check.y)) // TODO: Was the else on this line correct? Probably not, since what if there are illegal moves but this move isn't colliding with them?
                    {
                        hAfter = problem.GetSingleAgentShortestPath(allAgentsState[i].agent.agentNum, check.x, check.y);

                        if (hBefore != 0)
                            singleAgentFDifferences[i][(int)check.direction] = (byte)(hAfter - hBefore + 1); // h difference + g difference
                        else if (hAfter != 0) // If agent moved from its goal we must count and add all the steps it was stationed at the goal, since they're now part of its g difference
                            singleAgentFDifferences[i][(int)check.direction] = (byte)(hAfter - hBefore + makespan - allAgentsState[i].arrivalTime + 1);
                        else
                            singleAgentFDifferences[i][(int)check.direction] = 0;
                    }
                    else
                    {
                        singleAgentFDifferences[i][(int)check.direction] = byte.MaxValue;
                    }
                }
            }

            // Sum across all agents of max F difference by a legal move
            this.maxFChange = singleAgentFDifferences.Select<byte[], byte>(
                        agentFDifferences => agentFDifferences.Where<byte>(
                            x => (x < byte.MaxValue)
                        ).Max<byte>()
                   ).Max<byte>();

            fLookup = new sbyte[allAgentsState.Length][];
            for (int i = 0; i < fLookup.Length; i++)
            {
                fLookup[i] = new sbyte[this.maxFChange + 1]; // Towards the last agents most of the row will be wasted (the last one can one do F change of 0 or 1),
                                                             // but it's easier than fiddling with array sizes
            }
        }

        /// <summary>
        /// Returns whether all possible f values were generated from this node already
        /// </summary>
        /// <returns></returns>
        public bool hasMoreChildren()
        {
            return this.targetFChange <= this.maxFChange;
        }
        
        public bool isAlreadyExpanded()
        {
            return alreadyExpanded;
        }

        public bool hasChildrenForCurrentF(int agentNum=0)
        {
            return existsChildForF(agentNum, this.remainingFChange);
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
                if (singleAgentFDifferences[agentNum][direction] > remainingTargetFChange) // Small optimization - no need to make the recursive call just to request a negative target from it and get false (because we assume the heuristic function is consistent)
                    continue;
                if (existsChildForF(agentNum + 1, (byte)(remainingTargetFChange - singleAgentFDifferences[agentNum][direction])))
                {
                    fLookup[agentNum][remainingTargetFChange] = 1;
                    return true;
                }
            }
            fLookup[agentNum][remainingTargetFChange] = -1;
            return false;
        }

        /// <summary>
        /// An agent was moved between calculating the singleAgentFDifferences and this call. Using the data that describes its F change potential before the move.
        /// </summary>
        /// <param name="agentIndex"></param>
        public void UpdateRemainingFChange(int agentIndex) {
            byte fChangeFromLastMove = this.singleAgentFDifferences[agentIndex][(int)this.allAgentsState[agentIndex].last_move.direction];
            if (fChangeFromLastMove != byte.MaxValue && this.remainingFChange >= fChangeFromLastMove)
                this.remainingFChange -= fChangeFromLastMove;
            else
                this.remainingFChange = byte.MaxValue;
        }
    }
}
