using System.Collections.Generic;

namespace CPF_experiment
{
    class WorldStateForPartialExpansion : WorldState
    {
        public byte currentFChange;
        public bool alreadyExpanded;
        public byte nextFvalue;
        public bool notExpanded;

        /// <summary>
        /// Create a state with the given state for every agent.
        /// </summary>
        /// <param name="allAgentsState"></param>
        public WorldStateForPartialExpansion(AgentState[] allAgentsState): base(allAgentsState)
        {
            alreadyExpanded = false;
            this.notExpanded = true;
            currentFChange = 0;
        }

        /// <summary>
        /// Copy constructor
        /// </summary>
        /// <param name="cpy"></param>
        public WorldStateForPartialExpansion(WorldState cpy)
            : base(cpy)
        {
            alreadyExpanded = false;
            currentFChange = 0;
            this.notExpanded = true; // Not cpy.notExpanded?
        }

        public byte[][] getSingleAgentMoves(ProblemInstance problem)
        {

            byte[][] singleAgentMoves = new byte[allAgentsState.Length][]; // [0] - agent number [1] - direction [in table]- effecte on F) 


            //init
            for (int i = 0; i < singleAgentMoves.Length; i++)
            {
                singleAgentMoves[i] = new byte[Move.NUM_NON_DIAG_MOVES];
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
                            singleAgentMoves[i][(int)check.direction] = byte.MaxValue;
                            continue;
                        }
                    }
                    else if (problem.isValidTile(check.x, check.y))
                    {
                        hAfter = problem.GetSingleAgentShortestPath(allAgentsState[i].agent.agentNum, check.x, check.y);

                        if (hBefore != 0)
                            singleAgentMoves[i][(int)check.direction] = (byte)(hAfter - hBefore + 1);
                        else if (hAfter != 0) //if agent moved from its goal we must count and add all the steps it was stationed at the goal
                            singleAgentMoves[i][(int)check.direction] = (byte)(hAfter - hBefore + makespan - allAgentsState[i].arrivalTime + 1);
                        else
                            singleAgentMoves[i][(int)check.direction] = 0;
                    }
                    else
                    {
                        singleAgentMoves[i][(int)check.direction] = byte.MaxValue;
                    }
                }
            }
            return singleAgentMoves;
        }

        public int getMaxFchange(byte[][] singleAgentMoves)
        {
            int maxF;
            int maxFchange = 0;
            for (int agent = 0; agent < singleAgentMoves.Length; agent++)
            {
                maxF = 0;
                for (int direction = 0; direction < singleAgentMoves[agent].Length; direction++)
                {
                    if (singleAgentMoves[agent][direction] < byte.MaxValue && singleAgentMoves[agent][direction] > maxF)
                        maxF = singleAgentMoves[agent][direction];
                }
                maxFchange += maxF;
            }
            return maxFchange;
        }

        /// <summary>
        /// true if moves so far haven't changed the F val by more than this step allows
        /// </summary>
        /// <param name="movesList"></param>
        /// <param name="agentForNextStep"></param>
        /// <returns></returns>
        public bool hasMoreChildren(int maxFchange)
        {
            return this.currentFChange <= maxFchange;
        }
        
        public bool isAlreadyExpanded()
        {
            return alreadyExpanded;
        }

        override public int conflictsCount(HashSet<TimedMove> conflictAvoidence)
        {
            int ans = 0;
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                if (allAgentsState[i].last_move.isColliding(conflictAvoidence)) // Behavior change: this didn't check for head-on collisions
                    ans++;
            }
            return ans;
        }
    }
}
