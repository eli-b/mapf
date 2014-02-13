using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experiment
{
    class WorldStateForPartialExpansion : WorldState
    {

        public byte currentFChange;
        public bool allReadyExpanded;

          /// <summary>
        /// Create a state with the given state for every agent.
        /// </summary>
        /// <param name="allAgentsState"></param>
        public WorldStateForPartialExpansion(AgentState[] allAgentsState): base(allAgentsState)
        {
            allReadyExpanded = false;
            currentFChange = 0;
        }

        /// <summary>
        /// Copy constructor
        /// </summary>
        /// <param name="cpy"></param>
        public WorldStateForPartialExpansion(WorldState cpy)
            : base(cpy)
        {
            allReadyExpanded = false;
            currentFChange = 0;
        }

        public byte[][] getSingleAgentMoves(ProblemInstance problem)
        {

            byte[][] singleAgentMoves = new byte[allAgentsState.Length][]; // [0] - agent number [1] - direction [in table]- effecte on F) 


            //init
            for (int i = 0; i < singleAgentMoves.Length; i++)
            {
                singleAgentMoves[i] = new byte[5];
            }

            int hBefore, hAfter, deltaX, deltaY, pos_X, pos_Y;


            //set values
            TimedMove check = new TimedMove();
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                pos_X = allAgentsState[i].pos_X;
                pos_Y = allAgentsState[i].pos_Y;
                hBefore = problem.GetSingleAgentShortestPath(allAgentsState[i].agent.agentNum, pos_X, pos_Y);

                for (byte direction = 0; direction < 5; direction++)
                {
                    deltaX = WorldState.operators[direction, 0];
                    deltaY = WorldState.operators[direction, 1];
                    if (problem.parameters.ContainsKey(Trevor.ILLEGAL_MOVES_KEY))
                    {
                        HashSet<TimedMove> reserved = (HashSet<TimedMove>)(problem.parameters[Trevor.ILLEGAL_MOVES_KEY]);
                        check.setup(pos_X + deltaX, pos_Y + deltaY, -1, g + 1);
                        if (reserved.Contains(check))
                        {
                            singleAgentMoves[i][direction] = byte.MaxValue;
                            continue;
                        }
                        check.direction = direction;
                        check.setOppositeMove();
                        if (reserved.Contains(check))
                        {
                            singleAgentMoves[i][direction] = byte.MaxValue;
                            continue;
                        }
                    }
                    else if (problem.isValidTile(pos_X + deltaX, pos_Y + deltaY))
                    {
                        hAfter = problem.GetSingleAgentShortestPath(allAgentsState[i].agent.agentNum, pos_X + deltaX, pos_Y + deltaY);

                        if (hBefore != 0)
                            singleAgentMoves[i][direction] = (byte)(hAfter - hBefore + 1);
                        else if (hAfter != 0) //if agent moved from its goal we must count and add all the steps he was stationd at the goal
                            singleAgentMoves[i][direction] = (byte)(hAfter - hBefore + makespan - allAgentsState[i].arrivalTime + 1);
                        else
                            singleAgentMoves[i][direction] = 0;

                    }
                    else
                    {
                        singleAgentMoves[i][direction] = byte.MaxValue;
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
        /// true if moves so far havent changed the F val by more then this step allowes
        /// </summary>
        /// <param name="movesList"></param>
        /// <param name="agentForNextStep"></param>
        /// <returns></returns>
        public bool hasMoreChildren(int maxFchange)
        {
            return this.currentFChange <= maxFchange;
        }
        
        public bool isAllReadyExpanded()
        {
            return allReadyExpanded;
        }

        override public int conflictsCount(HashSet<TimedMove> conflictAvoidence)
        {
            TimedMove check = new TimedMove();
            int ans = 0;
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                check.setup(allAgentsState[i].pos_X,allAgentsState[i].pos_Y,-1,allAgentsState[i].currentStep);
                if (conflictAvoidence.Contains(check))
                    ans++;
                check.direction = allAgentsState[i].direction;
                check.setOppositeMove();
                if (conflictAvoidence.Contains(check))
                    ans++;
            }
            return ans;
        }

    }
}
