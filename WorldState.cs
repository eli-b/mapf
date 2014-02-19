using System;
using System.Collections.Generic;

namespace CPF_experiment
{
    /// <summary>
    /// Describes a node in the A* search space.
    /// </summary>
    public class WorldState : IComparable<IBinaryHeapItem> , IBinaryHeapItem
    {
        /// <summary>
        /// A list of allowed operators (currently this is a 4-connected grid, so there are 4 moves and a STAY option).
        /// Each tuple is composed of 
        /// 1. The delta that should be added to the x coordinate
        /// 2. The delta that should be added to the y coordinate 
        /// 3. A code that represents the direction of the move (used to discover head-on collisions)
        /// </summary>
        static public readonly int[,] operators = {{0,  0, (int)Move.Direction.Wait},
                                                   {-1, 0, (int)Move.Direction.North},
                                                   {0,  1, (int)Move.Direction.East},
                                                   {1,  0, (int)Move.Direction.South},
                                                   {0, -1, (int)Move.Direction.West}};
        // TODO are the above really necessary?

        public int makespan; // Total time steps passed, max(agent makespans)
        public int g; // Sum of agent makespans until they reach their goal
        public int h;
        public bool notExpanded;
        public AgentState[] allAgentsState;
        public WorldState prevStep;
        private int binaryHeapIndex;
        public int potentialConflictsCount;
        public int dncInternalConflictsCount;
        public byte nextFvalue;

        /// <summary>
        /// Create a state with the given state for every agent.
        /// </summary>
        /// <param name="allAgentsState"></param>
        public WorldState(AgentState[] allAgentsState)
        {
            this.allAgentsState = allAgentsState;
            makespan = 0;
            g = 0;
            potentialConflictsCount = 0;
            dncInternalConflictsCount = 0;
            notExpanded = true;
        }

        /// <summary>
        /// Copy constructor
        /// </summary>
        /// <param name="cpy"></param>
        public WorldState(WorldState cpy)
        {
            this.makespan = cpy.makespan;
            this.h = cpy.h;
            this.allAgentsState = new AgentState[cpy.allAgentsState.Length];
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                this.allAgentsState[i] = new AgentState(cpy.allAgentsState[i]);
            }
            notExpanded = true;
        }

        /// <summary>
        /// copies a state with only a sub group of the original agents
        /// </summary>
        /// <param name="cpy"></param>
        /// <param name="numOfAgents"></param>
        public WorldState(WorldState cpy, int numOfAgents)
        {
            this.allAgentsState = new AgentState[numOfAgents];
            for (int i = 0; i < numOfAgents; i++)
            {
                this.allAgentsState[i] = new AgentState(cpy.allAgentsState[i]);
            }
            notExpanded = true;
        }


        /// <summary>
        /// Make a move object with the given operator.
        /// </summary>
        /// <param name="opIndex">The index of the operator (corresponding to the first index of WorldState.operators) </param>
        /// <param name="state">The state of a single agent that want to perform the move</param>
        /// <returns></returns>
        public static Move MakeMove(int opIndex, AgentState state)
        {
            int deltaX = WorldState.operators[opIndex, 0];
            int deltaY = WorldState.operators[opIndex, 1];
            return  new Move(state.pos_X + deltaX, state.pos_Y + deltaY, WorldState.operators[opIndex, 2]);
        }

        /// <summary>
        /// Creates a new state by extracting a subset of the agents from
        /// the original Travor_WorldState. We overload the constructor because
        /// while building our pattern databass, we rewrite the problem and
        /// therefore need to make a deep copy of the state data structures so
        /// as to not overwrite the original problem. The ultimate solution
        /// would be to rework the code to remove static variables so that we
        /// can instantiate subproblems without affecting the original data
        /// structures.
        /// </summary>
        /// <param name="allAgentsStatesrc">A set of agent states in the original problem.</param>
        /// <param name="vAgents">A list of indices referring to the subset of agents we want to extract.</param>

        public WorldState(AgentState[] allAgentsState, List<uint> vAgents)
        {
            this.allAgentsState = new AgentState[vAgents.Count];
            int nCurrent = 0;
            foreach (uint a in vAgents)
                this.allAgentsState[nCurrent++] = new AgentState(allAgentsState[a]);
            makespan = 0;
            g = 0;
        }        
        
        public Boolean GoalTest(int minDepth)
        {
            if (makespan >= minDepth)
                return h == 0;
            return false;
        }

        public virtual int CompareTo(IBinaryHeapItem other)
        {
            WorldState that = (WorldState)other;
            if (this.h + this.g < that.h + that.g)
                return -1;
            if (this.h + this.g > that.h + that.g)
                return 1;

            if (this.potentialConflictsCount < that.potentialConflictsCount)
                return -1;
            if (this.potentialConflictsCount > that.potentialConflictsCount)
                return 1;

            if (this.dncInternalConflictsCount < that.dncInternalConflictsCount)
                return -1;
            if (this.dncInternalConflictsCount > that.dncInternalConflictsCount)
                return 1;

            if (this.g < that.g)
                return 1;
            if (this.g > that.g)
                return -1;

            return 0;
        }
        

        /// <summary>
        /// Calculate and set the g of the state as the sum of the different agent g values.
        /// </summary>
        public virtual void CalculateG()
        {
            g = 0;
            foreach (AgentState singleAgentState in allAgentsState)
            {
                if (singleAgentState.atGoal())
                    g += singleAgentState.arrivalTime;
                else
                    g += makespan;
            }
        }


        public override string ToString()
        {
            string ans = "makespan: "+makespan+" h: "+h+" g: "+g+"\n";
            foreach (AgentState temp in allAgentsState)
            {
                ans +=" agent" + temp.agent.agentNum + ": (" + temp.pos_X + "," + temp.pos_Y + ")\n";
            }
            return ans;
        }


        /// <summary>
        /// Returns the last move of all the agents in this state
        /// </summary>
        /// <returns>A list of points</returns>
        public LinkedList<Move> GetAgentsMoves()
        {
            LinkedList<Move> ans = new LinkedList<Move>();
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                ans.AddFirst(new Move(allAgentsState[i].pos_X, allAgentsState[i].pos_Y, allAgentsState[i].direction));
            }
            return ans;
        }

        public Move getSingleAgentMove(int index)
        {
            return new Move(allAgentsState[index].pos_X, allAgentsState[index].pos_Y, allAgentsState[index].direction);
        }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        /// <returns></returns>
        public int getIndexInHeap() { return binaryHeapIndex; }
        /// <summary>
        /// BH_Iteam implimatation
        /// </summary>
        /// <returns></returns>
        public void setIndexInHeap(int index) { binaryHeapIndex = index; }

        /// <summary>
        /// true if given a valid successor
        /// </summary>
        /// <param name="nextStep"></param>
        /// <returns></returns>
        public bool isValidSuccessor(WorldState nextStep)
        {
            if (nextStep == null)
                return false;
            for (int i = 0; i < this.allAgentsState.Length; i++)
            {
                for (int j = i+1; j < nextStep.allAgentsState.Length; j++)
                {
                    if (nextStep.allAgentsState[i].pos_X == nextStep.allAgentsState[j].pos_X && nextStep.allAgentsState[i].pos_Y == nextStep.allAgentsState[j].pos_Y)
                        return false;
                    if (this.allAgentsState[i].pos_X == nextStep.allAgentsState[j].pos_X && this.allAgentsState[i].pos_Y == nextStep.allAgentsState[j].pos_Y)
                        if (this.allAgentsState[j].pos_X == nextStep.allAgentsState[i].pos_X && this.allAgentsState[j].pos_Y == nextStep.allAgentsState[i].pos_Y)
                             return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Returns a hash value for the given state (used in Hash based data structures).
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            int ans = 0;
            unchecked
            {
                for (int i = 0; i < allAgentsState.Length; i++)
                {
                    ans += allAgentsState[i].pos_X * Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length];
                    ans += allAgentsState[i].pos_Y * Constants.PRIMES_FOR_HASHING[(i + 10) % Constants.PRIMES_FOR_HASHING.Length];
                }
            }
            return ans;
        }


        /// <summary>
        /// Currently returns false even if this is the same location and smaller g.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            WorldState that = (WorldState)obj;
            if (this.allAgentsState.Length != that.allAgentsState.Length)
                return false;
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                if (!this.allAgentsState[i].Equals(that.allAgentsState[i]))
                    return false;
            }
            return true;
        }
        
        public virtual int conflictsCount(HashSet<TimedMove> conflictAvoidence)
        {
            int ans = 0;
            TimedMove check = new TimedMove();
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                check.setup(allAgentsState[i].pos_X, allAgentsState[i].pos_Y, -1, allAgentsState[i].currentStep);
                if (conflictAvoidence.Contains(check))
                    ans++;
                check.direction = allAgentsState[i].direction;
                check.setOppositeMove();
                if (conflictAvoidence.Contains(check))
                    ans++;
            }
            return ans;
        }

        public virtual int conflictsCount(HashSet_U<TimedMove> conflictAvoidence)
        {
            int ans = 0;
            TimedMove check = new TimedMove();
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                check.setup(allAgentsState[i].pos_X, allAgentsState[i].pos_Y, -1, allAgentsState[i].currentStep);
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
