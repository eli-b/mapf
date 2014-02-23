using System;
using System.Linq;
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
        // TODO are the above really necessary in addition to Move's tables?

        public int makespan; // Total time steps passed, max(agent makespans)
        public int g; // Sum of agent makespans until they reach their goal
        public int h;
        public bool notExpanded; // Used only by AStarWithPartialExpansionBasic. Consider moving.
        public AgentState[] allAgentsState;
        public WorldState prevStep;
        private int binaryHeapIndex;
        public int potentialConflictsCount;
        public int dncInternalConflictsCount;
        public byte nextFvalue; // Used only by AStarWithPartialExpansionBasic. Consider moving.
        public HashSet<TimedMove> currentMoves;

        /// <summary>
        /// Create a state with the given state for every agent.
        /// </summary>
        /// <param name="allAgentsState"></param>
        public WorldState(AgentState[] allAgentsState)
        {
            this.allAgentsState = allAgentsState;
            this.makespan = 0;
            this.g = 0;
            this.potentialConflictsCount = 0;
            this.dncInternalConflictsCount = 0;
            this.notExpanded = true;
            this.currentMoves = new HashSet<TimedMove>();
        }

        /// <summary>
        /// Copy constructor. Doesn't copy g!
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
            this.notExpanded = true; // Not cpy.notExpanded?
            this.currentMoves = new HashSet<TimedMove>(cpy.currentMoves);
        }

        /// <summary>
        /// Copies a state with only a sub group of the original agents.
        /// Doesn't copy h or makespan!
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
            this.notExpanded = true;
            //TODO: Copy the relevant parts of cpy.currentMoves
        }

        /// <summary>
        /// Make a move object with the given operator.
        /// TODO: Find a better place for this
        /// </summary>
        /// <param name="opIndex">The index of the operator (corresponding to the first index of WorldState.operators) </param>
        /// <param name="state">The state of a single agent that want to perform the move</param>
        /// <returns></returns>
        public static Move MakeMove(int opIndex, AgentState state)
        {
            int deltaX = WorldState.operators[opIndex, 0];
            int deltaY = WorldState.operators[opIndex, 1];
            return new Move(state.pos_X + deltaX, state.pos_Y + deltaY, WorldState.operators[opIndex, 2]);
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
            // Copy specified agents:
            this.allAgentsState = vAgents.Select<uint, AgentState>(index => new AgentState(allAgentsState[index])).ToArray<AgentState>();
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

            // f, conflicts and internal conflicts being equal, prefer nodes with a larger g
            // - they're closer to the goal so less nodes would probably be generated by them on the way to it.
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
            string ans = "makespan: " + makespan + " h: " + h + " g: " + g + "\n";
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
        /// BH_Item implementation
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
                    // This block duplicates logic found in class Move.
                    // Internal conflict in nextStep: two agents with same move target
                    if (nextStep.allAgentsState[i].pos_X == nextStep.allAgentsState[j].pos_X && 
                        nextStep.allAgentsState[i].pos_Y == nextStep.allAgentsState[j].pos_Y)
                        return false;

                    // Internal conflict in nextStep: head-on collision
                    if (this.allAgentsState[i].pos_X == nextStep.allAgentsState[j].pos_X && 
                        this.allAgentsState[i].pos_Y == nextStep.allAgentsState[j].pos_Y &&
                        this.allAgentsState[j].pos_X == nextStep.allAgentsState[i].pos_X && 
                        this.allAgentsState[j].pos_Y == nextStep.allAgentsState[i].pos_Y)
                        return false;

                    // Internal conflicts in nextStep should probably be checked for using one of nextStep's methods.
                    // Not actually checking that nextStep is a successor to this step: that all agents can get from
                    // their current to the next
                }
            }
            return true;
        }

        /// <summary>
        /// Only the agents positions are used in the hash. The directions, g, makespan, h, potentialConflictsCount, dncInternalConflictsCount and others are ignored.
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
        /// Only the AgentStates are compared. g, makespan, h, potentialConflictsCount, dncInternalConflictsCount and others are ignored.
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
