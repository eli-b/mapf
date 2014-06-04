using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;

namespace CPF_experiment
{
    /// <summary>
    /// Describes a node in the A* search space.
    /// </summary>
    public class WorldState : IComparable<IBinaryHeapItem> , IBinaryHeapItem
    {
        public int makespan; // Total time steps passed, max(agent makespans)
        public int g; // Sum of agent makespans until they reach their goal
        public int h;
        public AgentState[] allAgentsState;
        public WorldState prevStep;
        private int binaryHeapIndex;
        public int potentialConflictsCount;
        public int cbsInternalConflictsCount;
        public int minDepth;
        /// <summary>
        /// The last move of all agents that have already moved in this turn.
        /// Used for making sure the next agent move doesn't collide with moves already made.
        /// </summary>
        public HashSet<TimedMove> currentMoves;
        protected Plan plan;
        protected static readonly int NOT_SET = -1;
        /// <summary>
        /// For computing expansion delay
        /// </summary>
        public int expandedCountWhenGenerated;
        /// <summary>
        /// For lazy heuristics
        /// </summary>
        public CBS_LocalConflicts cbsState;

        /// <summary>
        /// Create a state with the given state for every agent.
        /// </summary>
        /// <param name="allAgentsState"></param>
        public WorldState(AgentState[] allAgentsState, int minDepth = -1)
        {
            this.allAgentsState = allAgentsState.ToArray<AgentState>();
            this.makespan = allAgentsState.Max<AgentState>(state => state.lastMove.time); // We expect to only find at most two G values within the agent group
            this.CalculateG(); // G not necessarily zero when solving a partially solved problem.
            this.potentialConflictsCount = 0;
            this.cbsInternalConflictsCount = 0;
            this.minDepth = minDepth;
            this.currentMoves = new HashSet<TimedMove>();
            this.goalCost = NOT_SET;
        }

        /// <summary>
        /// Copy constructor.
        /// </summary>
        /// <param name="cpy"></param>
        public WorldState(WorldState cpy)
        {
            this.makespan = cpy.makespan;
            this.g = cpy.g;
            this.h = cpy.h;
            // The potentialConflictsCount and cbsInternalConflictsCount are only copied later if necessary.
            this.minDepth = cpy.minDepth;
            this.allAgentsState = new AgentState[cpy.allAgentsState.Length];
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                this.allAgentsState[i] = new AgentState(cpy.allAgentsState[i]); // Shallow copy - it's still the same lastMove inside.
            }
            this.currentMoves = new HashSet<TimedMove>(cpy.currentMoves);
            this.goalCost = NOT_SET;
        }

        /// <summary>
        /// Creates a new state by extracting a subset of the agents from
        /// the original Travor_WorldState. We overload the constructor because
        /// while building our pattern database, we rewrite the problem and
        /// therefore need to make a deep copy of the state data structures so
        /// as to not overwrite the original problem. The ultimate solution
        /// would be to rework the code to remove static variables so that we
        /// can instantiate subproblems without affecting the original data
        /// structures.
        /// </summary>
        /// <param name="allAgentsState">A set of agent states in the original problem.</param>
        /// <param name="vAgents">A list of indices referring to the subset of agents we want to extract.</param>
        public WorldState(AgentState[] allAgentsState, List<uint> vAgents)
            // Copy specified agents only
            : this(vAgents.Select<uint, AgentState>(index => new AgentState(allAgentsState[index])).ToArray<AgentState>())
        {}
        
        public bool GoalTest()
        {
            if (makespan >= this.minDepth)
            {
                if (this.plan != null) // If we know the optimal solution, it doesn't matter if this is a goal node or not, we can finish.
                    return true;
                
                return h == 0; // That's crazy! A node that is close to the goal might also get h==0.
                               // Our specific heuristic doesn't behave that way, though.
                               // FIXME: Implement a proper goal test and use it when h==0.
            }
            return false;
        }

        /// <summary>
        /// Set the optimal solution of this node as a problem instance.
        /// </summary>
        /// <param name="solution"></param>
        public virtual void SetSolution(Plan solution)
        {
            this.plan = new Plan(this); // This node may be a partial solution itself, need to start from the real root.
            this.plan.ContinueWith(solution);
        }

        /// <summary>
        /// Returns the optimal plan to the goal through this node, if this is a goal node (of any kind),
        /// else returns the optimal plan to this node.
        /// </summary>
        /// <returns></returns>
        public Plan GetPlan()
        {
            if (this.plan == null)
                this.plan = new Plan(this);
            return this.plan;
        }

        protected int goalCost;

        /// <summary>
        /// Returns the optimal cost to the goal from the start through this node.
        /// </summary>
        /// <returns></returns>
        public int GetGoalCost()
        {
            Debug.Assert(this.GoalTest(), "Only call for goal nodes!");

            if (goalCost == NOT_SET) // This is just a proper goal
            {
                return this.g;
            }
            else
                return this.goalCost;
        }

        /// <summary>
        /// Set the optimal cost from the start to the goal through this node
        /// </summary>
        /// <param name="cost"></param>
        public void SetGoalCost(int cost)
        {
            this.goalCost = cost;
        }

        public int[] GetSingleCosts()
        {
            return allAgentsState.Select<AgentState, int>(agent => agent.g).ToArray<int>(); // FIXME! Doesn't account for generalised goal states!
        }

        protected virtual WorldState Parent()
        {
            return this.prevStep;
        }

        /// <summary>
        /// Used when WorldState objects are put in the open list priority queue
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public virtual int CompareTo(IBinaryHeapItem other)
        {
            WorldState that = (WorldState)other;
            if (this.h + this.g < that.h + that.g)
                return -1;
            if (this.h + this.g > that.h + that.g)
                return 1;

            // Tie breaking:
            bool thisIsGoal = this.GoalTest();
            bool thatIsGoal = that.GoalTest();
            if (thisIsGoal == true && thatIsGoal == false) // The elaborate form is necessary to keep the comparison consistent. Otherwise goalA<goalB and goalB<goalA
                return -1;
            if (thatIsGoal == true && thisIsGoal == false)
                return 1;

            if (this.potentialConflictsCount < that.potentialConflictsCount)
                return -1;
            if (this.potentialConflictsCount > that.potentialConflictsCount)
                return 1;

            if (this.cbsInternalConflictsCount < that.cbsInternalConflictsCount)
                return -1;
            if (this.cbsInternalConflictsCount > that.cbsInternalConflictsCount)
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
            g = allAgentsState.Sum<AgentState>(agent => agent.g);
        }

        public override string ToString()
        {
            string ans = "makespan: " + makespan + ", h: " + h + ", g: " + g + "\n";
            foreach (AgentState temp in allAgentsState)
            {
                ans +=" agent " + temp.agent.agentNum + ": " + temp.lastMove + "\n";
            }
            return ans;
        }

        /// <summary>
        /// Returns a copy of the last move of all the agents in this state.
        /// Why a copy?
        /// </summary>
        /// <returns>A list of points</returns>
        public List<Move> GetAgentsMoves()
        {
            var ans = new List<Move>();
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                ans.Add(new Move(allAgentsState[i].lastMove));
            }
            return ans;
        }

        public Move GetSingleAgentMove(int index)
        {
            return new Move(allAgentsState[index].lastMove);
        }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        /// <returns></returns>
        public int GetIndexInHeap() { return binaryHeapIndex; }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        /// <returns></returns>
        public void SetIndexInHeap(int index) { binaryHeapIndex = index; }

        /// <summary>
        /// Checks for internal conflicts
        /// </summary>
        /// <returns></returns>
        public bool isValid()
        {
            for (int i = 0; i < this.allAgentsState.Length; i++)
            {
                for (int j = i+1; j < this.allAgentsState.Length; j++)
                {
                    // Internal conflict
                    if (this.allAgentsState[i].lastMove.IsColliding(this.allAgentsState[j].lastMove))
                        return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Only the agent states are used in the hash.
        /// The g, makespan, h, potentialConflictsCount, cbsInternalConflictsCount and others are ignored, as neccesary.
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            int ans = 0;
            unchecked
            {
                for (int i = 0 ; i < allAgentsState.Length; i++)
                {
                    ans += allAgentsState[i].GetHashCode() * Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length];
                }
            }
            return ans;
        }

        /// <summary>
        /// Only the AgentStates are compared.
        /// g, makespan, h, potentialConflictsCount, cbsInternalConflictsCount and others are ignored, as necessary.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            WorldState that = (WorldState)obj;
            return this.allAgentsState.SequenceEqual<AgentState>(that.allAgentsState);
        }

        public virtual int ConflictsCount(ICollection<TimedMove> conflictAvoidance)
        {
            int ans = 0;
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                if (allAgentsState[i].lastMove.IsColliding(conflictAvoidance))
                    ans++; // Assuming there are no collision within the table, it's correct to only add 1.
            }
            return ans;
        }

        public virtual ProblemInstance ToProblemInstance(ProblemInstance initial)
        {
            // Notice this is not a subproblem in the number of agents but
            // in the steps from the start.
            // It might even be harder if the steps were away from the goal.
            return initial.Subproblem(this.allAgentsState);
        }
    }
}
