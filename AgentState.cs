using System;
using System.Collections.Generic;

namespace CPF_experiment
{
    [Serializable] public class AgentState : IComparable<IBinaryHeapItem>, IBinaryHeapItem
    {
        /// <summary>
        /// Only used when AgentState objects are put in the open list priority queue - mainly in AStarForSingleAgent, I think.
        /// </summary>
        public int h;
        public Agent agent;
        /// <summary>
        /// At goal
        /// </summary>
        public int arrivalTime;
        public TimedMove last_move;
        private int binaryHeapIndex;
        public int potentialConflicts;
        public ushort potentialConflictsID;
        [NonSerialized] public AgentState prev;

        public AgentState(int pos_X, int pos_Y, Agent agent)
        {
            this.last_move = new TimedMove(pos_X, pos_Y, Move.Direction.NO_DIRECTION, 0);
            this.agent = agent;
        }

        public AgentState(int startX, int startY, int goalX, int goalY, int agentId)
            : this(startX, startY, new Agent(goalX, goalY, agentId))
        {}

        public AgentState(AgentState copy)
        {
            this.agent = copy.agent;
            this.h = copy.h;
            this.arrivalTime = copy.arrivalTime;
            this.last_move = new TimedMove(copy.last_move); // Can we just do this.last_move = copy.last_move?
        }

        public void swapCurrentWithGoal()
        {
            int nTemp = last_move.x;
            last_move.x = agent.Goal.x;
            agent.Goal.x = nTemp;
            nTemp = last_move.y;
            last_move.y = agent.Goal.y;
            agent.Goal.y = nTemp;
        }

        /// <summary>
        /// Updates the agent's last move with the given move and set arrivalTime (at goal) if necessary.
        /// </summary>
        public void move(TimedMove move)
        {
            last_move = move;

            // If performed a non STAY move and reached the agent's goal - store the arrival time
            if ((move.direction != Move.Direction.Wait) && (this.atGoal()))
                this.arrivalTime = last_move.time;
        }

        /// <summary>
        /// Checks if the agent is at its goal location
        /// </summary>
        /// <returns>True if the agent has reached its goal location</returns>
        public bool atGoal()
        {
            return this.agent.Goal.Equals(this.last_move); // Comparing Move to TimedMove is allowed, the reverse isn't.
        }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        public int GetIndexInHeap() { return binaryHeapIndex; }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        public void SetIndexInHeap(int index) { binaryHeapIndex = index; }

        /// <summary>
        /// Checks last_move and agent
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            AgentState that = (AgentState)obj;
            
            if (this.last_move.Equals(that.last_move) /*behavior change! now checks time too*/ && this.agent.Equals(that.agent))
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// Uses last_move and agent.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                return 3 * this.agent.GetHashCode() + 5 * this.last_move.GetHashCode(); // Behavior change: now uses currentStep too.
            }
        }

        public Move GetMove()
        {
            return new Move(this.last_move);
        }

        /// <summary>
        /// Used when AgentState objects are put in the open list priority queue - mainly in AStarForSingleAgent, I think.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public int CompareTo(IBinaryHeapItem other)
        {
            AgentState that = (AgentState)other;
            if (this.h + this.last_move.time < that.h + that.last_move.time)
                return -1;
            if (this.h + this.last_move.time > that.h + that.last_move.time)
                return 1;

            if (this.potentialConflictsID < that.potentialConflictsID)
                return -1;
            if (this.potentialConflictsID > that.potentialConflictsID)
                return 1;

            if (this.potentialConflicts < that.potentialConflicts) // Doesn't this come before the potentialConflictsID in other places?
                return -1;
            if (this.potentialConflicts > that.potentialConflicts)
                return 1;

            if (this.last_move.time < that.last_move.time)
                return 1;
            if (this.last_move.time > that.last_move.time) // Prefer larger g
                return -1;
            return 0;
        }

        public override string ToString()
        {
            return " step-" + last_move.time + " position " + this.last_move;
        }
    }

    ///<summary>
    /// Compares two AgentStates according to their AgentNum
    /// This method is used for Rtrevor
    ///</summary>
    class AgentsNumsComparator : IComparer<AgentState>
    {
        public int Compare(AgentState x, AgentState y)
        {
            return x.agent.agentNum.CompareTo(y.agent.agentNum);
        }
    }
}
