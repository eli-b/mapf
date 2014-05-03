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
        /// <summary>
        /// The last move's time is the agent's G
        /// </summary>
        public TimedMove lastMove;
        private int binaryHeapIndex;
        public int potentialConflicts;
        public ushort potentialConflictsID;
        [NonSerialized] public AgentState prev;

        public AgentState(int pos_X, int pos_Y, Agent agent)
        {
            this.lastMove = new TimedMove(pos_X, pos_Y, Move.Direction.NO_DIRECTION, 0);
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
            this.lastMove = new TimedMove(copy.lastMove); // Can we just do this.lastMove = copy.lastMove? I think we can now, since MoveTo replaces the move
        }

        public void SwapCurrentWithGoal()
        {
            int nTemp = lastMove.x;
            lastMove.x = agent.Goal.x;
            agent.Goal.x = nTemp;
            nTemp = lastMove.y;
            lastMove.y = agent.Goal.y;
            agent.Goal.y = nTemp;
        }

        /// <summary>
        /// Updates the agent's last move with the given move and sets arrivalTime (at goal) if necessary.
        /// </summary>
        public void MoveTo(TimedMove move)
        {
            this.lastMove = move;

            // If performed a non WAIT move and reached the agent's goal - store the arrival time
            if ((move.direction != Move.Direction.Wait) && (this.AtGoal()))
                this.arrivalTime = lastMove.time;
        }

        /// <summary>
        /// Checks if the agent is at its goal location
        /// </summary>
        /// <returns>True if the agent has reached its goal location</returns>
        public bool AtGoal()
        {
            return this.agent.Goal.Equals(this.lastMove); // Comparing Move to TimedMove is allowed, the reverse isn't.
        }

        public int g
        {
            get
            {
                if (this.AtGoal())
                    return this.arrivalTime;
                else
                    return this.lastMove.time;
            }
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
        /// Checks lastMove and agent
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            AgentState that = (AgentState)obj;
            
            if (this.lastMove.Equals(that.lastMove) && this.agent.Equals(that.agent))
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// Uses lastMove and agent.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                return 3 * this.agent.GetHashCode() + 5 * this.lastMove.GetHashCode(); // Behavior change: now uses currentStep too.
            }
        }

        public Move GetMove()
        {
            return new Move(this.lastMove);
        }

        /// <summary>
        /// Used when AgentState objects are put in the open list priority queue - mainly in AStarForSingleAgent, I think.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public int CompareTo(IBinaryHeapItem other)
        {
            AgentState that = (AgentState)other;
            if (this.h + this.lastMove.time < that.h + that.lastMove.time)
                return -1;
            if (this.h + this.lastMove.time > that.h + that.lastMove.time)
                return 1;

            if (this.potentialConflictsID < that.potentialConflictsID)
                return -1;
            if (this.potentialConflictsID > that.potentialConflictsID)
                return 1;

            if (this.potentialConflicts < that.potentialConflicts) // Doesn't this come before the potentialConflictsID in other places?
                return -1;
            if (this.potentialConflicts > that.potentialConflicts)
                return 1;

            if (this.lastMove.time < that.lastMove.time)
                return 1;
            if (this.lastMove.time > that.lastMove.time) // Prefer larger g
                return -1;
            return 0;
        }

        public override string ToString()
        {
            return "step-" + lastMove.time + " position " + this.lastMove;
        }
    }
}
