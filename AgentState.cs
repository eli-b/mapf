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
        /// <summary>
        /// Only used by AStarForSingleAgent, which should itself be deleted.
        /// </summary>
        [NonSerialized] public AgentState prev;
        /// <summary>
        /// For CBS this must be set to false.
        /// </summary>
        public static bool EquivalenceOverDifferentTimes = true;

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
            this.lastMove = copy.lastMove;
            //this.prev = copy;
            this.g = copy.g;
        }

        /// <summary>
        /// Only used by EnumeratedPDB - check if can be removed
        /// </summary>
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

            bool isWait = move.direction == Move.Direction.Wait;
            bool atGoal = this.AtGoal();

            // If performed a non WAIT move and reached the agent's goal - store the arrival time
            if (atGoal && (isWait == false))
                this.arrivalTime = move.time;

            if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG)
            {
                if (this.AtGoal())
                    this.g = this.arrivalTime;
                else
                    this.g = this.lastMove.time;
            }
            else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
            {
                if ((atGoal && isWait) == false)
                    this.g += 1;
            }
        }

        /// <summary>
        /// Checks if the agent is at its goal location
        /// </summary>
        /// <returns>True if the agent has reached its goal location</returns>
        public bool AtGoal()
        {
            return this.agent.Goal.Equals(this.lastMove); // Comparing Move to TimedMove is allowed, the reverse isn't.
        }

        public int g;

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        public int GetIndexInHeap() { return binaryHeapIndex; }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        public void SetIndexInHeap(int index) { binaryHeapIndex = index; }

        /// <summary>
        /// When equivalence over different times is necessary,
        /// checks this.agent and last position only,
        /// ignoring data that would make this state different to other equivalent states:
        /// It doesn't matter from which direction the agent got to its current location.
        /// It's also necessary to ignore the agents' move time - we want the same positions
        /// in any time to be equivalent.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            AgentState that = (AgentState)obj;

            if (AgentState.EquivalenceOverDifferentTimes)
            {
                return this.agent.Equals(that.agent) &&
                       this.lastMove.x == that.lastMove.x && 
                       this.lastMove.y == that.lastMove.y; // Ignoring the time and the direction
            }
            else
            {
                return this.agent.Equals(that.agent) &&
                       this.lastMove.x == that.lastMove.x &&
                       this.lastMove.y == that.lastMove.y &&
                       this.lastMove.time == that.lastMove.time; // Ignoring the direction
            }
        }

        /// <summary>
        /// When equivalence over different times is necessary,
        /// uses this.agent and last position only, ignoring direction and time.
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                if (AgentState.EquivalenceOverDifferentTimes)
                    return 3 * this.agent.GetHashCode() + 5 * this.lastMove.x + 7 * this.lastMove.y;
                else
                    return 3 * this.agent.GetHashCode() + 5 * this.lastMove.GetHashCode();
            }
        }

        public Move GetMove()
        {
            return this.lastMove;
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

            // TODO: Prefer goal nodes.

            // Prefer larger g:
            if (this.lastMove.time < that.lastMove.time)
                return 1;
            if (this.lastMove.time > that.lastMove.time)
                return -1;
            return 0;
        }

        public override string ToString()
        {
            return "step-" + lastMove.time + " position " + this.lastMove;
        }
    }
}
