using System.Diagnostics;

namespace mapf
{
    public class CbsConflict
    {
        public int agentAIndex; // Agent index and not agent num since this class is only used to represent internal conflicts
        public int agentBIndex;
        public Move agentAmove;
        public Move agentBmove;
        public int timeStep;
        /// <summary>
        /// If true, conflict is two agents have same dest, from any direction. Otherwise it's an edge conflict.
        /// </summary>
        public bool isVertexConflict;
        public WillCostIncrease willCostIncreaseForAgentA;
        public WillCostIncrease willCostIncreaseForAgentB;

        public enum WillCostIncrease
        {
            MAYBE,
            YES,
            NO
        }

        public CbsConflict(int conflictingAgentAIndex, int conflictingAgentBIndex, Move agentAMove, Move agentBMove, int timeStep)
        {
            this.agentAIndex = conflictingAgentAIndex;
            this.agentBIndex = conflictingAgentBIndex;
            this.agentAmove = agentAMove;
            this.agentBmove = agentBMove;
            this.timeStep = timeStep;
            if (agentAMove.x == agentBMove.x && agentAMove.y == agentBMove.y) // Same dest, from any direction
                this.isVertexConflict = true;
            else
            {
                this.isVertexConflict = false;
                Debug.Assert(Constants.ALLOW_HEAD_ON_COLLISION == false, "Creating an edge conflict when head-on collision are allowed");
            }
            this.willCostIncreaseForAgentA = WillCostIncrease.MAYBE;
            this.willCostIncreaseForAgentB = WillCostIncrease.MAYBE;
        }

        public override string ToString()
        {
            return $"Agent {this.agentAIndex} going {this.agentAmove} collides with agent " +
                   $"{this.agentBIndex} going {this.agentBmove} at time {this.timeStep}";
        }

        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            var other = (CbsConflict)obj;
            if (this.agentAIndex != other.agentAIndex)
                return false;
            if (this.agentBIndex != other.agentBIndex)
                return false;
            if (this.isVertexConflict != other.isVertexConflict)
                return false;
            if (this.timeStep != other.timeStep)
                return false;
            if (this.isVertexConflict)
            { // Compare dests, ignore directions. Enough to compare one agent's move because the other is colliding with it.
                if (this.agentAmove.x != other.agentAmove.x)
                    return false;
                if (this.agentAmove.y != other.agentAmove.y)
                    return false;
            }
            else
            { // Compare dests and directions (unless direction is NO_DIRECTION)
                if (this.agentAmove.Equals(other.agentAmove) == false)
                    return false;
                if (this.agentBmove.Equals(other.agentBmove) == false)
                    return false;
            }               
            return true;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return 2 * this.agentAIndex +
                       3 * this.agentBIndex +
                       5 * this.timeStep +
                       7 * this.isVertexConflict.GetHashCode() +
                       11 * this.agentAmove.GetHashCode() +
                       13 * this.agentBmove.GetHashCode();
            }
        }
    }
}
