
namespace CPF_experiment
{
    public class CbsConflict
    {
        public int agentA;
        public int agentB;
        public Move agentAmove;
        public Move agentBmove;
        public int timeStep;
        /// <summary>
        /// If true, conflict is two agents have same dest, from any direction. Otherwise it's an edge conflict.
        /// </summary>
        public bool vertex;

        public CbsConflict(int conflictingAgentA, int conflictingAgentB, Move agentAMove, Move agentBMove, int timeStep)
        {
            this.agentA = conflictingAgentA;
            this.agentB = conflictingAgentB;
            this.agentAmove = agentAMove;
            this.agentBmove = agentBMove;
            this.timeStep = timeStep;
            this.vertex = false;
            if (agentAMove.x == agentBMove.x && agentAMove.y == agentBMove.y) // Same dest, from any direction
                this.vertex = true;
        }

        public override string ToString()
        {
            return "Agent " + this.agentA + " going " + this.agentAmove + " collides with agent " + this.agentB + " going " + this.agentBmove + " at time " + this.timeStep;
        }

        public override bool Equals(object obj) // TODO: Implement GetHashCode()
        {
            if (this.agentA != ((CbsConflict)obj).agentA)
                return false;
            if (this.agentB != ((CbsConflict)obj).agentB)
                return false;
            if (this.vertex != ((CbsConflict)obj).vertex)
                return false;
            if (this.timeStep != ((CbsConflict)obj).timeStep)
                return false;
            if (this.vertex)
            { // Compare dests, ignore directions
                if (this.agentAmove.x != ((CbsConflict)obj).agentAmove.x)
                    return false;
                if (this.agentAmove.y != ((CbsConflict)obj).agentAmove.y)
                    return false;
            }
            else
            { // Compare dests and directions (unless direction is NO_DIRECTION)
                if (this.agentAmove.Equals(((CbsConflict)obj).agentAmove) == false)
                    return false;
                if (this.agentBmove.Equals(((CbsConflict)obj).agentBmove) == false)
                    return false;
            }               
            return true;
        }
    }
}
