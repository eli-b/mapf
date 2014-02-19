
namespace CPF_experiment
{
    public class DnCConflict
    {
        public int agentA;
        public int agentB;
        public Move agentAmove;
        public Move agentBmove;
        public int timeStep;
        public bool vartex; // Same dest, from any direction

        public DnCConflict(int conflictingAgentA, int conflictingAgentB, Move agentA, Move agentB, int timeStep)
        {
            this.agentA = conflictingAgentA;
            this.agentB = conflictingAgentB;
            this.agentAmove = agentA;
            this.agentBmove = agentB;
            this.timeStep = timeStep;
            this.vartex = false;
            if (agentA.x == agentB.x && agentA.y == agentB.y) // Same dest, from any direction
                this.vartex = true;
        }

        public override bool Equals(object obj)
        {
            if (this.agentA != ((DnCConflict)obj).agentA)
                return false;
            if (this.agentB != ((DnCConflict)obj).agentB)
                return false;
            if (this.vartex != ((DnCConflict)obj).vartex)
                return false;
            if (this.timeStep != ((DnCConflict)obj).timeStep)
                return false;
            if (this.vartex)
            { // Compare dests, ignore directions
                if (this.agentAmove.x != ((DnCConflict)obj).agentAmove.x)
                    return false;
                if (this.agentAmove.y != ((DnCConflict)obj).agentAmove.y)
                    return false;
            }
            else
            { // Compare dests and directions (unless direction is NO_DIRECTION)
                if (this.agentAmove.Equals(((DnCConflict)obj).agentAmove) == false)
                    return false;
                if (this.agentBmove.Equals(((DnCConflict)obj).agentBmove) == false)
                    return false;
            }               
            return true;
        }
    }
}
