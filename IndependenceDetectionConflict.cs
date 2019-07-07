
namespace mapf
{
    /// <summary>
    /// This class represents a conflict between two groups of agents in Trevor Standley's
    /// Indepedence Detection algorithm.
    /// </summary>
    class IndependenceDetectionConflict
    {
        public IndependenceDetectionAgentsGroup group1;
        public IndependenceDetectionAgentsGroup group2;
        public int timeOfConflict;

        public override string ToString()
        {
            return "conflict in time " + timeOfConflict + " between " + group1 + " and " + group2;
        }

        public IndependenceDetectionConflict(IndependenceDetectionAgentsGroup group1, IndependenceDetectionAgentsGroup group2, int time)
        {
            this.group1 = group1;
            this.group2 = group2;
            timeOfConflict = time;
        }
        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            IndependenceDetectionConflict other = (IndependenceDetectionConflict)obj;
            if (this.group1.Equals(other.group1) && this.group2.Equals(other.group2))
                return true; // Ignoring timeOfConflict, not ignoring order of groups.
            return false;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return this.group1.GetHashCode() + 3 * this.group2.GetHashCode();
            }
        }
    }
}
