
namespace CPF_experiment
{
    /// <summary>
    /// This class represents a conflict between two groups of agents in Standley's Indepedence Detection algorithm and in ICTS.
    /// </summary>
    class Conflict
    {
        public AgentsGroup group1;
        public AgentsGroup group2;
        public int timeOfConflict;

        public override string ToString()
        {
            return "conflict in time " + timeOfConflict + " between " + group1 + " and " + group2;
        }

        public Conflict(AgentsGroup group1, AgentsGroup group2, int time)
        {
            this.group1 = group1;
            this.group2 = group2;
            timeOfConflict = time;
        }
        public override bool Equals(object obj)
        {
            Conflict other = (Conflict)obj;
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
