using System;

namespace CPF_experiment
{
    [Serializable] public class Agent
    {
        public int agentNum;
        public int Goal_X;
        public int Goal_Y;

        public Agent(Agent a)
        {
            agentNum = a.agentNum;
            Goal_X = a.Goal_X;
            Goal_Y = a.Goal_Y;
        }

        public Agent(int Goal_X, int Goal_Y, int agentNum)
        {
            this.agentNum = agentNum;
            this.Goal_X = Goal_X;
            this.Goal_Y = Goal_Y;
        }
        public override string ToString()
        {
            return "Agent-" + agentNum + " Goal-(" + Goal_X + "," + Goal_Y + ")";
        }

        public override bool Equals(object other_obj)
        {
            Agent other = (Agent)other_obj;
            return agentNum == other.agentNum && Goal_X == other.Goal_X && Goal_Y == other.Goal_Y;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + agentNum;
                hash = hash * 23 + Goal_X;
                hash = hash * 23 + Goal_Y;
                return hash;
            }
        }
    }
}
