using System;

namespace mapf;

[Serializable] public class Agent
{
    public int agentNum;
    public Move Goal; // TODO: Make it a coordinate

    public Agent(Agent a)
    {
        agentNum = a.agentNum;
        Goal = new Move(a.Goal);
    }

    public Agent(int Goal_X, int Goal_Y, int agentNum)
    {
        this.agentNum = agentNum;
        Goal = new Move(Goal_X, Goal_Y, Move.Direction.NO_DIRECTION);
    }
    public override string ToString()
    {
        return $"Agent-{agentNum} Goal-{Goal}";
    }

    public override bool Equals(object other_obj)
    {
        if (other_obj == null)
            return false;
        Agent other = (Agent)other_obj;
        return agentNum == other.agentNum && Goal.Equals(other.Goal);
    }

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 23 + agentNum;
            hash = hash * 23 + Goal.GetHashCode();
            return hash;
        }
    }
}
