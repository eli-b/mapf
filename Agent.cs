using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experement
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
            return "Agent-"+agentNum+" Goal-("+Goal_X+","+Goal_Y+")";
        }
    }
}
