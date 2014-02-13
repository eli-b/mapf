using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experiment
{
    public class DnCConstraint : IComparable
    {
        protected byte[] agent;
        protected ushort posX;
        protected ushort posY;
        protected sbyte direction;
        protected ushort timeStep;
        //public byte group;
       // bool onVartex;
        public static bool fullyEqual;

        public DnCConstraint(int agent, int posX, int posY, int timeStep, int direction)
        {
            this.agent = new byte[1];
            this.agent[0] = (byte)agent;
           this.posX=(ushort)posX;
           this.posY = (ushort)posY;
           this.timeStep = (ushort)timeStep;
           this.direction = (sbyte)direction;
          // this.onVartex = onVartex;
        }

        public DnCConstraint(){}

        public DnCConstraint(DnCConflict conflict, ProblemInstance instance, bool agentA)
        {
            if (agentA)
            {
                this.agent = new byte[1];
                this.agent[0] = (byte)instance.m_vAgents[conflict.agentA].agent.agentNum;
                this.posX = (ushort)conflict.agentAmove.x;
                this.posY = (ushort)conflict.agentAmove.y;
                this.direction = (sbyte)conflict.agentAmove.direction;
            }
            else
            {
                this.agent = new byte[1];
                this.agent[0] = (byte)instance.m_vAgents[conflict.agentB].agent.agentNum;
                this.posX = (ushort)conflict.agentBmove.x;
                this.posY = (ushort)conflict.agentBmove.y;
                this.direction = (sbyte)conflict.agentBmove.direction;
            }
            this.timeStep = (ushort)conflict.timeStep;
          //  this.onVartex = conflict.vartex;
            if (conflict.vartex)
                this.direction = -1;
        }

        public void init(int agent, int posX, int posY, int timeStep, int direction)
        {
            this.agent = new byte[1];
            this.agent[0] = (byte)agent;
            this.posX = (ushort)posX;
            this.posY = (ushort)posY;
            this.timeStep = (ushort)timeStep;
           this.direction = (sbyte)direction;
        }

        public override bool Equals(object obj)
        {
            if (fullyEqual && sameAgents(obj) == false)
                return false;
            foreach (byte agent in this.agent)
            {
                if (((DnCConstraint)obj).agent.Contains(agent) == false)
                    return false;
            }
            //if (this.agent != ((DnCConstraint)obj).agent)
            //    return false;
            if(this.posX != ((DnCConstraint)obj).posX)
                return false;
            if(this.posY != ((DnCConstraint)obj).posY)
                return false;
            if(this.timeStep != ((DnCConstraint)obj).timeStep)
                return false;
            if (this.direction != -1 && ((DnCConstraint)obj).direction != -1 && this.direction != ((DnCConstraint)obj).direction)
                return false;
            //if(obj.GetType().Equals(this.GetType()))
            //    if (this.group != ((DnCConstraint)obj).group)
            //         return false;
            return true;
        }

        public void addAgents(List<byte> addAgents)
        {
            if (addAgents.Count == 0)
                return;
            byte[] newAgents = new byte[agent.Length + addAgents.Count];
            int i;
            for (i = 0; i < agent.Length; i++)
			{
			    newAgents[i]=agent[i];
			}
            foreach (byte j in addAgents)
            {
                newAgents[i]=j;
                i++;
            }
            agent=newAgents;
        }

        public bool sameAgents(object obj)
        {
            if (this.agent.Length != ((DnCConstraint)obj).agent.Length)
                return false;
            //foreach (byte agent in this.agent)
            //{
            //    if (((DnCConstraint)obj).agent.Contains(agent) == false)
            //        return false;
            //}
            return true;
        }

        public override int  GetHashCode()
        {
 	        int ans = 0;
            ans+= posX;
            ans+= posY*1000;
            ans+= timeStep*887;
            //ans+= agent*79;
           // ans += direction * 9;
            return ans;
        }

        public int getX() { return this.posX; }
        public int getY() { return this.posY; }
        //public int getAgentNum() { return this.agent; }
        public int getTimeStep() { return this.timeStep; }
        public int getDirection()
        {
            return this.direction;
        }
        public override string ToString()
        {
            return "("+posX+","+posY+") direction-{"+direction+"} time-{"+timeStep+"}";
        }

        public bool allowes(DnCConstraint other)
        {
            if (this.posX != other.posX)
                return true;
            if (this.posY != other.posY)
                return true;
            if (this.timeStep != other.timeStep)
                return true;
            if (this.direction != other.direction)
                return true;
            foreach (byte agent in other.agent)
            {
                if (this.agent.Contains(agent))
                    return false;
            }
            return true;
        }

        public int CompareTo(object item)
        {
            DnCConstraint other = (DnCConstraint)item;

            if (this.timeStep > other.timeStep)
                return 1;
            return -1;
        }

        public bool vioalates(int agentO, int posXO, int posYO, int timeStepO, int directionO)
        {
            bool hasAgent = false;
            foreach (byte a in agent)
            {
                if (agentO == a)
                    hasAgent = true;
            }
            if (!hasAgent)
                return false;
            if (posXO != posX)
                return true;
            if (posYO != posY)
                return true;
            if (direction != -1 && directionO != direction)
                return true;
            return false;
        }
    }
}
