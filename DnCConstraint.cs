using System;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    public class DnCConstraint : IComparable
    {
        protected byte[] agents;
        protected ushort posX;
        protected ushort posY;
        protected sbyte direction;
        protected ushort timeStep;
      //public byte group;
      //bool onVartex;
        public static bool fullyEqual;

        public DnCConstraint(int agent, int posX, int posY, int timeStep, int direction)
        {
            this.agents = new byte[1] { (byte)agent };
            this.posX = (ushort)posX;
            this.posY = (ushort)posY;
            this.timeStep = (ushort)timeStep;
            this.direction = (sbyte)direction;
          //this.onVartex = onVartex;
        }

        public DnCConstraint() {}

        public DnCConstraint(DnCConflict conflict, ProblemInstance instance, bool agentA)
        {
            Move move;
            int agentNum;
            if (agentA)
            {
                move = conflict.agentAmove;
                agentNum = instance.m_vAgents[conflict.agentA].agent.agentNum;
            }
            else
            {
                move = conflict.agentBmove;
                agentNum = instance.m_vAgents[conflict.agentB].agent.agentNum;
            }
            this.posX = (ushort)move.x;
            this.posY = (ushort)move.y;
            this.direction = (sbyte)move.direction;
            this.agents = new byte[1] { (byte)agentNum };
            
            this.timeStep = (ushort)conflict.timeStep;
          //this.onVartex = conflict.vartex;
            if (conflict.vartex)
                this.direction = (int)Move.Direction.NO_DIRECTION;
        }

        public void init(int agent, int posX, int posY, int timeStep, int direction)
        {
            this.agents = new byte[1] { (byte)agent };
            this.posX = (ushort)posX;
            this.posY = (ushort)posY;
            this.timeStep = (ushort)timeStep;
            this.direction = (sbyte)direction;
        }

        public override bool Equals(object obj)
        {
            if (fullyEqual && (sameAgents(obj) == false))
                return false;
            foreach (byte agent in this.agents)
            {
                if (((DnCConstraint)obj).agents.Contains(agent) == false)
                    return false;
            }
            //if (this.agents != ((DnCConstraint)obj).agents)
            //    return false;
            if (this.posX != ((DnCConstraint)obj).posX)
                return false;
            if (this.posY != ((DnCConstraint)obj).posY)
                return false;
            if (this.timeStep != ((DnCConstraint)obj).timeStep)
                return false;
            if (this.direction != (int)Move.Direction.NO_DIRECTION && 
                ((DnCConstraint)obj).direction != (int)Move.Direction.NO_DIRECTION && 
                this.direction != ((DnCConstraint)obj).direction)
                return false;
            //if (obj.GetType().Equals(this.GetType()))
            //    if (this.group != ((DnCConstraint)obj).group)
            //         return false;
            return true;
        }

        public void addAgents(List<byte> addAgents)
        {
            if (addAgents.Count == 0)
                return;
            byte[] newAgents = new byte[agents.Length + addAgents.Count];
            Array.Copy(agents, newAgents, agents.Length);
            Array.Copy(addAgents.ToArray<byte>(), 0, newAgents, agents.Length, addAgents.Count);
            agents = newAgents;
        }

        public bool sameAgents(object obj)
        {
            if (this.agents.Length != ((DnCConstraint)obj).agents.Length)
                return false;
            //foreach (byte agents in this.agents)
            //{
            //    if (((DnCConstraint)obj).agents.Contains(agents) == false)
            //        return false;
            //}
            return true;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int ans = 0;
                ans += posX;
                ans += posY * 1000;
                ans += timeStep * 887;
              //ans += agents * 79;
              //ans += direction * 9;
                return ans;
            }
        }

        public int getX() { return this.posX; }
        public int getY() { return this.posY; }
        //public int getAgentNum() { return this.agents; }
        public int getTimeStep() { return this.timeStep; }

        public int getDirection()
        {
            return this.direction;
        }
        
        public override string ToString()
        {
            return "(" + posX + "," + posY + ") direction-{" + direction + "} time-{" + timeStep + "}";
        }

        /// <summary>
        /// What's the difference between Equals and this method?
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool allows(DnCConstraint other)
        {
            if (this.posX != other.posX)
                return true;
            if (this.posY != other.posY)
                return true;
            if (this.timeStep != other.timeStep)
                return true;
            if (this.direction != other.direction)
                return true;
            foreach (byte agent in other.agents)
            {
                if (this.agents.Contains(agent))
                    return false;
            }
            return true;
        }

        public int CompareTo(object item)
        {
            DnCConstraint other = (DnCConstraint)item;

            return this.timeStep.CompareTo(other.timeStep);
        }

        /// <summary>
        /// Why is there one allows method and one violates method? Choose an interface!
        /// </summary>
        /// <param name="agentO"></param>
        /// <param name="posXO"></param>
        /// <param name="posYO"></param>
        /// <param name="timeStepO">This param is ignored!</param>
        /// <param name="directionO"></param>
        /// <returns></returns>
        public bool violates(int agentO, int posXO, int posYO, int timeStepO, int directionO)
        {
            if (agents.Contains<byte>((byte)agentO) == false)
                return false;
            if (posXO != posX)
                return true; // Not false??
            if (posYO != posY)
                return true; // Not false??
            if (direction != (int)Move.Direction.NO_DIRECTION && directionO != (int)Move.Direction.NO_DIRECTION &&
                directionO != direction)
                return true; // Not false??
            return false;
        }
    }
}
