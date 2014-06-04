using System;
using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    public class CbsConstraint : IComparable
    {
        protected List<byte> agents;
        protected TimedMove move;
      //public byte group;
        public static bool fullyEqual;

        public CbsConstraint(int agent, int posX, int posY, Move.Direction direction, int timeStep)
        {
            this.agents = new List<byte>();
            this.agents.Add((byte)agent);
            this.move = new TimedMove(posX, posY, direction, timeStep);
        }

        public CbsConstraint(int agent, TimedMove move)
        {
            this.agents = new List<byte>();
            this.agents.Add((byte)agent);
            this.move = new TimedMove(move);
        }

        public CbsConstraint() : this(-1, -1, -1, Move.Direction.NO_DIRECTION, -1) {} // Nonsense values until Init, just allocate move

        public CbsConstraint(CbsConflict conflict, ProblemInstance instance, bool agentA)
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
            this.agents = new List<byte>();
            this.agents.Add((byte)agentNum);

            this.move = new TimedMove(move, conflict.timeStep);
            if (conflict.vertex)
                this.move.direction = Move.Direction.NO_DIRECTION;
        }

        public void Init(int agent, int posX, int posY, Move.Direction direction, int timeStep)
        {
            this.agents = new List<byte>();
            this.agents.Add((byte)agent);
            this.move.setup(posX, posY, direction, timeStep);
        }

        public void Init(int agent, TimedMove move)
        {
            this.Init(agent, move.x, move.y, move.direction, move.time);
        }

        public int time
        {
            get
            {
                return this.move.time;
            }
        }

        /// <summary>
        /// If fullyEqual, checks that the agent sets are equal, otherwise checks that this.agents is a subset of obj.agents.
        /// If not fullyEqual, this doesn't implement commutativity!
        /// Always compares the move.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            CbsConstraint other = (CbsConstraint)obj;
            if (fullyEqual)
            {
                if (this.agents.Count != other.agents.Count)
                    return false;
            }

            // This only checks that this.agents is a subset of other.agents!
            foreach (byte agent in this.agents)
            {
                if (other.agents.Contains(agent) == false)
                    return false;
            }
            //if (obj.GetType().Equals(this.GetType()))
            //    if (this.group != ((CbsConstraint)obj).group)
            //         return false;
            return this.move.Equals(other.move);
        }

        public void AddAgents(List<byte> addAgents)
        {
            this.agents.AddRange(addAgents);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int ans = 0;
                ans += move.GetHashCode() * 3;
                for (int i = 0; i < this.agents.Count; i++)
                {
                    ans += this.agents[i] * Constants.PRIMES_FOR_HASHING[(i + 2) % Constants.PRIMES_FOR_HASHING.Length];
                }
                return ans;
            }
        }

        public int GetX() { return this.move.x; } // Not used anywhere
        public int GetY() { return this.move.y; } // Not used anywhere
        public int GetTimeStep() { return this.move.time; } // FIXME: Make this and the above into properties
        public List<byte> GetAgents() { return this.agents; }

        public Move.Direction GetDirection()
        {
            return this.move.direction;
        }
        
        public override string ToString()
        {
            return move.ToString() + " direction-{" + move.direction + "} time-{" + move.time + "} first agent {" + agents[0] + "}";
        }

        /// <summary>
        /// Kind of the opposite of Equals: checks that the moves are unequal or that not one of the other's agents appears in this.agents.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool Allows(CbsConstraint other)
        {
            if (this.move.Equals(other.move) == false) // Minor behavior change: if exactly one move has a set direction, and they're otherwise equal the method used to return true.
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
            CbsConstraint other = (CbsConstraint)item;

            return this.move.time.CompareTo(other.move.time);
        }

        /// <summary>
        /// Not used anywhere.
        /// </summary>
        /// <param name="agent"></param>
        /// <param name="posX"></param>
        /// <param name="posY"></param>
        /// <param name="timeStep"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        public bool ViolatesMustConstraint(byte agent, int posX, int posY, Move.Direction direction, int timeStep)
        {
            return this.ViolatesMustConstraint(agent, new TimedMove(posX, posY, direction, timeStep));
        }

        public bool ViolatesMustConstraint(byte agent, TimedMove move)
        {
            if (agents.Contains<byte>(agent) == false) // Must-constraints can only logically have one agent -
                                                       // you can't force two agents to be at the same place and at the same time
                return false;
            return this.move.Equals(move) == false;
        }
    }
}
