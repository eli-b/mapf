using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace mapf
{
    public class CbsConstraint : IComparable
    {
        public byte agentNum {get; protected set;}
        public TimedMove move {get; protected set;}
        public bool queryInstance = false;

        public CbsConstraint(int agentNum, int posX, int posY, Move.Direction direction, int timeStep)
        {
            this.Init(agentNum, posX, posY, direction, timeStep);
        }

        public CbsConstraint(int agentNum, TimedMove move)
        {
            this.Init(agentNum, move);
        }

        public CbsConstraint() : this(-1, -1, -1, Move.Direction.NO_DIRECTION, -1) {} // Nonsense values until Init, just allocate move

        public CbsConstraint(CbsConflict conflict, ProblemInstance instance, bool agentA)
        {
            Move move;
            int agentNum;

            if (agentA)
            {
                move = conflict.agentAmove;
                agentNum = instance.agents[conflict.agentAIndex].agent.agentNum;
            }
            else
            {
                move = conflict.agentBmove;
                agentNum = instance.agents[conflict.agentBIndex].agent.agentNum;
            }

            this.agentNum = (byte)agentNum;
            this.move = new TimedMove(move, conflict.timeStep);

            if (conflict.isVertexConflict)
                this.move.direction = Move.Direction.NO_DIRECTION;
        }

        public void Init(int agentNum, int posX, int posY, Move.Direction direction, int timeStep)
        {
            this.Init(agentNum, new TimedMove(posX, posY, direction, timeStep));
        }

        public void Init(int agentNum, TimedMove move)
        {
            this.agentNum = (byte)agentNum;
            this.move = move;
        }

        public int time
        {
            get
            {
                return this.move.time;
            }
        }

        /// <summary>
        /// Checks that the agentNum is equal, and compares the move.
        /// If one of the constraints is a query, an instance only created and used to quickly search for a move in a set of constraints,
        /// the direction is ignored if the other constraint is a vertex constraint.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            CbsConstraint other = (CbsConstraint)obj;
            if (this.agentNum != other.agentNum)
                return false;

            Debug.Assert(this.queryInstance == false || other.queryInstance == false); // At most one of the instances is a query
            Debug.Assert(this.queryInstance == false || this.move.direction != Move.Direction.NO_DIRECTION); // Must query regarding a specific direction
            Debug.Assert(other.queryInstance == false || other.move.direction != Move.Direction.NO_DIRECTION); // Must query regarding a specific direction
            if (this.queryInstance || other.queryInstance) // This way if the constraint is a vertex constraint than it will be equal to a query containing a move from any direction to that position,
                                                           // and if it is an edge constraint than it will only be equal to queries containing a move from that specific direction to that position.
                return this.move.Equals(other.move);
            else // A vertex constraint is different to an edge constraint for the same agentNum and position.
                 // Must check the direction explicitly because vertex constraints have no direction and moves with no direction
                 // compare equal to moves with any direction
                 // TODO: Get rid of all of this using Nathan's advice.
                return this.move.Equals(other.move) && this.move.direction == other.move.direction; 
        }

        /// <summary>
        /// Uses the move and the agents.
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int ans = 0;
                ans += this.move.GetHashCode() * 3;
                ans += this.agentNum * 5;
                return ans;
            }
        }

        public int GetTimeStep() { return this.move.time; } // FIXME: Make this into a property

        public Move.Direction GetDirection()
        {
            return this.move.direction;
        }
        
        public override string ToString()
        {
            return $"{move}-{move.direction,-12} time={move.time} agentNum {agentNum}";
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
            if (this.agentNum == other.agentNum)
                return false;
            return true;
        }

        public int CompareTo(object item)
        {
            CbsConstraint other = (CbsConstraint)item;

            return this.move.time.CompareTo(other.move.time);
        }

        public bool ViolatesMustConstraint(byte agent, TimedMove move)
        {
            if (this.agentNum != agent)
                return false;
            return this.move.Equals(move) == false;
        }
    }
}
