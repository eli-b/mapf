using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experement
{
    /// <summary>
    /// Describe a move at a given timestep.
    /// </summary>
    public class TimedMove  : Move
    {
        public int time;

        public TimedMove(int x, int y, int direction, int time) : base(x,y,direction)
        {
            this.time = time;
        }
        public TimedMove(Move cpy, int time)
            : base(cpy)
        {
            this.time = time;
        }

        public TimedMove() { }

        public TimedMove(TimedMove cpy) : base(cpy)
        {
            this.time = cpy.time;
        }

        public override bool Equals(object obj)
        {
            if (this.time != ((TimedMove)obj).time)
                return false;

            /// <remarks>
            /// Calling the base class might cause a problem in the efficiency (adds another function call).
            /// Consider just copying the code.
            /// </remarks>
            return base.Equals(obj);
        }

        public TimedMove GetMoveWithoutDirection()
        {
            TimedMove copy = new TimedMove(this);
            copy.direction = NO_DIRECTION;
            return copy;
        }

        /// <summary>
        /// Check if the given move collides with this move.
        /// This includes:
        /// 1. Head on collision
        /// 2. When oth moves target the same location.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool isColliding(TimedMove other)
        {
            if (this.time != other.time)
                return false;

            /// <remarks>
            /// Calling the base class might cause a problem in the efficiency (adds another function call).
            /// Consider just copying the code.
            /// </remarks>
            return base.Equals(other);
        }

        /// <summary>
        /// TODO: Replace direction codes with an enum or at least a constant.
        /// </summary>
        /// <returns></returns>
        public TimedMove GetOppositeMove()
        {
            Move oppositeMove = base.GetOppositeMove();

            /// <remarks>
            /// Calling the base class might cause a problem in the efficiency (adds another function call).
            /// Consider just copying the code.
            /// </remarks>
            return new TimedMove(oppositeMove.x, oppositeMove.y, oppositeMove.direction, this.time);
        }

        public void setup(Move cpy)
        {
            base.setup(cpy);
        }

        public void setup(Move cpy, int time)
        {
            base.setup(cpy);
            this.time = time;
        }

        public void setup(int x, int y, int direction,int time)
        {
            base.setup(x, y, direction);
            this.time = time;
        }

        
        public bool containd(HashSet<TimedMove> CAT)
        {
            if (CAT == null)
                return false;
            int saveDirection = this.direction;
            this.direction = -1;

            if (CAT.Contains(this))
                return true;

            this.direction = direction;
            this.setOppositeMove();

            if (CAT.Contains(this))
            {
                this.setOppositeMove();
                return true;
            }
            this.setOppositeMove();
            return false;
        }

    public bool containd(HashSet_U<TimedMove> CAT)
        {
            if (CAT == null)
                return false;
            int saveDirection = this.direction;
            this.direction = -1;

            if (CAT.Contains(this))
                return true;

            this.direction = direction;
            this.setOppositeMove();

            if (CAT.Contains(this))
            {
                this.setOppositeMove();
                return true;
            }
            this.setOppositeMove();
            return false;
        }
    }


    class CoordinateForConflictRatio : TimedMove
    {
        ProblemInstance ins;
        public int cardinality;

        public CoordinateForConflictRatio(ProblemInstance ins, AgentState state)
            : base(state.pos_X,state.pos_Y,state.direction,state.currentStep)
        {
            this.ins = ins;
            this.cardinality = ins.getCardinality(state);
        }



        public CoordinateForConflictRatio(CoordinateForConflictRatio cpy)
            : base(cpy)
        {
            this.ins = cpy.ins;
            this.cardinality = cpy.cardinality;
        }

        public override bool Equals(object obj)
        {
            return base.Equals(obj);
        }

        public override int GetHashCode()
        {
            return base.time * (ins.maxCardinality + 26) + base.direction * 5 + cardinality;
        }
    }
}
