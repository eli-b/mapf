using System.Collections.Generic;

namespace CPF_experiment
{
    /// <summary>
    /// Describes a Move at a given timestep.
    /// </summary>
    public class TimedMove  : Move
    {
        public int time;

        public TimedMove(int x, int y, int direction, int time)
            : base(x, y, direction)
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

            return base.Equals(obj);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return base.GetHashCode() * 3 + this.time;
            }
        }

        public new TimedMove GetMoveWithoutDirection()
        {
            TimedMove copy = new TimedMove(this);
            copy.direction = (int)Direction.NO_DIRECTION;
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

            return base.Equals(other);
        }

        public new TimedMove GetOppositeMove()
        {
            Move oppositeMove = base.GetOppositeMove();

            return new TimedMove(oppositeMove.x, oppositeMove.y, oppositeMove.direction, this.time);
        }

        public void setup(Move cpy, int time)
        {
            base.setup(cpy);
            this.time = time;
        }

        public void setup(int x, int y, int direction, int time)
        {
            base.setup(x, y, direction);
            this.time = time;
        }

        public bool isColliding(HashSet<TimedMove> CAT)
        {
            // Sadly, since there's currently no System.Collections.Generic.IReadOnlySet, Move.isColliding accepts an ISet<Move>,
            // so the compiler doesn't let us just call base.isColliding(CAT) because base might put a Move that isn't a TimedMove in CAT,
            if (CAT == null)
                return false;

            int saveDirection = this.direction;
            this.direction = (int)Move.Direction.NO_DIRECTION;
            if (CAT.Contains(this))
            {
                this.direction = saveDirection;
                return true;
            }
            this.direction = saveDirection;

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

        /// <summary>
        /// Probably the intended implementation
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            return base.Equals(obj);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = 23 * hash + base.GetHashCode();
                // Until Equals checks these fields, we can't use them in the hash :(
                //hash = 23 * hash + ins.GetHashCode();
                //hash = 23 * hash + cardinality;
                return hash;
            }
        }
    }
}
