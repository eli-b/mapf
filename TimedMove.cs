using System.Collections.Generic;
using System.Linq;

namespace CPF_experiment
{
    /// <summary>
    /// Describes a Move at a given timestep.
    /// </summary>
    public class TimedMove  : Move
    {
        public int time;

        public TimedMove(int x, int y, Move.Direction direction, int time)
            : base(x, y, direction)
        {
            this.time = time;
        }

        /// <summary>
        /// A generator yielding new adjacent TimedMoves. Reimplemented to avoid creating temporary Moves.
        /// </summary>
        /// <param name="allowDiag"></param>
        /// <returns></returns>
        public IEnumerable<TimedMove> GetNextMoves(bool allowDiag = false)
        {
            int count;
            if (allowDiag)
                count = Move.NUM_DIRECTIONS;
            else
                count = Move.NUM_NON_DIAG_MOVES;
            foreach (Direction op in System.Enum.GetValues(typeof(Move.Direction)).OfType<Direction>().Take<Direction>(count))
            {
                yield return new TimedMove(this.x + Move.directionToDeltas[(int)op, 0],
                                           this.y + Move.directionToDeltas[(int)op, 1], op, this.time + 1);
            }
        }

        public override void Update(Direction direction)
        {
            base.Update(direction);
            this.time += 1;
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
            copy.direction = Direction.NO_DIRECTION;
            return copy;
        }

        /// <summary>
        /// Check if the given move collides with this move.
        /// This includes:
        /// 0. Same time
        /// 1. Head on collision
        /// 2. When other moves target the same location.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool isColliding(TimedMove other)
        {
            return isColliding(other.x, other.y, other.direction, other.time);
        }

        public bool isColliding(int other_x, int other_y, Direction other_direction, int time)
        {
            if (this.time != time)
                return false;

            return base.isColliding(other_x, other_y, other_direction);
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

        public void setup(TimedMove cpy)
        {
            base.setup(cpy);
            this.time = cpy.time;
        }

        public void setup(int x, int y, Move.Direction direction, int time)
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

            Move.Direction saveDirection = this.direction;
            this.direction = Move.Direction.NO_DIRECTION;
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
            : base(state.last_move.x, state.last_move.y, state.last_move.direction, state.last_move.time)
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
