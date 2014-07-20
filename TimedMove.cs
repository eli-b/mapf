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
        /// <returns></returns>
        public new IEnumerable<TimedMove> GetNextMoves()
        {
            Direction[] directions;
            if (Constants.ALLOW_DIAGONAL_MOVE)
                directions = Move.validDirections;
            else
                directions = Move.validDirectionsNoDiag;
            foreach (Direction op in directions)
            {
                yield return new TimedMove(this.x + Move.directionToDeltas[(int)op, 0],
                                           this.y + Move.directionToDeltas[(int)op, 1], op, this.time + 1);
            }
        }

        /// <summary>
        /// Change coordinates in specified direction and increment timestep.
        /// </summary>
        /// <param name="direction"></param>
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
        public bool IsColliding(TimedMove other)
        {
            return IsColliding(other.x, other.y, other.direction, other.time);
        }

        public bool IsColliding(int other_x, int other_y, Direction other_direction, int time)
        {
            if (this.time != time)
                return false;

            return base.IsColliding(other_x, other_y, other_direction);
        }

        /// <summary>
        /// Reimplemented to avoid creating temporary Move objects
        /// </summary>
        /// <returns></returns>
        public new TimedMove GetOppositeMove()
        {
            if (direction == Direction.Wait || direction == Direction.NO_DIRECTION)
                return this;
            return new TimedMove(this.x + Move.directionToOppositeDeltas[(int)direction, 0],
                            this.y + directionToOppositeDeltas[(int)direction, 1],
                            directionToOppositeDirection[(int)direction], this.time);
        }

        /// <summary>
        /// Isn't used anywhere
        /// </summary>
        /// <param name="cpy"></param>
        /// <param name="time"></param>
        public void setup(Move cpy, int time)
        {
            base.setup(cpy);
            this.time = time;
        }

        /// <summary>
        /// Not used anywhere
        /// </summary>
        /// <param name="cpy"></param>
        public void setup(TimedMove cpy)
        {
            base.setup(cpy);
            this.time = cpy.time;
        }

        /// <summary>
        /// Almost isn't used anywhere
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="direction"></param>
        /// <param name="time"></param>
        public void setup(int x, int y, Move.Direction direction, int time)
        {
            base.setup(x, y, direction);
            this.time = time;
        }

        public bool IsColliding(ICollection<TimedMove> CAT)
        {
            // Sadly, since there's currently no System.Collections.Generic.IReadOnlySet, Move.IsColliding accepts an ISet<Move>,
            // so the compiler doesn't let us just call base.IsColliding(CAT) because base might put a Move that isn't a TimedMove in CAT,
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
}
