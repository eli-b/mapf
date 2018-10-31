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

//        public static const int FOREVER_AFTER = int.MaxValue

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
            if (obj == null)
                return false;
            if (this.time != ((TimedMove)obj).time)
                return false;

            //return base.Equals(obj);

            // Begin copied code of base to avoid a method call
            Move that = (Move)obj;
            return (this.x == that.x && this.y == that.y &&
                    ((this.direction == Direction.NO_DIRECTION) || (that.direction == Direction.NO_DIRECTION) ||
                     (this.direction == that.direction)));
            // End copied code of base
        }

        public override int GetHashCode()
        {
            unchecked
            {
                //return base.GetHashCode() * 3 + this.time;

                // Begin copied code of base to avoid a method call:
                int hash = 17;
                hash = 23 * hash + x;
                hash = 23 * hash + y;
                // End copied code of base
                return hash * 3 + this.time;
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
        /// 
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

        public bool IsColliding(ICollection<TimedMove> moves)
        {
            Move.Direction saveDirection = this.direction;
            this.direction = Move.Direction.NO_DIRECTION;
            if (moves.Contains(this))
            {
                this.direction = saveDirection;
                return true;
            }
            this.direction = saveDirection;

            if (Constants.ALLOW_HEAD_ON_COLLISION == false)
            {
                this.setOppositeMove();
                if (moves.Contains(this)) // Check direction too now
                {
                    this.setOppositeMove();
                    return true;
                }
                this.setOppositeMove();
            }

            return false;
        }

        public bool IsColliding(IReadOnlyDictionary<TimedMove, int> timedMovesToAgentID)
        {
            Move.Direction saveDirection = this.direction;
            this.direction = Move.Direction.NO_DIRECTION;
            if (timedMovesToAgentID.ContainsKey(this))
            {
                this.direction = saveDirection;
                return true;
            }
            this.direction = saveDirection;

            if (Constants.ALLOW_HEAD_ON_COLLISION == false)
            {
                this.setOppositeMove();
                if (timedMovesToAgentID.ContainsKey(this)) // Check direction too now
                {
                    this.setOppositeMove();
                    return true;
                }
                this.setOppositeMove();
            }

            return false;
        }

        /// <summary>
        /// Gets a dictionary mapping TimedMoves to the agent that already made them
        /// and returns a list of agents this TimedMove collides with.
        /// </summary>
        /// <param name="timedMovesToAgentIndex"></param>
        /// <returns></returns>
        public List<int> GetColliding(IReadOnlyDictionary<TimedMove, int> timedMovesToAgentIndex)
        {
            List<int> ans = null;
            Move.Direction saveDirection = this.direction;
            Direction[] directions;
            if (Constants.ALLOW_DIAGONAL_MOVE)
                directions = Move.validDirections;
            else
                directions = Move.validDirectionsNoDiag;
            foreach (var direction in directions) // TEMP FIX! Need to get rid of the whole NO_DIRECTION SHTICK! It breaks transitivity!
            {
                this.direction = direction;
                if (timedMovesToAgentIndex.ContainsKey(this))
                {
                    if (ans == null)
                        ans = new List<int>(4);
                    ans.Add(timedMovesToAgentIndex[this]);
                }
            }
            this.direction = saveDirection;

            if (Constants.ALLOW_HEAD_ON_COLLISION == false)
            {
                this.setOppositeMove();
                if (timedMovesToAgentIndex.ContainsKey(this)) // Check direction too now
                {
                    if (ans == null)
                        ans = new List<int>(1);
                    ans.Add(timedMovesToAgentIndex[this]);
                }
                this.setOppositeMove();
            }

            if (ans != null)
                return ans;
            else
                return TimedMove.emptyList;
        }

        private static readonly List<int> emptyList = new List<int>(0);

        /// <summary>
        /// Gets a dictionary mapping TimedMoves to the agents that already made them
        /// and returns a list of agents this TimedMove collides with.
        /// </summary>
        /// <param name="timedMovesToAgentNumLists"></param>
        /// <returns></returns>
        public List<int> GetColliding(IReadOnlyDictionary<TimedMove, List<int>> timedMovesToAgentNumLists)
        {
            List<int> ans = null;
            Move.Direction saveDirection = this.direction;
            Direction[] directions;
            if (Constants.ALLOW_DIAGONAL_MOVE)
                directions = Move.validDirections;
            else
                directions = Move.validDirectionsNoDiag;
            foreach (var direction in directions) // TEMP FIX! Need to get rid of the whole NO_DIRECTION SHTICK! It breaks transitivity!
            {
                this.direction = direction;
                if (timedMovesToAgentNumLists.ContainsKey(this))
                {
                    if (ans == null)
                        ans = new List<int>(timedMovesToAgentNumLists[this].Count + 3);  // Why +3? Just preemptively taking more space?
                    ans.AddRange(timedMovesToAgentNumLists[this]);
                }
            }
            this.direction = saveDirection;

            if (Constants.ALLOW_HEAD_ON_COLLISION == false)
            {
                this.setOppositeMove();
                if (timedMovesToAgentNumLists.ContainsKey(this)) // Check direction too now
                {
                    if (ans == null)
                        ans = new List<int>(timedMovesToAgentNumLists[this].Count);
                    ans.AddRange(timedMovesToAgentNumLists[this]);
                }
                this.setOppositeMove();
            }

            if (ans != null)
                return ans;
            else
                return TimedMove.emptyList;
        }

        public void UpdateConflictCounts(IReadOnlyDictionary<TimedMove, List<int>> conflictAvoidance,
                                         Dictionary<int, int> conflictCounts, Dictionary<int, List<int>> conflictTimes)
        {
            List<int> colliding = this.GetColliding(conflictAvoidance);
            foreach (int agentNum in colliding)
            {
                if (conflictCounts.ContainsKey(agentNum) == false)
                    conflictCounts[agentNum] = 0;
                conflictCounts[agentNum] += 1;
                if (conflictTimes.ContainsKey(agentNum) == false)
                    conflictTimes[agentNum] = new List<int>(4);
                conflictTimes[agentNum].Add(this.time);
            }
        }
    }
}
