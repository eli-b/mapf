using System.Collections.Generic;

namespace mapf;

/// <summary>
/// Describes a Move at a given timestep.
/// </summary>
public class TimedMove  : Move
{
    public int time;

//        public static const int FOREVER_AFTER = int.MaxValue

    public TimedMove(int x, int y, Direction direction, int time)
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
            yield return new TimedMove(this.X + Move.directionToDeltas[(int)op, 0],
                                        this.Y + Move.directionToDeltas[(int)op, 1], op, this.time + 1);
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
        return (this.X == that.X && this.Y == that.Y &&
                ((this.Direction == Direction.NO_DIRECTION) || (that.Direction == Direction.NO_DIRECTION) ||
                    (this.Direction == that.Direction)));
        // End copied code of base
    }

    public override int GetHashCode()
    {
        unchecked
        {
            //return base.GetHashCode() * 3 + this.time;

            // Begin copied code of base to avoid a method call:
            int hash = 17;
            hash = 23 * hash + X;
            hash = 23 * hash + Y;
            // End copied code of base
            return hash * 3 + this.time;
        }
    }

    public new TimedMove GetMoveWithoutDirection()
    {
        TimedMove copy = new TimedMove(this);
        copy.Direction = Direction.NO_DIRECTION;
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
        return IsColliding(other.X, other.Y, other.Direction, other.time);
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
        if (Direction == Direction.Wait || Direction == Direction.NO_DIRECTION)
            return this;
        return new TimedMove(this.X + Move.directionToOppositeDeltas[(int)Direction, 0],
                        this.Y + directionToOppositeDeltas[(int)Direction, 1],
                        directionToOppositeDirection[(int)Direction], this.time);
    }

    /// <summary>
    /// Isn't used anywhere
    /// </summary>
    /// <param name="cpy"></param>
    /// <param name="time"></param>
    public void setup(Move cpy, int time)
    {
        base.Setup(cpy);
        this.time = time;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="cpy"></param>
    public void setup(TimedMove cpy)
    {
        base.Setup(cpy);
        this.time = cpy.time;
    }

    /// <summary>
    /// Almost isn't used anywhere
    /// </summary>
    /// <param name="x"></param>
    /// <param name="y"></param>
    /// <param name="direction"></param>
    /// <param name="time"></param>
    public void setup(int x, int y, Direction direction, int time)
    {
        base.Setup(x, y, direction);
        this.time = time;
    }

    public bool IsColliding(ICollection<TimedMove> moves)
    {
        Direction saveDirection = this.Direction;
        this.Direction = Direction.NO_DIRECTION;
        if (moves.Contains(this))
        {
            this.Direction = saveDirection;
            return true;
        }
        this.Direction = saveDirection;

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
        if (timedMovesToAgentID == null)
            return false;
        Direction saveDirection = this.Direction;
        this.Direction = Direction.NO_DIRECTION;
        if (timedMovesToAgentID.ContainsKey(this))
        {
            this.Direction = saveDirection;
            return true;
        }
        this.Direction = saveDirection;

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
        Direction saveDirection = this.Direction;
        Direction[] directions;
        if (Constants.ALLOW_DIAGONAL_MOVE)
            directions = Move.validDirections;
        else
            directions = Move.validDirectionsNoDiag;
        foreach (var direction in directions) // TEMP FIX! Need to get rid of the whole NO_DIRECTION SHTICK! It breaks transitivity!
        {
            this.Direction = direction;
            if (timedMovesToAgentIndex.ContainsKey(this))
            {
                if (ans == null)
                    ans = new List<int>(4);
                ans.Add(timedMovesToAgentIndex[this]);
            }
        }
        this.Direction = saveDirection;

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
    /// <param name="CAT"></param>
    /// <returns></returns>
    public IReadOnlyList<int> GetColliding(ConflictAvoidanceTable CAT)
    {
        List<int> ans = null;
        Direction saveDirection = this.Direction;
        Direction[] directions;
        if (Constants.ALLOW_DIAGONAL_MOVE)
            directions = Move.validDirections;
        else
            directions = Move.validDirectionsNoDiag;
        foreach (var direction in directions) // TEMP FIX! Need to get rid of the whole NO_DIRECTION SHTICK! It breaks transitivity!
        {
            this.Direction = direction;
            if (CAT.ContainsKey(this))
            {
                if (ans == null)
                    ans = new List<int>(CAT[this]);
                else
                    ans.AddRange(CAT[this]);
            }
        }
        this.Direction = saveDirection;

        if (Constants.ALLOW_HEAD_ON_COLLISION == false)
        {
            this.setOppositeMove();
            if (CAT.ContainsKey(this)) // Check direction too now
            {
                if (ans == null)
                    ans = new List<int>(CAT[this]);
                else
                    ans.AddRange(CAT[this]);
            }
            this.setOppositeMove();
        }

        if (ans != null)
            return ans;
        else
            return TimedMove.emptyList;
    }

    public void IncrementConflictCounts(ConflictAvoidanceTable conflictAvoidance,
                                        Dictionary<int, int> conflictCounts, Dictionary<int, List<int>> conflictTimes)
    {
        IReadOnlyList<int> colliding = this.GetColliding(conflictAvoidance);
        foreach (int agentNum in colliding)
        {
            if (conflictCounts.ContainsKey(agentNum) == false)
                conflictCounts[agentNum] = 1;
            else
                conflictCounts[agentNum] += 1;
            if (conflictTimes.ContainsKey(agentNum) == false)
                conflictTimes[agentNum] = new List<int>(4) { this.time };
            else
                conflictTimes[agentNum].Add(this.time);
        }
    }
}
