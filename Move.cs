using System.Collections.Generic;

namespace mapf;

public enum Direction : byte
{
    Wait = 0,
    North,
    East,
    South,
    West,
    NorthEast,
    SouthEast,
    SouthWest,
    NorthWest,
    /// <summary>
    /// This constant is set to the direction field to mark that this move does not hold a direction.
    /// <remarks> 
    /// Directionless moves are a poor design choice instead of making a class that represents a 2D point,
    /// but will probably be more efficient.
    /// </remarks>
    /// </summary>
    NO_DIRECTION = 9,
};

/// <summary>
/// This class represents a single move of an agent. 
/// It includes the target location of the move and the direction of the move.
/// The start location can be extracted using the direction, with the method GetSource().
/// </summary>
public class Move
{
    public const int FIRST_NON_WAIT = (int)Direction.North;
    public const int NUM_NON_DIAG_MOVES = 5;
    public const int NUM_DIRECTIONS = 9;

    public int X {  get; set; }
    public int Y { get; set; }

    public Direction Direction { get; set; }

    public Move() { }

    public Move(int x, int y, Direction direction)
    {
        X = x;
        Y = y;
        Direction = direction;
    }

    public Move(Move cpy) : this( cpy.X, cpy.Y, cpy.Direction ) {}

    protected static readonly int[,] directionToDeltas = {
        {0,   0, }, // Wait
        {-1,  0, }, // N
        {0,   1, }, // E
        {1,   0, }, // S
        {0,  -1, }, // W
        {-1,  1, }, // NE
        {1,   1, }, // SE
        {1,  -1, }, // SW
        {-1, -1, }, // NW
        {0,   0, }, // no direction
    };

    // This exists for the marginal gain of not having to lookup the opposite direction and then its deltas
    protected static readonly int[,] directionToOppositeDeltas = {
        {0,   0, }, // Wait to Wait
        {1,   0, }, // N to S
        {0,  -1, }, // E to W
        {-1,  0, }, // S to N
        {0,   1, }, // W to E
        {1,  -1, }, // NE to SW
        {-1, -1, }, // SE to NW
        {-1,  1, }, // SW to NE
        {1,   1, }, // NW to SE
        {0,   0, } // no direction to no direction
    };

    protected static readonly Direction[] validDirectionsNoDiag = {
        Direction.Wait,
        Direction.North,
        Direction.East,
        Direction.South,
        Direction.West,
    };

    protected static readonly Direction[] validDirections = {
        Direction.Wait,
        Direction.North,
        Direction.East,
        Direction.South,
        Direction.West,
        Direction.NorthEast,
        Direction.SouthEast,
        Direction.SouthWest,
        Direction.NorthWest,
    };

    protected static readonly Direction[] directionToOppositeDirection = {
        Direction.Wait, // Wait to Wait
        Direction.South, // N to S
        Direction.West, // E to W
        Direction.North, // S to N
        Direction.East, // W to E
        Direction.SouthWest, // NE to SW
        Direction.NorthWest, // SE to NW
        Direction.NorthEast, // SW to NE
        Direction.SouthEast, // NW to SE
        Direction.NO_DIRECTION // no direction to no direction
    };

    /// <remarks>
    /// Deltas have to be used +1
    /// </remarks>
    protected static readonly Direction[,] deltasToDirection = {
        {Direction.NorthWest, Direction.North, Direction.NorthEast},
        {Direction.West, Direction.Wait, Direction.East},
        {Direction.SouthWest, Direction.South, Direction.SouthEast}
    };

    /// <summary>
    /// A generator yielding new adjacent Moves
    /// </summary>
    /// <returns></returns>
    public IEnumerable<Move> GetNextMoves()
    {
        Direction[] directions;
        if (Constants.ALLOW_DIAGONAL_MOVE)
            directions = Move.validDirections;
        else
            directions = Move.validDirectionsNoDiag;
        foreach (Direction op in directions)
        {
            yield return new Move(X + Move.directionToDeltas[(int)op, 0], Y + Move.directionToDeltas[(int)op, 1], op);
        }
    }

    /// <summary>
    /// Change coordinates in specified direction.
    /// </summary>
    /// <param name="direction"></param>
    public virtual void Update(Direction direction)
    {
        X += Move.directionToDeltas[(int)direction, 0];
        Y += Move.directionToDeltas[(int)direction, 1];
        Direction = direction;
    }

    public Move GetSource()
    {
        var source_x = X + directionToOppositeDeltas[(int)Direction, 0];
        var source_y = Y + directionToOppositeDeltas[(int)Direction, 1];
        return new Move(source_x, source_y, Direction.NO_DIRECTION);
    }

    public Move GetOppositeMove()
    {
        if (Direction == Direction.Wait || Direction == Direction.NO_DIRECTION)
            return this; // Not Move(this). TODO: Make sure this is correct.
        return new Move(X + directionToOppositeDeltas[(int)Direction, 0],
                        Y + directionToOppositeDeltas[(int)Direction, 1],
                        directionToOppositeDirection[(int)Direction]);
    }

    /// <summary>
    /// Returns a copy of this move, where the direction is set to Move.Direction.NO_DIRECTION
    /// </summary>
    /// <returns></returns>
    public Move GetMoveWithoutDirection()
    {
        Move copy =  new(this);
        copy.Direction = Direction.NO_DIRECTION;
        return copy;
    }

    /// <summary>
    /// Changes this move to represent its opposite. Warning: Changes the hash. Not safe after the object is put in a hash table!
    /// </summary>
    public void SetOppositeMove()
    {
        X += directionToOppositeDeltas[(int)Direction, 0];
        Y += directionToOppositeDeltas[(int)Direction, 1];
        // Consider making directionToOppositeDeltas a jagged array,
        // reducing the number of table lookups to one for the above lines
        // since both entries in the sub-array are needed
        Direction = directionToOppositeDirection[(int)Direction];
    }

    public void Setup(int x, int y, Direction direction)
    {
        X = x;
        Y = y;
        Direction = direction;
    }

    public void Setup(Move cpy) => Setup(cpy.X, cpy.Y, cpy.Direction);

    /// <summary>
    /// Removes the direction of this Move
    /// </summary>
    public void RemoveDirection() => Direction = Direction.NO_DIRECTION;

    /// <summary>
    /// Check if the given move collides with this move.
    /// This includes:
    /// 1. Head on collision
    /// 2. When other move targets the same location.
    /// </summary>
    public bool IsColliding(Move other) => IsColliding(other.X, other.Y, other.Direction);

    /// <summary>
    /// Check if colliding with an agent moving to the given x,y from the given direction.
    /// This includes:
    /// 1. Head on collision
    /// 2. When other move targets the same location.
    /// TODO: When diagonal moves are allowed, need to also check for diagonal collisions, e.g., (0,0)->(1,1) and (0,1)->(1,0) and such.
    /// No rush, though. We don't currently work with diagonal moves.
    /// </summary>
    public bool IsColliding(int other_x, int other_y, Direction other_direction)
    {
        // Same target check
        if (X == other_x && Y == other_y)
            return true;
        // Head-on collision check
        if (Constants.ALLOW_HEAD_ON_COLLISION == false)
        {
            var source_x = X + directionToOppositeDeltas[(int)Direction, 0];
            var source_y = Y + directionToOppositeDeltas[(int)Direction, 1];
            var other_source_x = other_x + directionToOppositeDeltas[(int)other_direction, 0];
            var other_source_y = other_y + directionToOppositeDeltas[(int)other_direction, 1];
            return X == other_source_x && Y == other_source_y && other_x == source_x && other_y == source_y;
        }
        else
        {
            return false;
        }
    }

    //public bool GetColliding(IReadOnlyDictionary<Move, List<Move>> group)
    //{
    //    // Same target check
    //    Direction saved_direction = direction;
    //    direction = Direction.NO_DIRECTION;
    //    if (group.ContainsKey(this))
    //    {
    //        direction = saved_direction;
    //        return true;
    //    }
    //    direction = saved_direction;

    //    setOppositeMove();
    //    if (group.ContainsKey(this))
    //    {
    //        setOppositeMove();
    //        return true;
    //    }
    //    setOppositeMove();
    //    return false;
    //}

    public static Direction GetDirection(int to_x, int to_y, int from_x, int from_y)
    {
        return deltasToDirection[to_x - from_x + 1, to_y - from_y + 1]; // +1 since indexing starts from 0
    }

    public override int GetHashCode()
    {
        // TODO: Make sure the Move's x and y are never changed after it's put into a collection that uses hashes
        unchecked // wrap-around is fine in hash functions
        {
            int hash = 17;
            hash = 23 * hash + X;
            hash = 23 * hash + Y;
            // NOT including the direction in the hash.
            // We want moves with no direction to be equal to moves with direction on the same coordinate,
            // so they need to have the same hash. Thus, even if this move has a direction it mustn't use it in the hash.
            return hash;
        }
    }

    /// <summary>
    /// Compare two Move objects. 
    /// If one of the Move objects does not have a direction that is set (i.e. direction == Move.Direction.NO_DIRECTION)
    /// then the direction part of the Move is ignored.
    /// </summary>
    /// <param name="obj"></param>
    /// <returns></returns>
    public override bool Equals(object obj)
    {
        if (obj == null)
            return false;
        Move that = (Move) obj;
        return (X == that.X && Y == that.Y &&
                ((Direction == Direction.NO_DIRECTION) || (that.Direction == Direction.NO_DIRECTION) || 
                    (Direction == that.Direction)));
    }

    public override string ToString() => $"({X},{Y})"; // not describing the direction
}    
