using System.Collections.Generic;

namespace CPF_experiment
{
    /// <summary>
    /// This class represents a single move of an agent. 
    /// It includes the target location of the move and the direction of the move.
    /// The start location can be extracted using the direction, with the method GetSource().
    /// Consider adding a GetNeighbors generator instead of exposing directionToDeltas
    /// </summary>
    public class Move
    {
        public enum Direction : int
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
        }

        public const int FIRST_NON_WAIT = (int)Direction.North;
        public const int LAST_NON_DIAG_MOVE = (int)Direction.West;
        public const int LAST_REAL_MOVE = (int)Direction.NorthWest;

        public int x;
        public int y;
        public int direction;

        public Move() { }

        public Move(int x, int y, int direction)
        {
            // TODO: Consider calling setup(x, y, direction) instead of this duplication
            this.x = x;
            this.y = y;
            this.direction = direction;
        }

        public Move(Move cpy)
        {
            // TODO: Consider calling setup(cpy) instead of this duplication
            this.x = cpy.x;
            this.y = cpy.y;
            this.direction = cpy.direction;
        }

        public static readonly int[,] directionToDeltas = {
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
        private static readonly int[,] directionToOppositeDeltas = {
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

        private static readonly Direction[] directionToOppositeDirection = {
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

        // <remarks>
        // Deltas have to be used +1
        // </remarks>
        private static readonly Direction[,] deltasToDirection = {
            {Direction.NorthWest, Direction.North, Direction.NorthEast},
            {Direction.West, Direction.Wait, Direction.East},
            {Direction.SouthWest, Direction.South, Direction.SouthEast}
        };

        public Move GetSource()
        {
            var source_x = this.x + directionToOppositeDeltas[direction, 0];
            var source_y = this.y + directionToOppositeDeltas[direction, 1];
            return new Move(source_x, source_y, (int)Direction.NO_DIRECTION);
        }

        public Move GetOppositeMove()
        {
            if (direction == (int)Direction.Wait || direction == (int)Direction.NO_DIRECTION)
                return this; // Not Move(this). TODO: Make sure this is correct.
            return new Move(this.x + directionToOppositeDeltas[direction, 0],
                            this.y + directionToOppositeDeltas[direction, 1],
                            (int)directionToOppositeDirection[direction]);
        }

        /// <summary>
        /// Returns a copy of this move, where the direction is set to -1
        /// </summary>
        /// <returns></returns>
        public Move GetMoveWithoutDirection()
        {
            Move copy =  new Move(this);
            copy.direction = (int)Direction.NO_DIRECTION;
            return copy;
        }

        /// <summary>
        /// Changes this move to represent its opposite. Warning: Changes the hash. Not safe after the object is put in a hash table!
        /// </summary>
        public void setOppositeMove()
        {
            this.x += directionToOppositeDeltas[direction, 0];
            this.y += directionToOppositeDeltas[direction, 1];
            // Consider making directionToOppositeDeltas a jagged array,
            // reducing the number of table lookups to one for the above lines
            // since both entries in the sub-array are needed
            this.direction = (int)directionToOppositeDirection[direction];
        }

        public void setup(int x, int y, int direction)
        {
            this.x = x;
            this.y = y;
            this.direction = direction;
        }

        public void setup(Move cpy)
        {
            this.x = cpy.x;
            this.y = cpy.y;
            this.direction = cpy.direction;
        }

        /// <summary>
        /// Removes the direction of this Move
        /// </summary>
        public void RemoveDirection()
        {
            this.direction = (int)Direction.NO_DIRECTION;
        }

        /// <summary>
        /// Check if the given move collides with this move.
        /// This includes:
        /// 1. Head on collision
        /// 2. When other move targets the same location.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool isColliding(Move other)
        {
            return isColliding(other.x, other.y, other.direction);
        }

        /// <summary>
        /// Check if colliding with an agent moving to the given x,y from the given direction.
        /// This includes:
        /// 1. Head on collision
        /// 2. When other move targets the same location.
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        public bool isColliding(int other_x, int other_y, int other_direction)
        {
            // Same target check
            if (this.x == other_x && this.y == other_x)
                return true;
            // Head-on collision check
            var source_x = this.x + directionToOppositeDeltas[this.direction, 0];
            var source_y = this.y + directionToOppositeDeltas[this.direction, 1];
            var other_source_x = other_x + directionToOppositeDeltas[other_direction, 0];
            var other_source_y = other_y + directionToOppositeDeltas[other_direction, 1];
            return this.x == other_source_x && this.y == other_source_y && other_x == source_x && other_y == source_y;
        }

        public bool isColliding(ISet<Move> group)
        {
            // Same target check
            int saved_direction = direction;
            direction = (int)Direction.NO_DIRECTION;
            if (group.Contains(this))
            {
                direction = saved_direction;
                return true;
            }
            direction = saved_direction;

            setOppositeMove();
            if (group.Contains(this))
            {
                setOppositeMove();
                return true;
            }
            setOppositeMove();
            return false;
        }

        public static int getDirection(int to_x, int to_y, int from_x, int from_y)
        {
            return (int)deltasToDirection[to_x - from_x + 1, to_y - from_y + 1]; // +1 since indexing starts from 0
        }

        public override int GetHashCode()
        {
            // TODO: Make sure the Move's x and y are never changed after it's put into a collection that uses hashes
            unchecked // wrap-around is fine in hash functions
            {
                int hash = 17;
                hash = 23 * hash + x;
                hash = 23 * hash + y;
                // NOT including the direction in the hash.
                // Notice that Moves with a set direction that have the same target will be 
                // non-Equal but with the same hash. This is allowed, but may
                // cause inefficiencies in hashtables.
                return hash;
            }
        }

        /// <summary>
        /// Compare two Move objects. 
        /// If one of the Move objects does not have a direction that is set (i.e. direction == -1)
        /// then the direction part of the Move is ignored.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            Move that = (Move) obj;
            return (this.x == that.x && this.y == that.y &&
                    ((this.direction == (int)Direction.NO_DIRECTION) || (that.direction == (int)Direction.NO_DIRECTION) || 
                     (this.direction == that.direction)));
        }

        public override string ToString()
        {
            return this.x + "," + this.y; // not describing the direction
        }
    }    
}
