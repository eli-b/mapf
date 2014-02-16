
namespace CPF_experiment
{
    /// <summary>
    /// This class represents a single move of an agent. 
    /// It includes the target location of the move and the direction of the move.
    /// The start location can be extracted using the direction, with the method 
    /// </summary>
    public class Move
    {
        /// <summary>
        /// This constant is set to the direction field to mark that this move does not hold a direction.
        /// <remarks> 
        /// Directionless moves are a poor design choice instead of making a class that represents a 2D point,
        /// but will probably be more efficient.
        /// </remarks>
        /// </summary>
        public static int NO_DIRECTION = -1;

        public int x;
        public int y;
        public int direction;

        public Move() { }

        public Move(int x, int y, int direction)
        {
            this.x = x;
            this.y = y;
            this.direction = direction;
        }

        public Move(Move cpy)
        {
            this.x = cpy.x;
            this.y = cpy.y;
            this.direction = cpy.direction;
        }

        /// <summary>
        /// TODO: Replace direction codes with an enum or at least a constant.
        /// </summary>
        /// <returns></returns>
        public Move GetOppositeMove()
        {
            switch (direction)
            {
                case 1:
                    return new Move(this.x + 1, this.y, 3);
                case 2:
                    return new Move(this.x, this.y - 1, 4);
                case 3:
                    return new Move(this.x - 1, this.y, 1);
                case 4:
                    return new Move(this.x, this.y + 1, 2);
                case 5:
                    return new Move(this.x + 1, this.y - 1, 7);
                case 6:
                    return new Move(this.x - 1, this.y - 1, 8);
                case 7:
                    return new Move(this.x - 1, this.y + 1, 5);
                case 8:
                    return new Move(this.x + 1, this.y + 1, 6);
            }
            return this;
        }


        /// <summary>
        /// Returns a copy of this move, where the direction is set to -1
        /// </summary>
        /// <returns></returns>
        public Move GetMoveWithoutDirection()
        {
            Move copy =  new Move(this);
            copy.direction = NO_DIRECTION;
            return copy;
        }

        public void setOppositeMove()
        {
            switch (this.direction)
            {
                case 1:
                    {
                        this.x++; 
                        this.direction = 3;
                        break;
                    }
                case 2:
                     this.y --; this.direction =4;break;
                case 3:
                    this.x--;this.direction = 1;break;
                case 4:
                    this.y ++; this.direction =2;break;
                case 5:
                    this.x ++; this.y --;this.direction = 7;break;
                case 6:
                    this.x --; this.y --; this.direction =8;break;
                case 7:
                    this.x --; this.y ++;this.direction = 5;break;
                case 8:
                    this.x ++; this.y ++; this.direction =6;break;
            }
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
            this.direction = NO_DIRECTION;
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
            if (this.x == other.x && this.y == other.y)
                return true;
            switch (direction)
            {
                case 0:
                    return false;
                case 1:
                    return (other.x == this.x + 1 && other.y == this.y && other.direction == 3);
                case 2:
                    return (other.x == this.x && other.y == this.y-1 && other.direction == 4);
                case 3:
                    return (other.x == this.x - 1 && other.y == this.y && other.direction == 1);
                case 4:
                    return (other.x == this.x && other.y == this.y+1 && other.direction == 2);
                case 5:
                    return (other.x == this.x + 1 && other.y == this.y-1 && other.direction == 7);
                case 6:
                    return (other.x == this.x - 1 && other.y == this.y-1 && other.direction == 8);
                case 7:
                    return (other.x == this.x - 1 && other.y == this.y+1 && other.direction == 5);
                case 8:
                    return (other.x == this.x + 1 && other.y == this.y+1 && other.direction == 6);
            }
            return false;
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
        public bool isColliding(int x, int y, int direction)
        {
            if (this.x == x && this.y == y)
                return true;
            switch (this.direction)
            {
                case 0:
                    return false;
                case 1:
                    return (x == this.x + 1 && y == this.y && direction == 3);
                case 2:
                    return (x == this.x && y == this.y - 1 && direction == 4);
                case 3:
                    return (x == this.x - 1 && y == this.y && direction == 1);
                case 4:
                    return (x == this.x && y == this.y + 1 && direction == 2);
                case 5:
                    return (x == this.x + 1 && y == this.y - 1 && direction == 7);
                case 6:
                    return (x == this.x - 1 && y == this.y - 1 && direction == 8);
                case 7:
                    return (x == this.x - 1 && y == this.y + 1 && direction == 5);
                case 8:
                    return (x == this.x + 1 && y == this.y + 1 && direction == 6);
            }
            return false;
        }

        public static int getDirection(int to_x, int to_y, int from_x, int from_y)
        {
            int temp = 0;
            temp += (to_x - from_x);
            temp += (to_y - from_y) * 2;
            switch (temp)
            {
                case 0:
                    return 0;
                case -1:
                    return 1;
                case 2:
                    return 2;
                case 1:
                    return 3;
                case -2:
                    return 4;
            }
            return 0;
        }        
        public override int GetHashCode()
        {
            return x + (y * 10000);
        }

        /// <summary>
        /// Compare two Move objects. 
        /// If one of the Move objects do not have a direction that is set (i.e. direction==-1)
        /// then the direction part of the Move is ignored.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            Move that = (Move)obj;
            return (this.x == that.x && this.y == that.y && ((this.direction == NO_DIRECTION) || (that.direction == NO_DIRECTION) || (this.direction == that.direction)));
        }


        public override string ToString()
        {
            return this.x + "," + this.y;
        }
    }    
}
