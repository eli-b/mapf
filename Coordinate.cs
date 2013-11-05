using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CPF_experement
{
    class Coordinate
    {
        protected ushort x;
        protected ushort y;
        sbyte direction;

        public Coordinate() { }

        public Coordinate(int x, int y, int direction)
        {
            this.x = (ushort)x;
            this.y = (ushort)y;
            this.direction = (sbyte)direction;
        }

        public override bool Equals(object obj)
        {
            Coordinate other = (Coordinate)obj;
            if (this.direction != -1 && other.direction != -1 && this.direction != other.direction)
                return false;
            return (this.x == other.x && this.y == other.y);
        }

        public int getX() { return x; }
        public int getY() { return y; }
    }

    //class CoordinateForConflictRatio
    //{
    //    int timedCoordinate;

    //    public CoordinateForConflictRatio(ProblemInstance ins, AgentState state)
    //    {
    //        timedCoordinate = state.currentStep * (ins.maxCardinality + 26) + state.direction * 5 + ins.getCardinality(state);
    //    }

    //    public override bool Equals(object obj)
    //    {
    //        CoordinateForConflictRatio other = (CoordinateForConflictRatio)obj;
    //        if (this.timedCoordinate != other.timedCoordinate )
    //            return false;
    //        return true;
    //    }

    //    public override int GetHashCode()
    //    {
    //        return timedCoordinate;
    //    }
    //}
}
