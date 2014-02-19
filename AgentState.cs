using System;
using System.Collections.Generic;

namespace CPF_experiment
{
    [Serializable] public class AgentState : IComparable<IBinaryHeapItem>,IBinaryHeapItem
    {
        public int pos_X;
        public int pos_Y;
        public int h;
        public Agent agent;
        public int arrivalTime;
        public int currentStep;
        public sbyte direction;
        private int binaryHeapIndex;
        public int potentialConflicts;
        public ushort potentialConflictsID;
        [NonSerialized] public AgentState prev;

        public AgentState(int pos_X, int pos_Y, Agent agent)
        {
            this.pos_X=pos_X;
            this.pos_Y=pos_Y;
            direction = 0;
            this.agent = agent;
        }

        public AgentState(int startX, int startY, int goalX, int goalY, int agentId)
            : this(startX, startY, new Agent(goalX, goalY, agentId))
        {}

        public AgentState(AgentState copy)
        {
            this.pos_X = copy.pos_X;
            this.pos_Y = copy.pos_Y;
            this.agent = copy.agent;
            this.h = copy.h;
            this.arrivalTime = copy.arrivalTime;
            this.direction = copy.direction;
            this.currentStep = copy.currentStep;
        }

        public void swapCurrentWithGoal()
        {
            int nTemp = pos_X;
            pos_X = agent.Goal_X;
            agent.Goal_X = nTemp;
            nTemp = pos_Y;
            pos_Y = agent.Goal_Y;
            agent.Goal_Y = nTemp;
        }

        /// <summary>
        /// Actually moves the agent and recalculates its heuristic
        /// </summary>
        /// <param name="deltaX"></param>
        /// <param name="deltaY"></param>
        public void move(int direction)
        {
            int deltaX = WorldState.operators[direction, 0];
            int deltaY = WorldState.operators[direction, 1];
            pos_X = pos_X + deltaX;
            pos_Y = pos_Y + deltaY;
            currentStep = currentStep + 1;
            this.direction = (sbyte)direction;

            // If performed a non STAY move and reached the agent's goal - store the arrival time
            if (((deltaX != 0) || (deltaY != 0)) && (this.atGoal()))
                this.arrivalTime = currentStep;
        }

        /// <summary>
        /// Checks it the agent is at its goal location
        /// </summary>
        /// <returns>True if the agent has reached its goal location</returns>
        public bool atGoal()
        {
            return ((this.pos_X == this.agent.Goal_X) && (this.pos_Y == this.agent.Goal_Y));
        }

        /// <summary>
        /// BH_Iteam implimatation
        /// </summary>
        /// <returns></returns>
        public int getIndexInHeap() { return binaryHeapIndex; }
        /// <summary>
        /// BH_Iteam implimatation
        /// </summary>
        /// <returns></returns>
        public void setIndexInHeap(int index) { binaryHeapIndex = index; }

        public override bool Equals(object obj)
        {
            AgentState that = (AgentState)obj;
            
            ///
            /// Roni: Currently Equals() also compares the direction, because this is needed for Standley's Operator Decomposition.
            /// However, this does waste time for the classic A*, so we might want to create a subclass of AgentState
            /// that is custom for Standley's OD.
            /// 
            if (CBS_LocalConflicts.isDnC == true)
                if (this.currentStep != that.currentStep)
                    return false;

            if (this.pos_X == that.pos_X && this.pos_Y == that.pos_Y && this.agent == that.agent)
            {
                return true;
            }
            return false;
        }

        public override int GetHashCode()
        {
            unchecked
            {
                return (this.agent.agentNum) + (this.pos_X * 5) + (this.pos_Y * 7);
            }
        }

        public LinkedList<Move> GetMove()
        {
            LinkedList<Move> ans = new LinkedList<Move>();
            ans.AddFirst(new Move(pos_X, pos_Y, direction));
            return ans;
        }

        public int CompareTo(IBinaryHeapItem other)
        {
            AgentState that = (AgentState)other;
            if (this.h + this.currentStep < that.h + that.currentStep)
                return -1;
            if (this.h + this.currentStep > that.h + that.currentStep)
                return 1;



            if (this.potentialConflictsID < that.potentialConflictsID)
                return -1;
            if (this.potentialConflictsID > that.potentialConflictsID)
                return 1;


            if (this.potentialConflicts < that.potentialConflicts)
                return -1;
            if (this.potentialConflicts > that.potentialConflicts)
                return 1;



            if (this.currentStep < that.currentStep)
                return 1;
            if (this.currentStep > that.currentStep)
                return -1;
            return 0;
        }

        public override string ToString()
        {
            return " step-"+currentStep+" possison ("+pos_X+","+pos_Y+")";
        }
}
 // Compares two AgentStates according to their AgentNum
    // This method is used for Rtrevor
    class AgentsNumsComparator : IComparer<AgentState>
    {
        public int Compare(AgentState x, AgentState y)
        {
            if (x.agent.agentNum > y.agent.agentNum)
                return 1;
            else if (x.agent.agentNum < y.agent.agentNum)
                return -1;
            else
                return 0;
        }
    }
}
