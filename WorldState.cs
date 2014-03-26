﻿using System;
using System.Linq;
using System.Collections.Generic;

namespace CPF_experiment
{
    /// <summary>
    /// Describes a node in the A* search space.
    /// </summary>
    public class WorldState : IComparable<IBinaryHeapItem> , IBinaryHeapItem
    {
        public int makespan; // Total time steps passed, max(agent makespans)
        public int g; // Sum of agent makespans until they reach their goal
        public int h;
        public bool notExpanded; // Used only by AStarWithPartialExpansionBasic. Consider moving.
        public AgentState[] allAgentsState;
        public WorldState prevStep;
        private int binaryHeapIndex;
        public int potentialConflictsCount;
        public int cbsInternalConflictsCount;
        public byte nextFvalue; // Used only by AStarWithPartialExpansionBasic. Consider moving.
        public HashSet<TimedMove> currentMoves;

        /// <summary>
        /// Create a state with the given state for every agent.
        /// </summary>
        /// <param name="allAgentsState"></param>
        public WorldState(AgentState[] allAgentsState)
        {
            this.allAgentsState = allAgentsState;
            this.makespan = 0;
            this.g = 0;
            this.potentialConflictsCount = 0;
            this.cbsInternalConflictsCount = 0;
            this.notExpanded = true;
            this.currentMoves = new HashSet<TimedMove>();
        }

        /// <summary>
        /// Copy constructor. Doesn't copy g!
        /// </summary>
        /// <param name="cpy"></param>
        public WorldState(WorldState cpy)
        {
            this.makespan = cpy.makespan;
            this.h = cpy.h;
            this.allAgentsState = new AgentState[cpy.allAgentsState.Length];
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                this.allAgentsState[i] = new AgentState(cpy.allAgentsState[i]);
            }
            this.notExpanded = true; // Not cpy.notExpanded?
            this.currentMoves = new HashSet<TimedMove>(cpy.currentMoves);
        }

        /// <summary>
        /// Copies a state with only a sub group of the original agents.
        /// Doesn't copy h or makespan!
        /// </summary>
        /// <param name="cpy"></param>
        /// <param name="numOfAgents"></param>
        public WorldState(WorldState cpy, int numOfAgents)
        {
            this.allAgentsState = new AgentState[numOfAgents];
            for (int i = 0; i < numOfAgents; i++)
            {
                this.allAgentsState[i] = new AgentState(cpy.allAgentsState[i]);
            }
            this.notExpanded = true;
            //TODO: Copy the relevant parts of cpy.currentMoves
        }

        /// <summary>
        /// Creates a new state by extracting a subset of the agents from
        /// the original Travor_WorldState. We overload the constructor because
        /// while building our pattern database, we rewrite the problem and
        /// therefore need to make a deep copy of the state data structures so
        /// as to not overwrite the original problem. The ultimate solution
        /// would be to rework the code to remove static variables so that we
        /// can instantiate subproblems without affecting the original data
        /// structures.
        /// </summary>
        /// <param name="allAgentsState">A set of agent states in the original problem.</param>
        /// <param name="vAgents">A list of indices referring to the subset of agents we want to extract.</param>
        public WorldState(AgentState[] allAgentsState, List<uint> vAgents)
        {
            // Copy specified agents:
            this.allAgentsState = vAgents.Select<uint, AgentState>(index => new AgentState(allAgentsState[index])).ToArray<AgentState>();
            makespan = 0;
            g = 0;
        }        
        
        public bool GoalTest(int minDepth)
        {
            if (makespan >= minDepth)
                return h == 0; // That's crazy! A node that is close to the goal might also get h==0.
                               // Our specific heuristic doesn't behave that way, though.
            return false;
        }

        /// <summary>
        /// Used when WorldState objects are put in the open list priority queue
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public virtual int CompareTo(IBinaryHeapItem other)
        {
            WorldState that = (WorldState)other;
            if (this.h + this.g < that.h + that.g)
                return -1;
            if (this.h + this.g > that.h + that.g)
                return 1;

            if (this.potentialConflictsCount < that.potentialConflictsCount)
                return -1;
            if (this.potentialConflictsCount > that.potentialConflictsCount)
                return 1;

            if (this.cbsInternalConflictsCount < that.cbsInternalConflictsCount)
                return -1;
            if (this.cbsInternalConflictsCount > that.cbsInternalConflictsCount)
                return 1;

            // f, conflicts and internal conflicts being equal, prefer nodes with a larger g
            // - they're closer to the goal so less nodes would probably be generated by them on the way to it.
            if (this.g < that.g)
                return 1;
            if (this.g > that.g)
                return -1;

            return 0;
        }
        

        /// <summary>
        /// Calculate and set the g of the state as the sum of the different agent g values.
        /// </summary>
        public virtual void CalculateG()
        {
            g = 0;
            foreach (AgentState singleAgentState in allAgentsState)
            {
                if (singleAgentState.atGoal())
                    g += singleAgentState.arrivalTime;
                else
                    g += makespan;
            }
        }

        public override string ToString()
        {
            string ans = "makespan: " + makespan + ", h: " + h + ", g: " + g + "\n";
            foreach (AgentState temp in allAgentsState)
            {
                ans +=" agent " + temp.agent.agentNum + ": " + temp.last_move + "\n";
            }
            return ans;
        }

        /// <summary>
        /// Returns the last move of all the agents in this state
        /// </summary>
        /// <returns>A list of points</returns>
        public LinkedList<Move> GetAgentsMoves()
        {
            LinkedList<Move> ans = new LinkedList<Move>();
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                ans.AddFirst(new Move(allAgentsState[i].last_move));
            }
            return ans;
        }

        public Move getSingleAgentMove(int index)
        {
            return new Move(allAgentsState[index].last_move);
        }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        /// <returns></returns>
        public int getIndexInHeap() { return binaryHeapIndex; }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        /// <returns></returns>
        public void setIndexInHeap(int index) { binaryHeapIndex = index; }

        /// <summary>
        /// Checks for internal conflicts
        /// </summary>
        /// <returns></returns>
        public bool isValid()
        {
            for (int i = 0; i < this.allAgentsState.Length; i++)
            {
                for (int j = i+1; j < this.allAgentsState.Length; j++)
                {
                    // Internal conflict
                    if (this.allAgentsState[i].last_move.isColliding(this.allAgentsState[j].last_move))
                        return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Only the agent states are used in the hash.
        /// The g, makespan, h, potentialConflictsCount, cbsInternalConflictsCount and others are ignored, as neccesary.
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            int ans = 0;
            unchecked
            {
                for (int i = 0 ; i < allAgentsState.Length; i++)
                {
                    ans += allAgentsState[i].GetHashCode() * Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length];
                }
            }
            return ans;
        }

        /// <summary>
        /// Only the AgentStates are compared.
        /// g, makespan, h, potentialConflictsCount, cbsInternalConflictsCount and others are ignored, as necessary.
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            WorldState that = (WorldState)obj;
            if (this.allAgentsState.Length != that.allAgentsState.Length)
                return false;
            for (int i = 0; i < this.allAgentsState.Length; i++)
            {
                if (!this.allAgentsState[i].Equals(that.allAgentsState[i]))
                    return false;
            }
            return true;
        }
        
        public virtual int conflictsCount(HashSet<TimedMove> conflictAvoidence)
        {
            int ans = 0;
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                if (allAgentsState[i].last_move.isColliding(conflictAvoidence))
                    ans++;
            }
            return ans;
        }

        public virtual int conflictsCount(HashSet_U<TimedMove> conflictAvoidence)
        {
            int ans = 0;
            for (int i = 0; i < allAgentsState.Length; i++)
            {
                if (allAgentsState[i].last_move.isColliding(conflictAvoidence))
                    ans++;
            }
            return ans;
        }
    }
}
