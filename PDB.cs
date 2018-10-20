using System;
using System.IO;
using System.Collections.Generic;

namespace CPF_experiment
{
    [Serializable]
    class PDB : IHeuristicCalculator
    {
        [NonSerialized] protected ProblemInstance m_Problem;

        /// <summary>
        /// The agents to extract from the state. Each value in this list is an
        /// index into the <see cref="WorldState"/>.allAgentsStates array. This is
        /// ignored while we are building the pattern database (because it is
        /// assumed that we are initialized by a root node that contains only
        /// the agents we are interested in), but is used during the search effort
        /// in the real problem.
        /// </summary>
        protected List<uint> m_vAgents;

        /// <summary>
        /// Initializes the pattern database by storing references to the
        /// problem instance and also the subset of agents that the pattern
        /// database pertains to.
        /// </summary>
        /// <param name="pi">The problem instance.</param>
        /// <param name="vAgents">The agents that the pattern database should keep track of.</param>
        public virtual void init(ProblemInstance pi, List<uint> vAgents)
        {
            m_Problem = pi;
            m_vAgents = new List<uint>(vAgents);
            m_vAgents.Sort();
        }

        /// <summary>
        /// Returns an estimate of the amount of memory in bytes that is
        /// required to keep this PDB in memory. This is useful in
        /// determining how many agents we should allocate to a particular
        /// pattern database to efficiently utilize our available memory.
        /// </summary>
        /// <returns></returns>
        public virtual UInt64 estimateSize()
        {
            /**
             * @TODO Write an estimation function here!
             */

            return 0;
        }

        public virtual void build() {}

        public virtual uint h(WorldState s)
        {
            return 0;
        }

        /// <summary>
        /// Expands a node. This is done recursively - generating agent possibilities one at a time.
        /// This includes:
        /// - Generating the children
        /// - Inserting them into OPEN
        /// - Insert node into CLOSED
        /// Why does a PDB need to know how to expand nodes? Seems like someone else's job
        /// </summary>
        /// <param name="currentNode">The node to expand</param>
        /// <param name="children">The generated nodes will be filled into this collection</param>
        public void Expand(WorldState currentNode, ICollection<WorldState> children)
        {
            this.Expand(currentNode, 0, children, new HashSet<Move>()); // TODO: Need to think if HashSet is the correct option here.
        }

        /// <summary>
        /// Expands a node. This is done recursively - generating agent possibilities one at a time.
        /// This includes:
        /// - Generating the children
        /// - Inserting them into OPEN
        /// - Insert node into CLOSED
        /// </summary>
        /// <param name="currentNode">The node to expand</param>
        /// <param name="agentIndex">The index of the agent to expand children for</param>
        /// <param name="children">A list in which to set the children states</param>
        /// <param name="previousMoves">A collection of moves performed by the previous agents in this time step (needed to verify that no collisions occur)</param>
        public void Expand(WorldState currentNode, int agentIndex, ICollection<WorldState> children, ICollection<Move> previousMoves)
        {
            WorldState prev = currentNode.prevStep;
            WorldState childNode;

            if (agentIndex == 0) // If this is the first agent that moves
            {
                prev = currentNode;
            }
            if (agentIndex == m_Problem.m_vAgents.Length) // If all the agents have moved
            {
                currentNode.makespan++;
                currentNode.CalculateG();
                children.Add(currentNode);
                return;
            }

            // Try all legal moves of the agent
            foreach (TimedMove agentLocation in currentNode.allAgentsState[agentIndex].lastMove.GetNextMoves())
            {
                if (IsValid(agentLocation, agentIndex, previousMoves))
                {
                    previousMoves.Add(agentLocation);
                    childNode = new WorldState(currentNode);
                    childNode.allAgentsState[agentIndex].MoveTo(agentLocation);
                    childNode.prevStep = prev;
                    Expand(childNode, agentIndex + 1,children, previousMoves);
                    previousMoves.Remove(agentLocation);
                }
            }
        }

        /// <summary>
        /// Check if the move is valid, i.e. not colliding into walls or other agents.
        /// </summary>
        /// <param name="possibleMove">The proposed move (to check if it is valid)</param>
        /// <param name="agentNum">The index of the agent that is proposed to perform the move</param>
        /// <param name="previousAgentMoves">A collection of moves that will be done by the other agents</param>
        /// <returns></returns>
        private bool IsValid(Move possibleMove, int agentNum, ICollection<Move> previousAgentMoves)
        {
            // If the tile is not free (out of the grid or with an obstacle)
            if (m_Problem.IsValid(possibleMove) == false)
                return false;

            // If previous move of another agent will collide with this move
            if (previousAgentMoves.Contains(possibleMove) || previousAgentMoves.Contains(possibleMove.GetOppositeMove()))
                return false;
            return true;
        }

        /// <summary>
        /// Prints header of statistics of a single run to the given output. 
        /// </summary>
        public virtual void OutputStatisticsHeader(TextWriter output) { }

        /// <summary>
        /// Prints statistics of a single run to the given output.
        /// </summary>
        public virtual void OutputStatistics(TextWriter output) { }

        public virtual int NumStatsColumns
        {
            get
            {
                return 0;
            }
        }

        /// <summary>
        /// Clears statistics.
        /// </summary>
        public virtual void ClearStatistics() { }

        public virtual void ClearAccumulatedStatistics() { }
        public virtual void AccumulateStatistics() { }
        public virtual void OutputAccumulatedStatistics(TextWriter output) { }
    }
}
