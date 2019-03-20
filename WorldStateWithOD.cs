using System.Collections.Generic;

namespace CPF_experiment
{
    /// <summary>
    /// This class represents a state in the A* search with operator decomposition,
    /// as proposed by Scott Standley's AAAI paper in 2010.
    /// More specifically, states can represent a partial move, in which only some of the agents have moved
    /// and the other have not yet moved in this turn. 
    /// </summary>
    public class WorldStateWithOD : WorldState
    {
        /// <summary>
        /// Marks the index of the agent that will move next. 
        /// All agents with index less than agentTurn are assumed to have already chosen their move for this time step,
        /// while agents with higher index have not chosen their move yet.
        /// </summary>
        public int agentTurn;

        public WorldStateWithOD(AgentState[] states, int minDepth = -1, int minCost = -1, MDDNode mddNode = null)
            : base(states, minDepth, minCost, mddNode)
        {
            this.agentTurn = 0;
        }
        
        public WorldStateWithOD(WorldStateWithOD cpy) : base(cpy)
        {
            this.agentTurn = cpy.agentTurn;
        }
        
        /// <summary>
        /// Used for PDB stuff only
        /// </summary>
        /// <param name="states"></param>
        /// <param name="relevantAgents"></param>
        public WorldStateWithOD(AgentState[] states, List<uint> relevantAgents) : base(states, relevantAgents)
        {
            this.agentTurn = 0;
        }

        public override ProblemInstance ToProblemInstance(ProblemInstance initial)
        {
            WorldState state = this;
            if (this.agentTurn != 0)
            {
                // CBS doesn't handle partially expanded nodes well.
                // Use the last fully expanded node and add the additional moves as must conds:
                state = this.prevStep; // Points to the last fully expanded node.
            }

            ProblemInstance subproblem = initial.Subproblem(state.allAgentsState); // Can't use base's method because we're operating on a different object

            if (this.agentTurn != 0)
            {
                subproblem.parameters = new Dictionary<string,object>(subproblem.parameters); // Use a copy to not pollute general problem instance with the must constraints
                if (subproblem.parameters.ContainsKey(CBS.MUST_CONSTRAINTS) == false)
                    subproblem.parameters[CBS.MUST_CONSTRAINTS] = new HashSet_U<CbsConstraint>();
                var mustConstraints = (HashSet_U<CbsConstraint>)subproblem.parameters[CBS.MUST_CONSTRAINTS];
                var newMustConstraints = new HashSet<CbsConstraint>();
                for (int i = 0; i < this.agentTurn; ++i)
                {
                    newMustConstraints.Add(new CbsConstraint(this.allAgentsState[i].agent.agentNum, this.allAgentsState[i].lastMove));
                }
                mustConstraints.Join(newMustConstraints);
            }

            return subproblem;
        }

        /// <summary>
        /// Set the optimal solution of this node as a problem instance.
        /// </summary>
        /// <param name="solution"></param>
        public override void SetSolution(SinglePlan[] solution)
        {
            if (this.agentTurn == 0)
                this.singlePlans = SinglePlan.GetSinglePlans(this);
            else
                this.singlePlans = SinglePlan.GetSinglePlans(this.prevStep);
                // ToProblemInstance gives the last proper state as the problem to solve,
                // with must constraints to make the solution go through the steps already
                // taken from there.

            for (int i = 0; i < solution.Length; ++i)
                this.singlePlans[i].ContinueWith(solution[i]);
        }

        /// <summary>
        /// Returns a hash value for the given state (used in Hash based data structures).
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hash = Constants.PRIMES_FOR_HASHING[0];
                hash = hash * Constants.PRIMES_FOR_HASHING[1] + base.GetHashCode();
                hash = hash * Constants.PRIMES_FOR_HASHING[2] + this.agentTurn;
                return hash;
            }
        }

        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            var that = (WorldStateWithOD)obj;
            if (that.agentTurn != this.agentTurn)
            // It's tempting to think that this check is enough to allow equivalence over different times,
            // because it differentiates between a state where all agents have moved and its
            // child where the first agent WAITed, allowing the child
            // to be generated because it isn't a hit in the closed list.
            // But it isn't enough.
            // This may a partially generated node,
            // and we may have already gotten to this specific set of agent positions,
            // but from a different set of locations, so the allowed moves of the remaining agents
            // that haven't already moved would be different.
                return false;

            if (this.agentTurn == 0) // All agents have moved, safe to ignore direction information.
                return base.Equals(obj);

            if (this.allAgentsState.Length != that.allAgentsState.Length)
                return false;

            // Comparing the agent states:
            for (int i = 0; i < this.allAgentsState.Length; ++i)
            {
                if (this.allAgentsState[i].Equals(that.allAgentsState[i]) == false)
                    return false;
                if (i < this.agentTurn) // Agent has already moved in this step
                {
                    bool mightCollideLater = false;
                    for (int j = this.agentTurn; j < this.allAgentsState.Length; j++)
                    {
                        if (this.allAgentsState[i].lastMove.x == this.allAgentsState[j].lastMove.x &&
                            this.allAgentsState[i].lastMove.y == this.allAgentsState[j].lastMove.y) // Can't just remove the direction and use IsColliding since the moves' time is different, so they'll never collide
                        {
                            mightCollideLater = true;
                            break;
                        }
                    }

                    if (mightCollideLater == true) // Then check the direction too
                    {
                        if (this.allAgentsState[i].lastMove.direction != Move.Direction.NO_DIRECTION &&
                             that.allAgentsState[i].lastMove.direction != Move.Direction.NO_DIRECTION &&
                             this.allAgentsState[i].lastMove.direction != that.allAgentsState[i].lastMove.direction) // Can't just use this.allAgentsState[i].lastMove.Equals(that.allAgentsState[i].lastMove) because TimedMoves don't ignore the time.
                            return false;
                    }
                }
            }
            return true;
        }

        /// <summary>
        /// Used when WorldStateWithOD objects are put in the open list priority queue.
        /// All other things being equal, prefers nodes where more agents have moved.
        /// G is already preferred, but this helps when the last move was a WAIT at the
        /// goal, which doesn't increment G.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public override int CompareTo(IBinaryHeapItem other)
        {
            int res = base.CompareTo(other);
            if (res != 0)
                return res;

            var that = (WorldStateWithOD)other;

            // Further tie-breaking
            // Prefer more fully generated nodes:
            // For the same F, they're probably closer to the goal.
            // The goal isn't necessarily a fully expanded node.
            // A*+OD may finish when all agents reached their goal even if it isn't a fully expanded state, and that's a nice feature!
            // So we prefer more fully generated nodes just because it gives a more DFS-like behavior
            // on the heuristic's fast path to the goal.
            if (this.agentTurn == 0 && that.agentTurn != 0)
                return -1;
            if (that.agentTurn == 0 && this.agentTurn != 0)
                return 1;
            return that.agentTurn.CompareTo(this.agentTurn); // Notice the order inversion - bigger is better.
        }

        /// <summary>
        /// Counts for last agent to move only, the counts from the previous agents to move are accumulated from the parent node.
        /// </summary>
        /// <param name="conflictAvoidance"></param>
        /// <returns></returns>
        public override void UpdateConflictCounts(IReadOnlyDictionary<TimedMove, List<int>> conflictAvoidance)
        {
            int lastAgentToMove = agentTurn - 1;
            if (agentTurn == 0)
                lastAgentToMove = allAgentsState.Length - 1;

            allAgentsState[lastAgentToMove].lastMove.UpdateConflictCounts(conflictAvoidance,
                                                                          this.cbsInternalConflicts, this.conflictTimes);
        }
    }
}
