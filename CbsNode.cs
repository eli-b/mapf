using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;

using ExtensionMethods;

namespace CPF_experiment
{
    [DebuggerDisplay("hash = {GetHashCode()}, f = {f}, g = {g}, h = {h}")]
    public class CbsNode : IComparable<IBinaryHeapItem>, IBinaryHeapItem, IHeuristicSearchNode
    {
        public int g { set; get; }  // Value depends on Constants.costFunction and Constants.sumOfCostsVariant, Sum of agent makespans until they reach their goal
        public int h { get; set; }
        public int hBonus { get; set; }
        /// <summary>
        /// The size of the minimum vertex cover of the node's cardinal conflict graph.
        /// Needs to be saved separately from h to allow speeding up the computation of the heuristic
        /// of the children.
        /// </summary>
        public int minimumVertexCover;
        public SinglePlan[] allSingleAgentPlans;
        public int[] allSingleAgentCosts;
        /// <summary>
        /// A lower estimate of the number of operations (replanning or merging) needed to solve the node.
        /// Used for tie-breaking.
        /// </summary>
        public int minOpsToSolve;
        /// <summary>
        /// For each agent in the problem instance, saves the number of agents from the problem instance that it conflicts with.
        /// Used for choosing the next conflict to resolve by replanning/merging/shuffling, and for tie-breaking.
        /// </summary>
        public int[] countsOfInternalAgentsThatConflict;
        /// <summary>
        /// Counts the number of external agents this node conflicts with.
        /// Used for tie-breaking.
        /// </summary>
        public int totalExternalAgentsThatConflict;
        /// <summary>
        /// Used for tie-breaking.
        /// </summary>
        public int totalConflictsWithExternalAgents;
        /// <summary>
        /// For each agent in the problem instance, maps agent _nums_ it conflicts with, internal or external,
        /// to the number of conflicts betweem them.
        /// Used for book-keeping to maintain countsOfInternalAgentsThatConflict,
        /// totalExternalAgentsThatConflict and minOpsToSolve, and other counts.
        /// </summary>
        public Dictionary<int, int>[] conflictCountsPerAgent;
        /// <summary>
        /// For each agent in the problem instance, maps agent _nums_ of agents it collides with to the time of their first collision.
        /// </summary>
        public Dictionary<int, List<int>>[] conflictTimesPerAgent;
        private int binaryHeapIndex;
        public CbsConflict conflict;
        public CbsConstraint constraint;
        /// <summary>
        /// Forcing an agent to be at a certain place at a certain time
        /// </summary>
        CbsConstraint mustConstraint;
        public CbsNode prev;
        public ushort depth;
        public ushort[] agentsGroupAssignment;
        public ushort replanSize;
        public enum ExpansionState: byte
        {
            NOT_EXPANDED = 0,
            DEFERRED,
            EXPANDED
        }
        /// <summary>
        /// For partial expansion
        /// </summary>
        public ExpansionState agentAExpansion;
        /// <summary>
        /// For partial expansion
        /// </summary>
        public ExpansionState agentBExpansion;
        protected ICbsSolver solver;
        protected ICbsSolver singleAgentSolver;
        public CBS_LocalConflicts cbs;
        public Dictionary<int, int> agentNumToIndex;
        public bool parentAlreadyLookedAheadOf;
        /// <summary>
        /// For tie-breaking
        /// </summary>
        public int totalInternalAgentsThatConflict;
        /// <summary>
        /// For tie-breaking
        /// </summary>
        public int largerConflictingGroupSize;
        /// <summary>
        /// For tie-breaking
        /// </summary>
        public int totalConflictsBetweenInternalAgents;

        /// <summary>
        /// For each agent, map each level (timestep) of its mdd to a narrowness degree.
        /// Non-narrow levels are omitted.
        /// </summary>
        public Dictionary<int, MDD.LevelNarrowness>[] mddNarrownessValues;

        public CbsNode(int numberOfAgents, ICbsSolver solver, ICbsSolver singleAgentSolver,
            CBS_LocalConflicts cbs, ushort[] agentsGroupAssignment = null)
        {
            this.cbs = cbs;
            allSingleAgentPlans = new SinglePlan[numberOfAgents];
            allSingleAgentCosts = new int[numberOfAgents];
            mddNarrownessValues = new Dictionary<int, MDD.LevelNarrowness>[numberOfAgents];
            countsOfInternalAgentsThatConflict = new int[numberOfAgents];
            conflictCountsPerAgent = new Dictionary<int, int>[numberOfAgents]; // Populated after Solve()
            conflictTimesPerAgent = new Dictionary<int, List<int>>[numberOfAgents]; // Populated after Solve()
            if (agentsGroupAssignment == null)
            {
                this.agentsGroupAssignment = new ushort[numberOfAgents];
                for (ushort i = 0; i < numberOfAgents; i++)
                    this.agentsGroupAssignment[i] = i;
            }
            else
                this.agentsGroupAssignment = agentsGroupAssignment.ToArray<ushort>();
            agentNumToIndex = new Dictionary<int, int>();
            for (int i = 0; i < numberOfAgents; i++)
            {
                agentNumToIndex[this.cbs.GetProblemInstance().m_vAgents[i].agent.agentNum] = i;
            }
            depth = 0;
            replanSize = 1;
            agentAExpansion = ExpansionState.NOT_EXPANDED;
            agentBExpansion = ExpansionState.NOT_EXPANDED;
            this.prev = null;
            this.constraint = null;
            this.solver = solver;
            this.singleAgentSolver = singleAgentSolver;
            this.minimumVertexCover = (int) ConflictGraph.MinVertexCover.NOT_SET;
        }

        /// <summary>
        /// Child from branch action constructor
        /// </summary>
        /// <param name="parent"></param>
        /// <param name="newConstraint"></param>
        /// <param name="agentToReplan"></param>
        public CbsNode(CbsNode parent, CbsConstraint newConstraint, int agentToReplan)
        {
            this.allSingleAgentPlans = parent.allSingleAgentPlans.ToArray();
            this.allSingleAgentCosts = parent.allSingleAgentCosts.ToArray();
            this.mddNarrownessValues = parent.mddNarrownessValues.ToArray();
            this.mddNarrownessValues[agentToReplan] = null;  // This agent has a new constraint, its old MDD isn't relevant anymore.

            this.countsOfInternalAgentsThatConflict = parent.countsOfInternalAgentsThatConflict.ToArray();
            this.conflictCountsPerAgent = new Dictionary<int, int>[parent.conflictCountsPerAgent.Length];
            for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
                this.conflictCountsPerAgent[i] = new Dictionary<int, int>(parent.conflictCountsPerAgent[i]); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
            this.conflictTimesPerAgent = new Dictionary<int, List<int>>[parent.conflictTimesPerAgent.Length];
            for (int i = 0; i < this.conflictTimesPerAgent.Length; i++)
            {
                this.conflictTimesPerAgent[i] = new Dictionary<int, List<int>>(); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
                foreach (var kvp in parent.conflictTimesPerAgent[i])
                    this.conflictTimesPerAgent[i][kvp.Key] = new List<int>(kvp.Value);
            }
            this.agentsGroupAssignment = parent.agentsGroupAssignment.ToArray();
            this.agentNumToIndex = parent.agentNumToIndex;
            this.prev = parent;
            this.constraint = newConstraint;
            this.depth = (ushort)(this.prev.depth + 1);
            this.agentAExpansion = ExpansionState.NOT_EXPANDED;
            this.agentBExpansion = ExpansionState.NOT_EXPANDED;
            this.replanSize = 1;
            this.solver = parent.solver;
            this.singleAgentSolver = parent.singleAgentSolver;
            this.cbs = parent.cbs;
            this.minimumVertexCover = (int) ConflictGraph.MinVertexCover.NOT_SET;
        }

        /// <summary>
        /// Child from merge action constructor. FIXME: Code dup with previous constructor.
        /// </summary>
        /// <param name="parent"></param>
        /// <param name="mergeGroupA"></param>
        /// <param name="mergeGroupB"></param>
        public CbsNode(CbsNode parent, int mergeGroupA, int mergeGroupB)
        {
            this.allSingleAgentPlans = parent.allSingleAgentPlans.ToArray();
            this.allSingleAgentCosts = parent.allSingleAgentCosts.ToArray();
            this.mddNarrownessValues = parent.mddNarrownessValues.ToArray();  // No new constraint was added so all of the parent's MDDs are valid
            this.countsOfInternalAgentsThatConflict = parent.countsOfInternalAgentsThatConflict.ToArray<int>();
            this.conflictCountsPerAgent = new Dictionary<int, int>[parent.conflictCountsPerAgent.Length];
            for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
                this.conflictCountsPerAgent[i] = new Dictionary<int, int>(parent.conflictCountsPerAgent[i]); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
            this.conflictTimesPerAgent = new Dictionary<int, List<int>>[parent.conflictTimesPerAgent.Length];
            for (int i = 0; i < this.conflictTimesPerAgent.Length; i++)
            {
                this.conflictTimesPerAgent[i] = new Dictionary<int, List<int>>(); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
                foreach (var kvp in parent.conflictTimesPerAgent[i])
                    this.conflictTimesPerAgent[i][kvp.Key] = new List<int>(kvp.Value);
            }
            this.agentsGroupAssignment = parent.agentsGroupAssignment.ToArray();
            this.agentNumToIndex = parent.agentNumToIndex;
            this.prev = parent;
            this.constraint = null;
            this.depth = (ushort)(this.prev.depth + 1);
            this.agentAExpansion = ExpansionState.NOT_EXPANDED;
            this.agentBExpansion = ExpansionState.NOT_EXPANDED;
            this.replanSize = 1;
            this.solver = parent.solver;
            this.singleAgentSolver = parent.singleAgentSolver;
            this.cbs = parent.cbs;
            
            this.MergeGroups(mergeGroupA, mergeGroupB);
            this.minimumVertexCover = (int) ConflictGraph.MinVertexCover.NOT_SET;
        }

        /// <summary>
        /// Total cost + heuristic estimate
        /// </summary>
        public int f
        {
            get { return this.g + this.h; }
        }

        public int GetTargetH(int f) => f - g;

        /// <summary>
        /// Solves the entire node - finds a plan for every agent group.
        /// Since this method is only called for the root of the constraint tree, every agent is in its own group.
        /// </summary>
        /// <param name="depthToReplan"></param>
        /// <returns>Whether solving was successful. Solving fails if a timeout occurs.</returns>
        public bool Solve(int depthToReplan)
        {
            this.g = 0;
            ProblemInstance problem = this.cbs.GetProblemInstance();
            var internalCAT = new ConflictAvoidanceTable();
            HashSet<CbsConstraint> newConstraints = this.GetConstraints(); // Probably empty as this is probably the root of the CT.
            var CAT = (Dictionary_U<TimedMove, int>)problem.parameters[CBS_LocalConflicts.CAT];
            var constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];

            HashSet_U<CbsConstraint> mustConstraints = null;
            HashSet<CbsConstraint> newMustConstraints = null;
            Dictionary<int,int> agentsWithMustConstraints = null;
            if (problem.parameters.ContainsKey(CBS_LocalConflicts.MUST_CONSTRAINTS))
            {
                mustConstraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS];
                newMustConstraints = this.GetMustConstraints();
                agentsWithMustConstraints = mustConstraints.Select<CbsConstraint, int>(constraint => constraint.agentNum).Distinct().ToDictionary<int, int>(x => x); // ToDictionary because there's no ToSet...
            }

            Dictionary<int, int> agentsWithConstraints = null;
            if (constraints.Count != 0)
            {
                int maxConstraintTimeStep = constraints.Max<CbsConstraint>(constraint => constraint.time);
                depthToReplan = Math.Max(depthToReplan, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
                agentsWithConstraints = constraints.Select<CbsConstraint, int>(constraint => constraint.agentNum).Distinct().ToDictionary<int, int>(x => x); // ToDictionary because there's no ToSet...
            }

            constraints.Join(newConstraints);
            CAT.Join(internalCAT);
            if (mustConstraints != null)
                mustConstraints.Join(newMustConstraints);
            // This mechanism of adding the constraints to the possibly pre-existing constraints allows having
            // layers of CBS solvers, each one adding its own constraints and respecting those of the solvers above it.

            // Find all the agents groups:
            var subGroups = new List<AgentState>[problem.m_vAgents.Length];
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (subGroups[i] == null)
                    subGroups[i] = new List<AgentState>();
                subGroups[this.agentsGroupAssignment[i]].Add(problem.m_vAgents[i]);
            }

            bool success = true;

            int maxPlanSize = 0;
            for (int i = 0; i < problem.m_vAgents.Length; i++)
            {
                if (this.agentsGroupAssignment[i] != i) // This isn't the first agent in its group - we've already solved its group.
                    continue;
                List<AgentState> subGroup = subGroups[i];

                bool agentGroupHasConstraints = (agentsWithConstraints != null) && subGroup.Any<AgentState>(state => agentsWithConstraints.ContainsKey(state.agent.agentNum));
                bool agentGroupHasMustConstraints = (agentsWithMustConstraints != null) && subGroup.Any<AgentState>(state => agentsWithMustConstraints.ContainsKey(state.agent.agentNum));

                // Solve for a single agent:
                if (agentGroupHasConstraints == false  &&
                    agentGroupHasMustConstraints == false &&
                    subGroup.Count == 1) // Top-most CBS with no must constraints on this agent. Shortcut available (that doesn't consider the CAT, though)
                {
                    allSingleAgentPlans[i] = new SinglePlan(problem.m_vAgents[i]); // All moves up to starting pos
                    allSingleAgentPlans[i].agentNum = problem.m_vAgents[this.agentsGroupAssignment[i]].agent.agentNum; // Use the group's representative
                    SinglePlan optimalPlan = problem.GetSingleAgentOptimalPlan(
                                    problem.m_vAgents[i],
                                    out this.conflictCountsPerAgent[i], out this.conflictTimesPerAgent[i]);
                    allSingleAgentPlans[i].ContinueWith(optimalPlan);
                    allSingleAgentCosts[i] = problem.m_vAgents[i].g + problem.GetSingleAgentOptimalCost(problem.m_vAgents[i]);
                    if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                    {
                        g += (ushort)allSingleAgentCosts[i];
                    }
                    else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                        Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                    {
                        g = Math.Max(g, (ushort)allSingleAgentCosts[i]);
                    }

                    this.UpdateAtGoalConflictCounts(i, maxPlanSize, CAT);
                }
                else
                {
                    success = this.Replan(i, depthToReplan, subGroup, maxPlanSize);

                    if (!success) // Usually means a timeout occured.
                        break;
                }

                // Add plan to the internalCAT
                foreach (AgentState agentState in subGroup)
                {
                    maxPlanSize = Math.Max(maxPlanSize, allSingleAgentPlans[this.agentNumToIndex[agentState.agent.agentNum]].GetSize());
                    internalCAT.AddPlan(allSingleAgentPlans[this.agentNumToIndex[agentState.agent.agentNum]]);
                }
            }

            CAT.Separate(internalCAT);
            constraints.Separate(newConstraints);
            if (mustConstraints != null)
                mustConstraints.Separate(newMustConstraints);

            if (!success)
                return false;

            // Update conflict counts: All agents but the last saw an incomplete CAT. Update counts backwards.
            for (int i = this.conflictCountsPerAgent.Length - 1; i >= 0; i--)
            {
                foreach (KeyValuePair<int, int> pair in this.conflictCountsPerAgent[i])
                {
                    if (this.agentNumToIndex.ContainsKey(pair.Key) && // An internal conflict, rather than external
                        this.agentNumToIndex[pair.Key] < i)                                 // Just an optimization. Would also be correct without this check.
                    {
                        this.conflictCountsPerAgent[this.agentNumToIndex[pair.Key]] // Yes, index here, num there
                            [problem.m_vAgents[i].agent.agentNum] = pair.Value; // Collisions are symmetrical, and agent "key" didn't see the route for agent "i" when planning.
                        this.conflictTimesPerAgent[this.agentNumToIndex[pair.Key]]
                            [problem.m_vAgents[i].agent.agentNum] = this.conflictTimesPerAgent[i][pair.Key];
                    }
                }
            }

            this.CountConflicts();

            this.CalcMinOpsToSolve();

            this.isGoal = this.countsOfInternalAgentsThatConflict.All(i => i == 0);

            return true;
        }

        /// <summary>
        /// Replan for a given agent (when constraints for that agent have changed).
        /// </summary>
        /// <param name="agentForReplan"></param>
        /// <param name="minPathTimeStep"></param>
        /// <param name="subGroup">If given, assume CAT is already populated and use this subGroup</param>
        /// <param name="maxPlanSizeOfOtherAgents">
        /// Internal optimization parameter indicating the max timestep to check for in-goal conflicts.
        /// If given, the value is used instead of computing it.
        /// </param>
        /// <param name="minPathCost"></param>
        /// <returns>Whether a path was successfully found</returns>
        public bool Replan(int agentForReplan, int minPathTimeStep, List<AgentState> subGroup = null,
                           int maxPlanSizeOfOtherAgents = -1, int minPathCost = -1)
        {
            ProblemInstance problem = this.cbs.GetProblemInstance();
            Dictionary_U<TimedMove, int> CAT = (Dictionary_U<TimedMove, int>)problem.parameters[CBS_LocalConflicts.CAT];

            ConflictAvoidanceTable internalCAT = null; // To quiet the compiler
            int groupNum = this.agentsGroupAssignment[agentForReplan];
            bool underSolve = true;

            if (subGroup == null)
            {
                underSolve = false;
                // Construct the subgroup of agents that are of the same group as agentForReplan,
                // and add the plans of all other agents to CAT
                internalCAT = new ConflictAvoidanceTable();
                subGroup = new List<AgentState>();
                maxPlanSizeOfOtherAgents = this.allSingleAgentPlans.Max(plan => plan.GetSize());
                for (int i = 0; i < agentsGroupAssignment.Length; i++)
                {
                    if (this.agentsGroupAssignment[i] == groupNum)
                        subGroup.Add(problem.m_vAgents[i]);
                    else
                        internalCAT.AddPlan(allSingleAgentPlans[i]);
                }
                
                CAT.Join(internalCAT);
            }
            HashSet<CbsConstraint> newConstraints = this.GetConstraints();
            var constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];

            HashSet_U<CbsConstraint> mustConstraints = null;
            HashSet<CbsConstraint> newMustConstraints = null;
            if (problem.parameters.ContainsKey(CBS_LocalConflicts.MUST_CONSTRAINTS))
            {
                mustConstraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS];
                newMustConstraints = this.GetMustConstraints();
            }
            
            this.replanSize = (ushort)subGroup.Count;

            ICbsSolver relevantSolver = this.solver;
            if (subGroup.Count == 1)
                relevantSolver = this.singleAgentSolver;

            ProblemInstance subProblem = problem.Subproblem(subGroup.ToArray());

            constraints.Join(newConstraints);
            if (mustConstraints != null)
                mustConstraints.Join(newMustConstraints);

            Dictionary<int, int> subGroupAgentNums = subGroup.Select(state => state.agent.agentNum).ToDictionary(num => num); // No need to call Distinct(). Each agent appears at most once

            IEnumerable<CbsConstraint> myConstraints = constraints.Where(constraint => subGroupAgentNums.ContainsKey(constraint.agentNum)); // TODO: Consider passing only myConstraints to the low level to speed things up.
            if (myConstraints.Count() != 0)
            {
                int maxConstraintTimeStep = myConstraints.Max(constraint => constraint.time);
                minPathTimeStep = Math.Max(minPathTimeStep, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
            }
            if (mustConstraints != null)
            {
                IEnumerable<CbsConstraint> myMustConstraints = mustConstraints.Where(constraint => subGroupAgentNums.ContainsKey(constraint.agentNum));
                if (myMustConstraints.Count() != 0)
                {
                    int maxMustConstraintTimeStep = myMustConstraints.Max(constraint => constraint.time);
                    minPathTimeStep = Math.Max(minPathTimeStep, maxMustConstraintTimeStep); // Give all must constraints a chance to affect the plan
                }
            }
           
            relevantSolver.Setup(subProblem, minPathTimeStep, this.cbs.runner, minPathCost);
            bool solved = relevantSolver.Solve();

            relevantSolver.AccumulateStatistics();
            relevantSolver.ClearStatistics();

            if (solved == false) // Usually means a timeout occured.
            {
                if (underSolve == false)
                    CAT.Separate(internalCAT); // Code dup, but if solved the CAT is needed for a bit longer.
                constraints.Separate(newConstraints);
                if (mustConstraints != null)
                    mustConstraints.Separate(newMustConstraints);
                return false;
            }

            // Copy the SinglePlans for the solved agent group from the solver to the appropriate places in this.allSingleAgentPlans
            SinglePlan[] singlePlans = relevantSolver.GetSinglePlans();
            int[] singleCosts = relevantSolver.GetSingleCosts();
            Dictionary<int, int> perAgent = relevantSolver.GetExternalConflictCounts();
            Dictionary<int, List<int>> conflictTimes = relevantSolver.GetConflictTimes();
            for (int i = 0; i < subGroup.Count; i++)
            {
                int agentNum = subGroup[i].agent.agentNum;
                int agentIndex = this.agentNumToIndex[agentNum];
                this.allSingleAgentPlans[agentIndex] = singlePlans[i];
                this.allSingleAgentPlans[agentIndex].agentNum = problem.m_vAgents[groupNum].agent.agentNum; // Use the group's representative
                this.allSingleAgentCosts[agentIndex] = singleCosts[i];
                if (i == 0) // This is the group representative
                {
                    this.conflictCountsPerAgent[agentIndex] = perAgent;
                    this.conflictTimesPerAgent[agentIndex] = conflictTimes;
                }
                else
                {
                    if (underSolve == false)
                    {
                        this.conflictCountsPerAgent[agentIndex].Clear(); // Don't over-count. Leave it to the group's representative.
                        this.conflictTimesPerAgent[agentIndex].Clear();
                    }
                    else
                    {
                        this.conflictCountsPerAgent[agentIndex] = new Dictionary<int, int>();
                        this.conflictTimesPerAgent[agentIndex] = new Dictionary<int, List<int>>();
                    }
                }
            }

            // Update conflict counts with what happens after the plan finishes
            foreach (var agentNumAndAgentNum in subGroupAgentNums)
            {
                int i = this.agentNumToIndex[agentNumAndAgentNum.Key];
                this.UpdateAtGoalConflictCounts(i, maxPlanSizeOfOtherAgents, CAT);
            }

            if (underSolve == false)
            {
                // Update conflictCountsPerAgent and conflictTimes for all agents
                int representativeAgentNum = subGroup[0].agent.agentNum;
                for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
                {
                    int agentNum = problem.m_vAgents[i].agent.agentNum;
                    if (perAgent.ContainsKey(agentNum))
                    {
                        this.conflictCountsPerAgent[i][representativeAgentNum] = perAgent[agentNum];
                        this.conflictTimesPerAgent[i][representativeAgentNum] = conflictTimes[agentNum];
                    }
                    else
                    {
                        this.conflictCountsPerAgent[i].Remove(representativeAgentNum);
                        this.conflictTimesPerAgent[i].Remove(representativeAgentNum);
                    }
                }

                this.CountConflicts();
                this.CalcMinOpsToSolve();
                CAT.Separate(internalCAT);
            }

            constraints.Separate(newConstraints);
            if (mustConstraints != null)
                mustConstraints.Separate(newMustConstraints);

            // Calc g
            if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
            {
                this.g = (ushort)Math.Max(this.allSingleAgentCosts.Sum(), this.g); // Conserve g from partial 
                                                                                   // expansion if it's higher
                                                                                   // (only happens when shuffling a partially expanded node)
            }
            else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
            {
                this.g = (ushort)Math.Max(this.allSingleAgentCosts.Max(), this.g); // Conserve g from partial
                                                                                   // expansion if it's higher
                                                                                   // (only happens when shuffling a partially expanded node)
            }

            this.isGoal = this.countsOfInternalAgentsThatConflict.All(i => i == 0);

            return true;
        }

        public void DebugPrint()
        {
            Debug.WriteLine("");
            Debug.WriteLine("");
            Debug.WriteLine($"Node hash: {this.GetHashCode()}");
            Debug.WriteLine($"Total cost so far: {this.g}");
            Debug.WriteLine($"h: {this.h}");
            Debug.WriteLine($"Min estimated ops needed: {this.minOpsToSolve}");
            Debug.WriteLine($"Expansion state: {this.agentAExpansion}, {this.agentBExpansion}");
            Debug.WriteLine($"Num of external agents that conflict: {totalExternalAgentsThatConflict}");
            Debug.WriteLine($"Num of internal agents that conflict: {totalInternalAgentsThatConflict}");
            Debug.WriteLine($"Num of conflicts between internal agents: {totalConflictsBetweenInternalAgents}");
            Debug.WriteLine($"Node depth: {this.depth}");
            if (this.prev != null)
                Debug.WriteLine($"Parent hash: {this.prev.GetHashCode()}");
            IList<CbsConstraint> constraints = this.GetConstraintsOrdered();
            Debug.WriteLine($"{constraints.Count} relevant internal constraints so far: ");
            foreach (CbsConstraint constraint in constraints)
            {
                Debug.WriteLine(constraint);
            }
            ISet<CbsConstraint> mustConstraints = this.GetMustConstraints(); // TODO: Ordered
            Debug.WriteLine($"{mustConstraints.Count} relevant internal must constraints so far: ");
            foreach (CbsConstraint mustConstraint in mustConstraints)
            {
                Debug.WriteLine(mustConstraint);
            }
            ProblemInstance problem = this.cbs.GetProblemInstance();
            var externalConstraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];
            Debug.WriteLine($"{externalConstraints.Count} external constraints: ");
            foreach (CbsConstraint constraint in externalConstraints)
            {
                Debug.WriteLine(constraint);
            }
            Debug.WriteLine($"Conflict: {this.GetConflict()}");
            Debug.Write("Agent group assignments: ");
            for (int j = 0; j < this.agentsGroupAssignment.Length; j++)
            {
                Debug.Write(" " + this.agentsGroupAssignment[j]);
            }
            Debug.WriteLine("");
            Debug.Write("Single agent costs: ");
            for (int j = 0; j < this.allSingleAgentCosts.Length; j++)
            {
                Debug.Write(" " + this.allSingleAgentCosts[j]);
            }
            Debug.WriteLine("");
            Debug.Write("Internal agents that conflict with each agent: ");
            for (int j = 0; j < this.countsOfInternalAgentsThatConflict.Length; j++)
            {
                Debug.Write($" {this.countsOfInternalAgentsThatConflict[j]}");
            }
            Debug.WriteLine("");
            for (int j = 0; j < this.conflictCountsPerAgent.Length; j++)
            {
                //if (this.conflictCountsPerAgent[j].Count != 0)
                {
                    Debug.Write($"Agent {problem.m_vAgents[j].agent.agentNum} conflict counts: ");
                    foreach (var pair in this.conflictCountsPerAgent[j])
                    {
                        Debug.Write($"{pair.Key}:{pair.Value} ");
                    }
                    Debug.WriteLine("");

                }
            }
            for (int j = 0; j < this.conflictTimesPerAgent.Length; j++)
            {
                //if (this.conflictCountsPerAgent[j].Count != 0)
                {
                    Debug.Write($"Agent {problem.m_vAgents[j].agent.agentNum} conflict times: ");
                    foreach (var pair in this.conflictTimesPerAgent[j])
                    {
                        Debug.Write($"{pair.Key}:[{String.Join(",", pair.Value)}], ");
                    }
                    Debug.WriteLine("");

                }
            }
            if (this.cbs.GetType() == typeof(CBS_GlobalConflicts) && this.cbs.mergeThreshold != -1)
            {
                for (int i = 0; i < ((CBS_GlobalConflicts)this.cbs).globalConflictsCounter.Length; i++)
                {
                    Debug.Write($"Agent {i} global historic conflict counts: ");
                    for (int j = 0; j < i; j++)
                    {
                        Debug.Write($"a{j}:{((CBS_GlobalConflicts)this.cbs).globalConflictsCounter[i][j]} ");
                    }
                    Debug.WriteLine("");
                }
            }
            var plan = this.CalculateJointPlan();
            if (plan.GetSize() < 200)
                Debug.WriteLine(plan);
            else
                Debug.WriteLine("Plan is too long to print");
            Debug.WriteLine("");
            Debug.WriteLine("");
        }

        /// <summary>
        /// Update conflict counts according to what happens after the plan finishes -
        /// needed if the plan is shorter than one of the previous plans and collides
        /// with it while at the goal.
        /// It's cheaper to do it this way than to force the solver the go more deeply.
        /// The conflict counts are saved at the group's representative.
        /// </summary>
        protected void UpdateAtGoalConflictCounts(int agentIndex, int maxPlanSize, IReadOnlyDictionary<TimedMove, List<int>> CAT)
        {
            ProblemInstance problem = this.cbs.GetProblemInstance();
            var afterGoal = new TimedMove(
                problem.m_vAgents[agentIndex].agent.Goal.x, problem.m_vAgents[agentIndex].agent.Goal.y, Move.Direction.Wait, 0);
            for (int time = allSingleAgentPlans[agentIndex].GetSize(); time < maxPlanSize; time++)
            {
                afterGoal.time = time;
                afterGoal.UpdateConflictCounts(CAT,
                                               this.conflictCountsPerAgent[this.agentsGroupAssignment[agentIndex]],
                                               this.conflictTimesPerAgent[this.agentsGroupAssignment[agentIndex]]);
            }
        }

        /// <summary>
        /// Calculates the minimum number of replans to solve, and from it the minimum number of replans or merges to solve.
        /// 
        /// A replan can resolve all of the agent's conflicts by luck, even if it was only targeting a single conflict.
        ///
        /// To calculate the minimum number of replans to solve, 
        /// what we want is the size of the minimum vertex cover of the conflict graph.
        /// Sadly, it's an NP-hard problem. Its decision variant is NP-complete.
        /// Happily, it has a 2-approximation: Just choose both endpoints of each uncovered edge repeatedly until no uncovered edges are lef.
        /// 
        /// So we can just take half the count from that approximation.
        /// 
        /// Notice a merge is like two replans in one, so we might need to take ceil(num_replans/2).
        /// Luckily, in Cbs_LocalConflicts, a merge is only possible once every B+1 depth steps,
        /// because we only count selected conflicts (they're guaranteed to be unequal),
        /// so we can cap the number of possible merges and substract less.
        /// 
        /// In Cbs_GlobalConflicts, we could use the global table to discount some merges.
        /// </summary>
        protected void CalcMinOpsToSolve()
        {
            if (this.cbs.disableTieBreakingByMinOpsEstimate == false)
            {
                var vertexCover = new HashSet<int>();

                for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
                {
                    if (vertexCover.Contains(i)) // This node is already in the cover - all its edges are already covered.
                        continue;

                    foreach (KeyValuePair<int, int> otherEndAgentNumAndCount in this.conflictCountsPerAgent[i])
                    {
                        if (this.agentNumToIndex.ContainsKey(otherEndAgentNumAndCount.Key)) // It's an internal conflict
                        {
                            int otherEndIndex = this.agentNumToIndex[otherEndAgentNumAndCount.Key];
                            if (vertexCover.Contains(otherEndAgentNumAndCount.Key) == false) // The vertex isn't covered from its other end yet
                            {
                                vertexCover.Add(i);
                                vertexCover.Add(otherEndIndex);
                                break; // All of this node's edges are now covered.
                            }
                        }
                    }
                }

                int minReplansToSolve = (int)Math.Ceiling(((double)vertexCover.Count) / 2); // We have a 2-approximation of the cover -
                // half that is at least half the value we're trying to approximate.
                //if (this.cbs.debug)
                //    Debug.WriteLine("min replans lower estimate: " + minReplansToSolve);
                if (this.cbs.mergeThreshold != -1) // Merges possible, account for them
                {
                    if (this.cbs.GetType() == typeof(CBS_LocalConflicts))
                    {
                        if (this.cbs.mergeThreshold > 0)
                        {
                            int maxPotentialMergeSavings = (int)Math.Floor(((double)minReplansToSolve) / 2);
                            int depthToGoTo = this.depth + minReplansToSolve;
                            int chainSize = this.cbs.mergeThreshold + 1; // Every series of B+1 downwards consecutive nodes may end with a merge.
                            int maxMerges = depthToGoTo / chainSize; // Round down to discount the last unfinished chain.

                            // Count the minimum amount of merges already done and substract it from maxMerges:
                            var groupSizes = new Dictionary<int, int>();
                            for (int i = 0; i < this.agentsGroupAssignment.Length; i++)
                            {
                                if (groupSizes.ContainsKey(this.agentsGroupAssignment[i]) == false)
                                    groupSizes[this.agentsGroupAssignment[i]] = 0;
                                groupSizes[this.agentsGroupAssignment[i]]++;
                            }
                            // Not using this.GetGroupSizes() because what we want is actually
                            // a list of the sizes of the different groups, not the size of each agent's group

                            foreach (int groupSize in groupSizes.Values)
                                maxMerges -= (int)Math.Ceiling(Math.Log(groupSize, 2)); // A group of size 1 has had zero merges, a group of size 2 has had 1, larger groups have had at least ceil(log2) their size merges.

                            int maxMergeSavings = Math.Min(maxPotentialMergeSavings, maxMerges);

                            this.minOpsToSolve = minReplansToSolve - maxMergeSavings;


                        }
                        else
                            this.minOpsToSolve = (int)Math.Ceiling(((double)minReplansToSolve) / 2);
                    }
                    else
                        this.minOpsToSolve = (int)Math.Ceiling(((double)minReplansToSolve) / 2); // TODO: We could look at the global table and maybe deduce something, but I'm not interested in that right now.
                }
                else
                    this.minOpsToSolve = (int)minReplansToSolve;
            }
        }

        /// <summary>
        /// Populates the totalInternalAgentsThatConflict, totalConflictsBetweenInternalAgents,
        /// totalConflictsWithExternalAgents, and countsOfInternalAgentsThatConflict counters
        /// from the conflictCountsPerAgent values that are created while solving or replanning.
        /// Those counters are used for tie-breaking.
        /// </summary>
        protected void CountConflicts()
        {
            var externalConflictingAgentNums = new HashSet<int>();
            this.totalInternalAgentsThatConflict = 0;
            this.totalConflictsBetweenInternalAgents = 0;
            this.totalConflictsWithExternalAgents = 0;

            for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
            {
                this.countsOfInternalAgentsThatConflict[i] = 0;

                if (conflictCountsPerAgent[i].Count != 0)
                    totalInternalAgentsThatConflict++;

                foreach (KeyValuePair<int, int> conflictingAgentNumAndCount in conflictCountsPerAgent[i])
                {
                    if (this.agentNumToIndex.ContainsKey(conflictingAgentNumAndCount.Key)) // It's an internal conflict
                    {
                        this.countsOfInternalAgentsThatConflict[i]++; // Counts one conflict for each agent the i'th agent conflicts with
                        this.totalConflictsBetweenInternalAgents += conflictingAgentNumAndCount.Value;
                    }
                    else
                    {
                        externalConflictingAgentNums.Add(conflictingAgentNumAndCount.Key);
                        this.totalConflictsWithExternalAgents += conflictingAgentNumAndCount.Value;
                        this.conflictTimesPerAgent[i].Remove(conflictingAgentNumAndCount.Key); // Not needed
                    }
                }
            }

            this.totalExternalAgentsThatConflict = externalConflictingAgentNums.Count;

            this.totalConflictsBetweenInternalAgents /= 2; // Each conflict was counted twice
            this.totalConflictsWithExternalAgents /= 2; // Each conflict was counted twice
        }

        /// <summary>
        /// Used to preserve state of conflict iteration.
        /// </summary>
        private IEnumerator<CbsConflict> nextConflicts;

        /// <summary>
        /// The iterator holds the state of the generator, with all the different queues etc - a lot of memory.
        /// We also clear the MDD narrowness values that were computed - if no child uses them, they'll be garbage-collected.
        /// </summary>
        public void ClearConflictChoiceData()
        {
            this.nextConflicts = null;
        }

        /// <summary>
        /// Use after expanding a node and finding the conflict wasn't cardinal
        /// </summary>
        /// <returns>Whether we found a new potentially cardinal conflict to work on</returns>
        public bool ChooseNextPotentiallyCardinalConflicts()
        {
            if (this.nextConflictCouldBeCardinal)
            {
                bool cycled = this.ChooseNextConflict();
                if (cycled)
                    return true;
                else
                    return false;
            }
            return false;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns>Whether another conflict was found</returns>
        public bool ChooseNextConflict()
        {
            bool hadNext = this.nextConflicts.MoveNext();
            if (hadNext)
                this.conflict = this.nextConflicts.Current;
            return hadNext;
        }

        /// <summary>
        /// Chooses an internal conflict to work on.
        /// Resets conflicts iteration if it's used.
        /// </summary>
        public void ChooseConflict()
        {
            if (this.allSingleAgentPlans.Length == 1) // A single internal agent can't conflict with anything internally
                return;

            if (this.isGoal) // Goal nodes don't have conflicts
                return;

            if (this.conflict != null) // Conflict already chosen before
                return;

            if (this.cbs.conflictChoice == CBS_LocalConflicts.ConflictChoice.FIRST)
            {
                this.ChooseFirstConflict();
            }
            else if (this.cbs.conflictChoice == CBS_LocalConflicts.ConflictChoice.MOST_CONFLICTING_SMALLEST_AGENTS)
            {
                this.ChooseConflictOfMostConflictingSmallestAgents();
            }
            else if (this.cbs.conflictChoice == CBS_LocalConflicts.ConflictChoice.CARDINAL_MDD)
            {
                // Choose the first (in order of looking at them), earliest (in time), cardinal
                // (if not found settle for semi-cardinal, then non-cardinal) conflict.
                // Assumes this.mergeThreshold == -1.
                this.nextConflicts = this.GetConflictsCardinalFirstUsingMdd().GetEnumerator();
                bool hasConflict = this.nextConflicts.MoveNext(); // This node isn't a goal node so this is expected to return true -
                                                                  // a conflict should be found
                if (hasConflict == false)
                {
                    this.DebugPrint();
                    Debug.Assert(false, "Non-goal node found no conflict");
                }
                this.conflict = this.nextConflicts.Current;
            }
            else if (this.cbs.conflictChoice == CBS_LocalConflicts.ConflictChoice.CARDINAL_LOOKAHEAD)
            {
                this.nextConflicts = this.GetConflictsNoOrder().GetEnumerator();
                bool hasConflict = this.nextConflicts.MoveNext(); // This node isn't a goal node so this is expected to return true -
                                                                  // a conflict should be found
                if (hasConflict == false)
                {
                    this.DebugPrint();
                    Debug.Assert(false, "Non-goal node found no conflict");
                }
                this.conflict = this.nextConflicts.Current;
                //FIXME: code dup with previous option
            }
            else
                throw new Exception("Unknown conflict choosing method");
        }

        private void ChooseConflictOfMostConflictingSmallestAgents()
        {
            (int groupRepA, int groupRepB, int time) = GetDetailsOfConflictOfMostConflictingSmallestAgents();
            this.conflict = FindConflict(groupRepA, groupRepB, time);
        }

        private void ChooseFirstConflict()
        {
            (int groupRepA, int groupRepB, int time) = GetFirstConflictDetails();
            this.conflict = FindConflict(groupRepA, groupRepB, time);
        }

        /// <summary>
        /// No special ordering.
        /// </summary>
        /// <returns></returns>
        private IEnumerable<CbsConflict> GetConflictsNoOrder()
        {
            ISet<int>[] groups = this.GetGroups();
            this.nextConflictCouldBeCardinal = true; // We don't know

            for (int agentIndex = 0; agentIndex < this.conflictTimesPerAgent.Length; agentIndex++)
            {
                foreach (int conflictingAgentNum in this.conflictTimesPerAgent[agentIndex].Keys)
                {
                    int conflictingAgentIndex = this.agentNumToIndex[conflictingAgentNum];
                    if (conflictingAgentIndex < agentIndex)
                        continue; // Return each conflict only once

                    foreach (int conflictTime in this.conflictTimesPerAgent[agentIndex][conflictingAgentNum])
                    {
                        yield return FindConflict(agentIndex, conflictingAgentIndex, conflictTime, groups);
                    }
                }
            }
        }

        /// <summary>
        /// Assumes this.mergeThreshold == -1.
        /// Builds MDDs for all agents.
        /// </summary>
        /// <returns></returns>
        private IEnumerable<CbsConflict> GetConflictsExhaustivelySearchingForCardinalsGreedily()
        {
            this.buildAllMDDs();
            return this.GetConflictsCardinalFirstUsingMdd();
        }

        public void ComputeHWithMVC()
        {
            if (this.conflict != null)
                return;  // TAKE CARE OF THAT TOO. The heuristic doesn't choose a conflict now.
            throw new NotImplementedException("CHOOSE THE CONFLICT TO WORK ON!");
        }
        
        /// <summary>
        /// Compute h value with a greedy maximal matching solver
        /// Assumes this.mergeThreshold == -1
        /// Also chooses the conflict to work on.
        /// </summary>
        /// <returns></returns>
        public void ComputeHWithMM()
        {
            if (this.conflict != null)
                return;
            uint MM = 0;
            bool[] agentsChosen = new bool[this.allSingleAgentPlans.Length];
            for (int i = 0; i < agentsChosen.Length; i++)
                agentsChosen[i] = false;
            ISet<int>[] groups = this.GetGroups();

            // Find a conflict by the way
            bool haveCardinalConflict = false;
            bool haveSemiCardinalConflict = false;
            int initialTimeStep = this.cbs.GetProblemInstance().m_vAgents[0].lastMove.time;
            foreach (var agentIndex in Enumerable.Range(0, this.allSingleAgentPlans.Length))
            {
                if (this.conflictTimesPerAgent[agentIndex].Count == 0)
                    continue;  // Agent has no conflicts
                this.buildMddForAgentWithItsCurrentCost(agentIndex);  // Does nothing if it's built already

                foreach (int conflictingAgentNum in this.conflictTimesPerAgent[agentIndex].Keys)
                {
                    int conflictingAgentIndex = this.agentNumToIndex[conflictingAgentNum];
                    if (conflictingAgentIndex < agentIndex) // check later
                        continue;
                    this.buildMddForAgentWithItsCurrentCost(conflictingAgentIndex);  // Does nothing if it's built already

                    foreach (int conflictTime in this.conflictTimesPerAgent[agentIndex][conflictingAgentNum])
                    {
                        bool iNarrow = this.DoesAgentHaveNoOtherOption(agentIndex, conflictTime, conflictingAgentIndex, groups);
                        bool jNarrow = this.DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, agentIndex, groups);
                        if (iNarrow && jNarrow) // Cardinal conflict
                        {
                            if (!agentsChosen[agentIndex] && !agentsChosen[conflictingAgentIndex])
                            {
                                MM++;
                                agentsChosen[agentIndex] = true;
                                agentsChosen[conflictingAgentIndex] = true;
                            }
                            if (!haveCardinalConflict) // this is the first cardinal conflict that we found
                            {
                                haveCardinalConflict = true;
                                this.conflict = FindConflict(agentIndex, conflictingAgentIndex, conflictTime);
                                this.conflict.mddPredictedCardinal = true;
                            }
                            else if (this.conflict.timeStep > conflictTime + initialTimeStep) // this cardinal conflict occurs earier than the former cardinal conflict
                            {
                                this.conflict = FindConflict(agentIndex, conflictingAgentIndex, conflictTime);
                                this.conflict.mddPredictedCardinal = true;
                            }
                         }
                        else if (iNarrow || jNarrow) // Semi-cardinal conflict
                        {
                            if (!haveCardinalConflict)
                            {
                                if (!haveSemiCardinalConflict) // this is the first semi-cardinal conflict that we found
                                {
                                    haveSemiCardinalConflict = true;
                                    this.conflict = FindConflict(agentIndex, conflictingAgentIndex, conflictTime);
                                }
                                else if (this.conflict.timeStep > conflictTime + initialTimeStep) // this semi-cardinal conflict occurs earier than the former semi-cardinal conflict
                                    this.conflict = FindConflict(agentIndex, conflictingAgentIndex, conflictTime);
                            }
                        }
                        else // Non-cardinal conflict
                        {
                            if (!haveCardinalConflict && !haveSemiCardinalConflict)
                            {
                                if (this.conflict == null || this.conflict.timeStep > conflictTime + initialTimeStep)
                                    this.conflict = FindConflict(agentIndex, conflictingAgentIndex, conflictTime);
                            }
                        }
                    }
                }
            }
            this.h = Math.Max(this.h, (ushort)MM);
        }

        public void buildAllMDDs()
        {
            foreach (var agentIndex in Enumerable.Range(0, this.allSingleAgentPlans.Length))
            {
                if (this.conflictTimesPerAgent[agentIndex].Count == 0)
                    continue;  // Agent has no conflicts
                this.buildMddForAgentWithItsCurrentCost(agentIndex);  // Does nothing if it's built already
            }
        }

        /// <summary>
        /// Assumes this.mergeThreshold == -1.
        /// Builds MDDs as necessary until a cardinal conflict is found.
        /// Also sets h to 1 if a cardinal conflict is found.
        /// </summary>
        /// <returns></returns>
        private IEnumerable<CbsConflict> GetConflictsExhaustivelySearchingForCardinalsLazily()
        {
            ISet<int>[] groups = this.GetGroups();

            foreach (var agentIndex in Enumerable.Range(0, this.allSingleAgentPlans.Length))
            {
                if (this.conflictTimesPerAgent[agentIndex].Count == 0)
                    continue;  // Agent has no conflicts
                this.buildMddForAgentWithItsCurrentCost(agentIndex);  // Does nothing if it's built already

                foreach (int conflictingAgentNum in this.conflictTimesPerAgent[agentIndex].Keys)
                {
                    int conflictingAgentIndex = this.agentNumToIndex[conflictingAgentNum];
                    this.buildMddForAgentWithItsCurrentCost(conflictingAgentIndex);  // Does nothing if it's built already

                    foreach (int conflictTime in this.conflictTimesPerAgent[agentIndex][conflictingAgentNum])
                    {
                        bool iNarrow = this.DoesAgentHaveNoOtherOption(agentIndex, conflictTime, conflictingAgentIndex, groups);
                        if (iNarrow)
                        {
                            bool otherNarrow = this.DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, agentIndex, groups);
                            if (otherNarrow)  // Both narrow!
                            {
                                CbsConflict cardinal = FindConflict(agentIndex, conflictingAgentIndex, conflictTime);
                                cardinal.mddPredictedCardinal = true;
                                // No need to set this.nextConflictCouldBeCardinal to true. Thie conflict is cardinal so the solver has not reason to cycle conflicts.
                                yield return cardinal;
                            }
                        }
                    }
                }
            }

            // No cardinal conflict was found
            var FullSearchIterator = this.GetConflictsCardinalFirstUsingMdd();
            foreach (var conflict in FullSearchIterator)
                yield return conflict;
        }

        /// <summary>
        /// CBS may use this to decide whether to give up the current conflict and check the next one.
        /// </summary>
        public bool nextConflictCouldBeCardinal = false;

        /// <summary>
        /// Returns all conflicts, cardinal first, then possibly cardinal, then semi cardinal,
        /// then possibly semi cardinal, and finally non-cardinal, building MDDs as necessary.
        /// Trying to build MDDs as late as possible.
        /// 
        /// TODO: Consider turning all queues into priority queues that prefer smaller agents, smaller degree etc.
        /// TODO: Find a better data structure to support faster deletion from queues.
        /// </summary>
        /// <returns></returns>
        private IEnumerable<CbsConflict> GetConflictsCardinalFirstUsingMdd()
        {
            if (this.totalConflictsBetweenInternalAgents == 1)
            {
                Debug.WriteLine("Single conflict. Just choosing it.");
                return GetConflictsNoOrder();
            }
            return GetConflictsCardinalFirstUsingMddInternal();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private IEnumerable<CbsConflict> GetConflictsCardinalFirstUsingMddInternal()
        {
            ISet<int>[] groups = this.GetGroups();
            // Queue items are <first agent index, second agent index, time>
            var NotCardinalMaybeSemi = new Queue<Tuple<int, int, int>>(); // Because first has an MDD
            var NotCardinalNotSemi = new Queue<Tuple<int, int, int>>();
            var SemiCardinal = new Queue<Tuple<int, int, int>>();
            var PossiblyCardinalFirstHasMddSecondDoesNotButCan = new Queue<Tuple<int, int, int>>();
            var PossiblyCardinalFirstHasMddSecondCannot = new Queue<Tuple<int, int, int>>();
            var PossiblyCardinalBothCannotBuildMdd = new Queue<Tuple<int, int, int>>();
            var PossiblyCardinalFirstCanBuildMdd = new Queue<Tuple<int, int, int>>(); // Going over these just get the first element, build its MDD and 
            var AgentIndexesWaitingToCheckTheirConflictsForCardinality = new Queue<int>(Enumerable.Range(0, this.allSingleAgentPlans.Length)); // Initially go over all conflicting agents.
                                                                                                                                               // TODO: this will also go over non-conflicting agents harmlessly. Is there an easy way to get a list of agents that have conflicts?
            // Positively cardinal conflicts are just yielded immediately
            // Conflicting agents are only entered into a queue once. Only if the conflicting agent with the larger index
            // can have an MDD built and the one with the lower can't, a pair of conflicting agents is entered in reverse.

            bool allowAgentOrderFlip = true; // Needed when rechecking agents to signal that we shouldn't 
                                             // rely on the other end to check a conflict

            // Incrementally scan conflicts and build MDDs
            while (true)
            {
                // 1. Go over AgentIndexesWaitingToCheckTheirConflictsForCardinality,
                // sorting conflicts into queues and yielding cardinal conflicts as necessary,
                // but not building new MDDs.
                while (AgentIndexesWaitingToCheckTheirConflictsForCardinality.Count != 0)  // Can't use foreach, we actually want to drain the queue
                {
                    var i = AgentIndexesWaitingToCheckTheirConflictsForCardinality.Dequeue();
                    bool hasMDD = this.mddNarrownessValues[i] != null;  // No need to check if its levels is null, we don't sync MDDs and we know there's a path with the current cost for the agent
                    bool canBuildMDD = groups[i].Count == 1;

                    foreach (int conflictingAgentNum in this.conflictTimesPerAgent[i].Keys)
                    {
                        int conflictingAgentIndex = this.agentNumToIndex[conflictingAgentNum];
                        bool otherHasMDD = this.mddNarrownessValues[conflictingAgentIndex] != null;
                        bool otherCanBuildMdd = groups[conflictingAgentIndex].Count == 1 && this.mddNarrownessValues[conflictingAgentIndex] == null;
                        if (allowAgentOrderFlip)
                        {
                            if ((conflictingAgentIndex < i) || (canBuildMDD == false && otherCanBuildMdd))
                                continue; // We'll take care of this conflict from the other end,
                                          // either because it's the first conflict sorting round
                                          // and we want to consider each conflict only once,
                                          // or because only the second agent can build an MDD and we
                                          // prefer the agent that can build an MDD to be the first one.
                        }

                        // Reaching here means either i < conflictingAgentIndex,
                        // or the i'th agent can build an MDD and the conflictingAgentIndex'th can't.
                        foreach (int conflictTime in this.conflictTimesPerAgent[i][conflictingAgentNum])
                        {
                            if (hasMDD) // Check if not cardinal
                            {
                                bool iNarrow = this.DoesAgentHaveNoOtherOption(i, conflictTime, conflictingAgentIndex, groups);
                                if (iNarrow == false) // Then it isn't cardinal. May still be semi cardinal.
                                {
                                    if (otherHasMDD == false) // Skip building the second MDD even if it's possible
                                    {
                                        NotCardinalMaybeSemi.Enqueue(new Tuple<int, int, int>(i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                    else // Other has MDD
                                    {
                                        bool otherNarrow = this.DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, i, groups);
                                        if (otherNarrow == false)
                                        {
                                            NotCardinalNotSemi.Enqueue(new Tuple<int, int, int>(i, conflictingAgentIndex, conflictTime));
                                            continue;
                                        }
                                        else // Other narrow but i not narrow at the time of the conflict
                                        {
                                            SemiCardinal.Enqueue(new Tuple<int, int, int>(i, conflictingAgentIndex, conflictTime));
                                            continue;
                                        }
                                    }
                                }
                                else // iNarrow
                                {
                                    if (otherHasMDD == false)
                                    {
                                        if (otherCanBuildMdd)
                                        {
                                            PossiblyCardinalFirstHasMddSecondDoesNotButCan.Enqueue(new Tuple<int, int, int>(i, conflictingAgentIndex, conflictTime));
                                            continue;
                                        }
                                        else // iNarrow, other cannot build an MDD
                                        {
                                            PossiblyCardinalFirstHasMddSecondCannot.Enqueue(new Tuple<int, int, int>(i, conflictingAgentIndex, conflictTime));
                                            continue;
                                        }
                                    }
                                    else // Other has MDD
                                    {
                                        bool otherNarrow = this.DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, i, groups);
                                        if (otherNarrow == false) // iNarrow but other not narrow
                                        {
                                            SemiCardinal.Enqueue(new Tuple<int, int, int>(i, conflictingAgentIndex, conflictTime));
                                            continue;
                                        }
                                        else // Both narrow!
                                        {
                                            Debug.WriteLine("Cardinal conflict chosen.");
                                            CbsConflict cardinal = FindConflict(i, conflictingAgentIndex, conflictTime, groups);
                                            cardinal.mddPredictedCardinal = true;
                                            this.h = Math.Max(this.h, (ushort) 1);  // The children's cost will be at least 1 more than this node's cost
                                            // No need to set this.nextConflictCouldBeCardinal to true. This conflict is cardinal so the solver has no reason to cycle conflicts.
                                            yield return cardinal;
                                            continue;
                                        }
                                    }
                                }
                            }
                            else // No MDD
                            {
                                if (canBuildMDD)
                                {
                                    PossiblyCardinalFirstCanBuildMdd.Enqueue(new Tuple<int, int, int>(i, conflictingAgentIndex, conflictTime));
                                    continue;
                                }
                                else // No MDD and can't build one and other can't build one either (already checked for the latter case above)
                                     // When re-checking an agent's conflicts we'll never get here because we only recheck agents that can build an MDD
                                {
                                    PossiblyCardinalBothCannotBuildMdd.Enqueue(new Tuple<int, int, int>(i, conflictingAgentIndex, conflictTime));
                                    continue;
                                }
                            }
                        }
                    }
                }

                allowAgentOrderFlip = false;  // We've flipped all we needed above
                
                // 2.
                if (PossiblyCardinalFirstHasMddSecondDoesNotButCan.Count != 0)
                {
                    //   a. Get one conflict from PossiblyCardinalFirstHasMddSecondDoesNotButCan and build the second agent's
                    // MDD.
                    int agentToBuildAnMddFor = PossiblyCardinalFirstHasMddSecondDoesNotButCan.Dequeue().Item2;
                    this.buildMddForAgentWithItsCurrentCost(agentToBuildAnMddFor);
                    //   b. Remove other conflicts from PossiblyCardinalFirstHasMddSecondDoesNotButCan where the second
                    //      agent is the one we built an MDD for (in all of those, the first agent's index is lower than the second's,
                    //      since we could build an MDD for it).
                    PossiblyCardinalFirstHasMddSecondDoesNotButCan = new Queue<Tuple<int,int,int>>(
                            PossiblyCardinalFirstHasMddSecondDoesNotButCan.Where<Tuple<int, int, int>>(
                                tuple => tuple.Item2 != agentToBuildAnMddFor));
                    //   c. Remove conflicts from PossiblyCardinalFirstCanBuildMdd where the first or second agent is the one we
                    //      built the MDD for.
                    PossiblyCardinalFirstCanBuildMdd = new Queue<Tuple<int, int, int>>(
                        PossiblyCardinalFirstCanBuildMdd.Where<Tuple<int, int, int>>(tuple => (tuple.Item2 != agentToBuildAnMddFor) &&
                                                                                              (tuple.Item1 != agentToBuildAnMddFor))
                                                                                       );
                    //   d. Remove conflicts from NotCardinalMaybeSemi where the second agent is the one we
                    //      built the MDD for
                    NotCardinalMaybeSemi = new Queue<Tuple<int, int, int>>(
                        NotCardinalMaybeSemi.Where<Tuple<int, int, int>>(tuple => tuple.Item2 != agentToBuildAnMddFor));
                    //   e. No need to check for the agent's conflicts in PossiblyCardinalFirstHasMddSecondCannot,
                    //      PossiblyCardinalBothCannotBuildMdd, NotCardinalNotSemi, SemiCardinal
                    //   f. Enter the agent into AgentIndexesWaitingToCheckTheirConflictsForCardinality. 
                    AgentIndexesWaitingToCheckTheirConflictsForCardinality.Enqueue(agentToBuildAnMddFor);
                    continue;
                }

                // 3.
                if (PossiblyCardinalFirstCanBuildMdd.Count != 0)
                {
                    //   a. Get one conflict from PossiblyCardinalFirstCanBuildMdd and build the first agent's
                    // MDD.
                    int agentToBuildAnMddFor = PossiblyCardinalFirstCanBuildMdd.Dequeue().Item1;
                    this.buildMddForAgentWithItsCurrentCost(agentToBuildAnMddFor);
                    //   b. Remove other conflicts from PossiblyCardinalFirstHasMddSecondDoesNotButCan where the second
                    //      agent is the one we built an MDD for (in all of those, the first agent's index is lower than the second's,
                    //      since we could build an MDD for it).
                    PossiblyCardinalFirstHasMddSecondDoesNotButCan = new Queue<Tuple<int, int, int>>(
                            PossiblyCardinalFirstHasMddSecondDoesNotButCan.Where<Tuple<int, int, int>>(
                                tuple => tuple.Item2 != agentToBuildAnMddFor));
                    //   c. Remove conflicts from PossiblyCardinalFirstCanBuildMdd where the first or second agent is the one we
                    //      built the MDD for.
                    PossiblyCardinalFirstCanBuildMdd = new Queue<Tuple<int, int, int>>(
                        PossiblyCardinalFirstCanBuildMdd.Where<Tuple<int, int, int>>(tuple => (tuple.Item2 != agentToBuildAnMddFor) &&
                                                                                              (tuple.Item1 != agentToBuildAnMddFor))
                                                                                       );
                    //   d. Remove conflicts from NotCardinalMaybeSemi where the second agent is the one we
                    //      built the MDD for
                    NotCardinalMaybeSemi = new Queue<Tuple<int, int, int>>(
                        NotCardinalMaybeSemi.Where<Tuple<int, int, int>>(tuple => tuple.Item2 != agentToBuildAnMddFor));
                    //   e. No need to check for the agent's conflicts in PossiblyCardinalFirstHasMddSecondCannot,
                    //      PossiblyCardinalBothCannotBuildMdd, NotCardinalNotSemi, SemiCardinal
                    //   f. Enter the agent into AgentIndexesWaitingToCheckTheirConflictsForCardinality. 
                    AgentIndexesWaitingToCheckTheirConflictsForCardinality.Enqueue(agentToBuildAnMddFor);
                    continue;
                }
                
                break; // No more queues to loot
            }

            // Yield the possibly cardinal conflicts where we can't build an MDD for the second agent
            while (PossiblyCardinalFirstHasMddSecondCannot.Count != 0)
            {
                Debug.WriteLine("We'll try and see if this conflict is cardinal");
                var tuple = PossiblyCardinalFirstHasMddSecondCannot.Dequeue();
                this.nextConflictCouldBeCardinal = (PossiblyCardinalFirstHasMddSecondCannot.Count != 0) ||
                                                   (PossiblyCardinalBothCannotBuildMdd.Count != 0);
                yield return FindConflict(tuple.Item1, tuple.Item2, tuple.Item3, groups);
            }

            // Yield the possibly cardinal conflicts where we can't build an MDD for either agent
            while (PossiblyCardinalBothCannotBuildMdd.Count != 0)
            {
                Debug.WriteLine("We'll try and see if this conflict is cardinal");
                var tuple = PossiblyCardinalBothCannotBuildMdd.Dequeue();
                this.nextConflictCouldBeCardinal = PossiblyCardinalBothCannotBuildMdd.Count != 0;
                yield return FindConflict(tuple.Item1, tuple.Item2, tuple.Item3, groups);
            }

            this.nextConflictCouldBeCardinal = false;

            // Yield semi cardinal conflicts
            while (SemiCardinal.Count != 0)
            {
                Debug.WriteLine("Settling for a semi-cardinal conflict.");
                var tuple = SemiCardinal.Dequeue();
                yield return FindConflict(tuple.Item1, tuple.Item2, tuple.Item3, groups);
            }

            // Yield the non cardinal conflicts, possibly semi first
            while (NotCardinalMaybeSemi.Count != 0)
            {
                Debug.WriteLine("No cardinal conflict found. This one's possibly a semi cardinal conflict.");
                var tuple = NotCardinalMaybeSemi.Dequeue();
                yield return FindConflict(tuple.Item1, tuple.Item2, tuple.Item3, groups);
            }

            while (NotCardinalNotSemi.Count != 0)
            {
                Debug.WriteLine("Non-cardinal conflict chosen");
                var tuple = NotCardinalNotSemi.Dequeue();
                yield return FindConflict(tuple.Item1, tuple.Item2, tuple.Item3, groups);
            }

            Debug.Assert(NotCardinalMaybeSemi.Count == 0);
            Debug.Assert(NotCardinalNotSemi.Count == 0);
            Debug.Assert(SemiCardinal.Count == 0);
            Debug.Assert(PossiblyCardinalFirstHasMddSecondDoesNotButCan.Count == 0);
            Debug.Assert(PossiblyCardinalFirstHasMddSecondCannot.Count == 0);
            Debug.Assert(PossiblyCardinalBothCannotBuildMdd.Count == 0);
            Debug.Assert(PossiblyCardinalFirstCanBuildMdd.Count == 0);
            Debug.Assert(AgentIndexesWaitingToCheckTheirConflictsForCardinality.Count == 0);
        }

        /// <summary>
        /// Assuming the groups conflict, return their conflict.
        /// </summary>
        /// <param name="aConflictingGroupMemberIndex"></param>
        /// <param name="bConflictingGroupMemberIndex"></param>
        /// <param name="time"></param>
        /// <param name="groups"></param>
        /// <returns></returns>
        private CbsConflict FindConflict(int aConflictingGroupMemberIndex,
            int bConflictingGroupMemberIndex, int time, ISet<int>[] groups = null)
        {
            int specificConflictingAgentA, specificConflictingAgentB;
            this.FindConflicting(aConflictingGroupMemberIndex, bConflictingGroupMemberIndex, time,
                                 out specificConflictingAgentA, out specificConflictingAgentB,
                                 groups);
            ProblemInstance problem = this.cbs.GetProblemInstance();
            int initialTimeStep = problem.m_vAgents[0].lastMove.time; // To account for solving partially solved problems.
            // This assumes the makespan of all the agents is the same.
            Move first = allSingleAgentPlans[specificConflictingAgentA].GetLocationAt(time);
            Move second = allSingleAgentPlans[specificConflictingAgentB].GetLocationAt(time);
            return new CbsConflict(specificConflictingAgentA, specificConflictingAgentB, first, second, time + initialTimeStep);
        }

        /// <summary>
        /// Assuming the groups conflict, find the specific agents that conflict.
        /// Also sets largerConflictingGroupSize.
        /// </summary>
        /// <param name="aConflictingGroupMemberIndex"></param>
        /// <param name="bConflictingGroupMemberIndex"></param>
        /// <param name="time"></param>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="groups"></param>
        /// <returns></returns>
        private void FindConflicting(int aConflictingGroupMemberIndex, int bConflictingGroupMemberIndex,
                                     int time, out int a, out int b,
                                     ISet<int>[] groups = null)
        {
            if (this.cbs.mergeThreshold == -1) // An optimization for CBS. We assume they collide.
            {
                a = aConflictingGroupMemberIndex;
                b = bConflictingGroupMemberIndex;
                return;
            }

            ISet<int> groupA;
            ISet<int> groupB;

            if (groups == null)
            {
                groupA = this.GetGroup(aConflictingGroupMemberIndex);
                groupB = this.GetGroup(bConflictingGroupMemberIndex);
            }
            else
            {
                groupA = groups[aConflictingGroupMemberIndex];
                groupB = groups[bConflictingGroupMemberIndex];
            }

            this.largerConflictingGroupSize = Math.Max(groupA.Count, groupB.Count);  // TODO: explain why

            if (groupA.Count == 1 && groupB.Count == 1) // We assume they collide.
            {
                a = aConflictingGroupMemberIndex;
                b = bConflictingGroupMemberIndex;
                return;
            }

            foreach (var varA in groupA)
            {
                foreach (var varB in groupB)
                {
                    if (allSingleAgentPlans[varA].IsColliding(time, allSingleAgentPlans[varB]))
                    {
                        a = varA;
                        b = varB;
                        return;
                    }
                }
            }

            // A conflict should have been found
            this.DebugPrint();
            throw new Exception("Conflict not found");
        }

        /// <summary>
        /// Builds an MDD for the specified agent with its current cost
        /// </summary>
        /// <param name="agentIndex"></param>
        /// <returns>Whether an MDD was built</returns>
        public bool buildMddForAgentWithItsCurrentCost(int agentIndex)
        {
            if (this.mddNarrownessValues[agentIndex] == null)
            {
                // Check if an ancestor has an appropriate one already.
                // If so, copy it down the CT branch to this node.
                CbsNode node = this;
                CbsNode ancestorWithAppropriateMdd = null;
                while (node != null)
                {
                    if (node.mddNarrownessValues[agentIndex] != null)
                    {
                        ancestorWithAppropriateMdd = node;
                        break;
                    }
                    if (node.constraint != null &&
                        this.agentNumToIndex[node.constraint.agentNum] == agentIndex)
                        break;  // This is where the last contraint on the agent was added.
                                // Ancestors will have inappropriate MDDs for this agent - no need to check them.
                    node = node.prev;
                }
                if (ancestorWithAppropriateMdd != null)
                {
                    node = this;
                    while (node != ancestorWithAppropriateMdd)
                    {
                        node.mddNarrownessValues[agentIndex] = ancestorWithAppropriateMdd.mddNarrownessValues[agentIndex];
                        node = node.prev;
                    }
                    return false;
                }

                // Build the MDD
                Debug.WriteLine($"Building MDD for agent index {agentIndex}");
                // TODO: Code dup with Replan, Solve
                ProblemInstance problem = this.cbs.GetProblemInstance();
                HashSet<CbsConstraint> newConstraints = this.GetConstraints();
                var constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];

                HashSet_U<CbsConstraint> mustConstraints = null;
                HashSet<CbsConstraint> newMustConstraints = null;
                if (problem.parameters.ContainsKey(CBS_LocalConflicts.MUST_CONSTRAINTS))
                {
                    mustConstraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS];
                    newMustConstraints = this.GetMustConstraints();
                }

                constraints.Join(newConstraints);
                if (mustConstraints != null)
                    mustConstraints.Join(newMustConstraints);

                int depth = this.allSingleAgentCosts.Max();

                IEnumerable<CbsConstraint> myConstraints = constraints.Where(
                    constraint => constraint.agentNum == problem.m_vAgents[agentIndex].agent.agentNum);
                if (myConstraints.Count() != 0)
                {
                    int maxConstraintTimeStep = myConstraints.Max(constraint => constraint.time);
                    depth = Math.Max(depth, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
                }
                if (mustConstraints != null)
                {
                    IEnumerable<CbsConstraint> myMustConstraints = mustConstraints.Where(
                        constraint => constraint.agentNum == problem.m_vAgents[agentIndex].agent.agentNum);
                    if (myMustConstraints.Count() != 0)
                    {
                        int maxMustConstraintTimeStep = myMustConstraints.Max(constraint => constraint.time);
                        depth = Math.Max(depth, maxMustConstraintTimeStep); // Give all must constraints a chance to affect the plan
                    }
                }

                var mdd = new MDD(agentIndex, problem.m_vAgents[agentIndex].agent.agentNum,
                    problem.m_vAgents[agentIndex].GetMove(), this.allSingleAgentCosts[agentIndex],
                    depth, problem.GetNumOfAgents(), problem,
                    ignoreConstraints: false, supportPruning: false);
                this.mddNarrownessValues[agentIndex] = mdd.getLevelNarrownessValues();
                constraints.Separate(newConstraints);
                if (mustConstraints != null)
                    mustConstraints.Separate(newMustConstraints);

                this.cbs.mddsBuilt++;

                // Copy the MDD up to ancestors where appropriate
                node = this;
                while (node != null)
                {
                    node.mddNarrownessValues[agentIndex] = this.mddNarrownessValues[agentIndex];
                    if (node.constraint != null &&
                        this.agentNumToIndex[node.constraint.agentNum] == agentIndex)
                        break;  // This is where the last contraint on the agent was added.
                                // Ancestors will have inappropriate MDDs for this agent - no need to check them.
                    node = node.prev;
                }

                return true;
            }
            else
                return false;
        }

        private (int groupRepA, int groupRepB, int time) GetFirstConflictDetails()
        {
            int groupRepA = -1; // To quiet the compiler
            int groupRepB = -1; // To quiet the compiler
            int time = int.MaxValue;
            for (int i = 0; i < this.conflictTimesPerAgent.Length; i++)
            {
                foreach (var otherAgentNumAndConflictTimes in this.conflictTimesPerAgent[i])
                {
                    if (otherAgentNumAndConflictTimes.Value[0] < time)
                    {
                        time = otherAgentNumAndConflictTimes.Value[0];
                        groupRepA = i;
                        groupRepB = this.agentNumToIndex[otherAgentNumAndConflictTimes.Key];
                    }
                }
            }
            return (groupRepA, groupRepB, time);
        }

        /// <summary>
        /// Chooses the first agent to be the one that maximizes the number of agents it conflicts with internally divided by 2^(group_size-1).
        /// Then chooses an agent among the agents it conflicts with using the same formula.
        /// Then chooses their first conflict.
        ///
        /// Choosing the agent that conflicts the most is a greedy strategy.
        /// Had replanning promised to resolve all conflicts, it would've been better to choose according to the minimum vertex cover.
        /// 
        /// Assumes all agents are initially on the same timestep (no OD).
        /// 
        /// TODO: Prefer conflicts where one of the conflicting agents is at their goal, to reduce the danger of task blow-up
        /// by enabling partial expansion. On the other hand, partial expansion is only possible in basic CBS.
        /// </summary>
        private (int groupRepA, int groupRepB, int time) GetDetailsOfConflictOfMostConflictingSmallestAgents()
        {
            int groupRepA = -1; // To quiet the compiler
            int groupRepB = -1; // To quiet the compiler
            int time = int.MaxValue;
            Func<int, double> formula = i => this.countsOfInternalAgentsThatConflict[i] / ((double)(1 << (this.GetGroupSize(i) - 1)));

            int chosenAgentIndex = Enumerable.Range(0, this.allSingleAgentPlans.Length).MaxByKeyFunc(formula);

            // We could just look for any of this agent's conflicts,
            // but the best choice among the agents it conflicts with is the one which maximizes the formula itself.
            IEnumerable<int> conflictsWithAgentNums = this.conflictCountsPerAgent[chosenAgentIndex].Keys;
            IEnumerable<int> conflictsWithInternallyAgentNums = conflictsWithAgentNums.Where(agentNum => this.agentNumToIndex.ContainsKey(agentNum));
            IEnumerable<int> conflictsWithInternallyAgentIndices = conflictsWithInternallyAgentNums.Select(agentNum => this.agentNumToIndex[agentNum]);
            int chosenConflictingAgentIndex = conflictsWithInternallyAgentIndices.MaxByKeyFunc(formula);

            groupRepA = chosenAgentIndex;
            groupRepB = chosenConflictingAgentIndex;

            ProblemInstance problem = this.cbs.GetProblemInstance();
            time = this.conflictTimesPerAgent[chosenAgentIndex] // Yes, the index of the first and the num of the second
                                                 [problem.m_vAgents[chosenConflictingAgentIndex].agent.agentNum][0];
            return (groupRepA, groupRepB, time);
        }

        public CbsConflict GetConflict()
        {
            return this.conflict;
        }

        /// <summary>
        /// Adopt everything but the new constraint, basically.
        /// 
        /// Notice that to correctly adopt a merge child, adopting its new agentsGroupAssignment is necessary.
        /// Otherwise its conflicts counts, SinglePlan.agentNum and conflict choice would be incompatible.
        /// </summary>
        /// <param name="child"></param>
        public void AdoptSolutionOf(CbsNode child)
        {
            Debug.Assert(this.g == child.g, "Tried to adopt node of a different cost");
            this.agentAExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
            this.agentBExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
            this.allSingleAgentCosts = child.allSingleAgentCosts;
            this.allSingleAgentPlans = child.allSingleAgentPlans;
            this.conflict = child.conflict;  // Probably null, see below
            this.isGoal = child.isGoal;
            this.countsOfInternalAgentsThatConflict = child.countsOfInternalAgentsThatConflict;
            this.conflictCountsPerAgent = child.conflictCountsPerAgent;
            this.conflictTimesPerAgent = child.conflictTimesPerAgent;
            this.totalExternalAgentsThatConflict = child.totalExternalAgentsThatConflict;
            this.minOpsToSolve = child.minOpsToSolve;
            this.totalInternalAgentsThatConflict = child.totalInternalAgentsThatConflict;
            this.totalConflictsWithExternalAgents = child.totalConflictsWithExternalAgents;
            this.totalConflictsBetweenInternalAgents = child.totalConflictsBetweenInternalAgents;
            this.largerConflictingGroupSize = child.largerConflictingGroupSize;
            // We don't adopt the child's constraints, nor its agents groups assignment
            // this.mdds is kept unchanged too since the cost of the replanned (meta-)agent
            // didn't change and we added no constraints.

            this.ChooseConflict(); // child probably hasn't chosen a conflict (and will never get a chance to),
                                   // need to choose the new conflict to work on.
                                   // (if child somehow had a conflict already, ChooseConflict does nothing)
                                   // We can't just continue the node's conflict iteration since
                                   // some conflicts may have been eliminated by the new plans
        }

        /// <summary>
        /// Uses the group assignments and the constraints.
        /// Irrelevant constraints stemming from conflicts between merged agents are ignored.
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int ans = 0;
                for (int i = 0; i < agentsGroupAssignment.Length; i++)
                {
                    ans += Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length] * agentsGroupAssignment[i];
                }

                HashSet<CbsConstraint> constraints = this.GetConstraints();

                // Add the hash codes for the contraints, ignoring their order
                foreach (CbsConstraint constraint in constraints)
                {
                    ans += constraint.GetHashCode();
                }

                return ans;
            }
        }

        /// <summary>
        /// Checks the group assignment and the constraints
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj) 
        {
            if (obj == null)
                return false;
            CbsNode other = (CbsNode)obj;

            if (this.agentsGroupAssignment.SequenceEqual(other.agentsGroupAssignment) == false)
                return false;

            CbsNode current = this;
            HashSet<CbsConstraint> other_constraints = other.GetConstraints();
            HashSet<CbsConstraint> constraints = this.GetConstraints();

            foreach (CbsConstraint constraint in constraints)
            {
                if (other_constraints.Contains(constraint) == false)
                    return false;
                current = current.prev;
            }
            // TODO: Consider replacing the above foreach with constraints.IsSubsetOf(other_constraints)

            return constraints.Count == other_constraints.Count;
        }

        /// <summary>
        /// Worth doing because the node may always be in the closed list
        /// </summary>
        public void Clear()
        {
            this.allSingleAgentPlans = null;
            this.allSingleAgentCosts = null;
            this.countsOfInternalAgentsThatConflict = null;
            this.conflictCountsPerAgent = null;
            this.conflictTimesPerAgent = null;
            this.agentNumToIndex = null;
        }

        public int CompareTo(IBinaryHeapItem item)
        {
            CbsNode other = (CbsNode)item;

            if (this.f < other.f)
                return -1;
            if (this.f > other.f)
                return 1;

            return this.CompareToIgnoreH(other);
        }

        public int CompareToIgnoreH(CbsNode other, bool ignorePartialExpansion = false)
        {
            // Tie breaking:

            // Prefer larger cost - higher h usually means more work needs to be done
            if (this.g > other.g)
                return -1;
            if (this.g < other.g)
                return 1;

            // Prefer less external conflicts, even over goal nodes, as goal nodes with less external conflicts are better.
            // External conflicts are also taken into account by the low level solver to prefer less conflicts between fewer agents.
            // This only helps when this CBS is used as a low level solver, of course.
            if (this.totalConflictsWithExternalAgents < other.totalConflictsWithExternalAgents)
                return -1;
            if (this.totalConflictsWithExternalAgents > other.totalConflictsWithExternalAgents)
                return 1;

            if (this.totalExternalAgentsThatConflict < other.totalExternalAgentsThatConflict)
                return -1;
            if (this.totalExternalAgentsThatConflict > other.totalExternalAgentsThatConflict)
                return 1;
            
            // Prefer goal nodes. The elaborate form is to keep the comparison consistent. Without it goalA<goalB and also goalB<goalA.
            if (this.GoalTest() == true && other.GoalTest() == false)
                return -1;
            if (other.GoalTest() == true && this.GoalTest() == false)
                return 1;

            // Prefer nodes which would possibly require less work.
            // Remember replans and merges don't necessarily enlarge the total cost, so the number of operations needed to solve
            // sadly can't be added to the node's total cost.
            if (this.cbs.disableTieBreakingByMinOpsEstimate == false)
            {
                if (this.minOpsToSolve < other.minOpsToSolve)
                    return -1;
                if (this.minOpsToSolve > other.minOpsToSolve)
                    return 1;
            }

            // Not preferring more depth because it makes no sense. It's not like preferring larger g,
            // which is smart because that part of the cost isn't an estimate.

            // Prefer less internal conflicts if the minOpsToSolve is the same
            if (this.totalConflictsBetweenInternalAgents < other.totalConflictsBetweenInternalAgents)
                return -1;
            if (this.totalConflictsBetweenInternalAgents > other.totalConflictsBetweenInternalAgents)
                return 1;

            if (this.totalInternalAgentsThatConflict < other.totalInternalAgentsThatConflict)
                return -1;
            if (this.totalInternalAgentsThatConflict > other.totalInternalAgentsThatConflict)
                return 1;

            if (ignorePartialExpansion == false)
            {
                // Prefer partially expanded nodes, in addition to their H bonus.
                // They're less work because they have less constraints and only one child to generate.
                // The elaborate form, again, is to keep the comparison consistent. Without it partiallyExpandedA<partiallyExpandedB and partiallyExpandedA>partiallyExpandedB
                if ((this.agentAExpansion == CbsNode.ExpansionState.DEFERRED || this.agentBExpansion == CbsNode.ExpansionState.DEFERRED) &&
                    other.agentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED && other.agentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED)
                    return -1;
                if ((other.agentAExpansion == CbsNode.ExpansionState.DEFERRED || other.agentBExpansion == CbsNode.ExpansionState.DEFERRED) &&
                    this.agentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED && this.agentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED)
                    return 1;
            }

            // Prefer nodes with conflicts between smaller groups of agents (irrelevant for goal nodes)
            // This requires that the conflict be chosen already, but we defer choosing the conflict to when
            // it comes *out* of OPEN, as choosing the conflict may be expensive.
            // TODO: enable this when both nodes have a chosen conflict. Nodes can be re-entered
            // into OPEN.
            //if (this.largerConflictingGroupSize < other.largerConflictingGroupSize)
            //    return -1;
            //if (this.largerConflictingGroupSize > other.largerConflictingGroupSize)
            //    return 1;

            return 0;
        }

        /// <summary>
        /// Not used.
        /// </summary>
        /// <returns></returns>
        public CbsConstraint GetLastConstraint()
        {
            return this.constraint;
        }

        public HashSet<CbsConstraint> GetConstraints()
        {
            var constraints = new HashSet<CbsConstraint>();
            CbsNode current = this;
            while (current.depth > 0) // The root has no constraints
            {
                if (current.constraint != null && // Next check not enough if "surprise merges" happen (merges taken from adopted child)
                    current.prev.conflict != null && // Can only happen for temporary lookahead nodes that were created and then later the parent adopted a goal node
                    this.agentsGroupAssignment[current.prev.conflict.agentAIndex] !=
                    this.agentsGroupAssignment[current.prev.conflict.agentBIndex]) // Ignore constraints that deal with conflicts between
                                                                                   // agents that were later merged. They're irrelevant
                                                                                   // since merging fixes all conflicts between merged agents.
                                                                                   // Nodes that only differ in such irrelevant conflicts will have the same single agent paths.
                                                                                   // Dereferencing current.prev is safe because current isn't the root.
                                                                                   // Also, merging creates a non-root node with a null constraint, and this helps avoid adding the null to the answer.
                    constraints.Add(current.constraint);
                current = current.prev;
            }
            return constraints;
        }

        /// <summary>
        /// For printing
        /// </summary>
        /// <returns></returns>
        public List<CbsConstraint> GetConstraintsOrdered()
        {
            var constraints = new List<CbsConstraint>();
            CbsNode current = this;
            while (current.depth > 0) // The root has no constraints
            {
                if (current.constraint != null && // Next check not enough if "surprise merges" happen (merges taken from adopted child)
                    current.prev.conflict != null && // Can only happen for temporary lookahead nodes the were created and then later the parent adopted a goal node
                    this.agentsGroupAssignment[current.prev.conflict.agentAIndex] !=
                    this.agentsGroupAssignment[current.prev.conflict.agentBIndex]) // Ignore constraints that deal with conflicts between
                    // agents that were later merged. They're irrelevant
                    // since merging fixes all conflicts between merged agents.
                    // Nodes that only differ in such irrelevant conflicts will have the same single agent paths.
                    // Dereferencing current.prev is safe because current isn't the root.
                    // Also, merging creates a non-root node with a null constraint, and this helps avoid adding the null to the answer.
                    constraints.Add(current.constraint);
                current = current.prev;
            }
            return constraints;
        }

        public HashSet<CbsConstraint> GetMustConstraints()
        {
            var constraints = new HashSet<CbsConstraint>();
            CbsNode current = this;
            while (current.depth > 0)
            {
                if (current.mustConstraint != null) // TODO: Ignore must constraint from merged agents
                    constraints.Add(current.mustConstraint);
                current = current.prev;
            }
            return constraints;
        }

        /// <summary>
        /// IBinaryHeapItem implementation
        /// </summary>
        /// <returns></returns>
        public int GetIndexInHeap() { return this.binaryHeapIndex; }

        /// <summary>
        /// IBinaryHeapItem implementation
        /// </summary>
        /// <returns></returns>
        public void SetIndexInHeap(int index) { this.binaryHeapIndex = index; }

        public Plan CalculateJointPlan()
        {
            return new Plan(allSingleAgentPlans);
        }

        /// <summary>
        /// Check if the agent groups that participate in the conflict of this node pass the merge threshold.
        /// </summary>
        /// <param name="mergeThreshold"></param>
        /// <returns>Whether to merge.</returns>
        public bool ShouldMerge(int mergeThreshold)
        {
            int conflictsCount = 1; // The agentA and agentB conflict in this node.
            ISet<int> firstGroup = this.GetGroup(this.conflict.agentAIndex);
            ISet<int> secondGroup = this.GetGroup(this.conflict.agentBIndex);

            CbsNode current = this.prev;
            int a, b;
            while (current != null)
            {
                a = current.conflict.agentAIndex;
                b = current.conflict.agentBIndex;
                if ((firstGroup.Contains(a) && secondGroup.Contains(b)) || (firstGroup.Contains(b) && secondGroup.Contains(a)))
                    conflictsCount++;
                current = current.prev;
            }

            return conflictsCount > mergeThreshold;
        }

        /// <summary>
        /// Check if the agent groups that participate in the conflict of this node pass the merge threshold,
        /// using the given conflict counts.
        /// </summary>
        /// <param name="mergeThreshold"></param>
        /// <param name="globalConflictCounter"></param>
        /// <returns>Whether to merge.</returns>
        public bool ShouldMerge(int mergeThreshold, int[][] globalConflictCounter)
        {
            int conflictCounter = 0;
            ISet<int> firstGroup = this.GetGroup(this.conflict.agentAIndex);
            ISet<int> secondGroup = this.GetGroup(this.conflict.agentBIndex);

            foreach (int a in firstGroup)
            {
                foreach (int b in secondGroup)
                {
                    conflictCounter += globalConflictCounter[Math.Max(a, b)][Math.Min(a, b)];
                }
            }

            return conflictCounter > mergeThreshold;
        }

        /// <summary>
        /// Returns a list of indices of agents in the group
        /// </summary>
        /// <param name="agentIndex"></param>
        /// <returns></returns>
        public ISet<int> GetGroup(int agentIndex)
        {
            int groupNumber = this.agentsGroupAssignment[agentIndex];
            ISet<int> group = new SortedSet<int>();

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == groupNumber)
                    group.Add(i);
            }
            return group;
        }

        /// <summary>
        /// Currently unused.
        /// </summary>
        /// <param name="groupNumber"></param>
        /// <returns></returns>
        public int GetGroupCost(int groupNumber)
        {
            int cost = 0;

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == groupNumber)
                    cost += this.allSingleAgentCosts[i];
            }
            return cost;
        }

        /// <summary>
        /// A bit cheaper than GetGroup(n).Count. Still O(n).
        /// </summary>
        /// <param name="agentIndex"></param>
        /// <returns></returns>
        public int GetGroupSize(int agentIndex)
        {
            int groupNumber = this.agentsGroupAssignment[agentIndex];
            int count = 0;

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == groupNumber)
                    count += 1;
            }
            return count;
        }

        /// <summary>
        /// In O(n)
        /// </summary>
        /// <returns></returns>
        public int[] GetGroupSizes()
        {
            int[] counts = new int[this.agentsGroupAssignment.Length];

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
                counts[this.agentsGroupAssignment[i]]++;

            int[] groupSizes = new int[this.agentsGroupAssignment.Length];

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
                groupSizes[i] = counts[this.agentsGroupAssignment[i]];
            
            return groupSizes;
        }

        /// <summary>
        /// In O(n)
        /// </summary>
        /// <returns></returns>
        public ISet<int>[] GetGroups()
        {
            Dictionary<int, ISet<int>> repsToGroups = new Dictionary<int, ISet<int>>();

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                int groupRep = this.agentsGroupAssignment[i];
                if (repsToGroups.ContainsKey(groupRep))
                    repsToGroups[groupRep].Add(i);
                else
                {
                    var newGroup = new HashSet<int>();
                    newGroup.Add(i);
                    repsToGroups[groupRep] = newGroup;

                }
            }

            ISet<int>[] res = new HashSet<int>[this.agentsGroupAssignment.Length];
            for (int i = 0; i < res.Length; i++)
                res[i] = repsToGroups[this.agentsGroupAssignment[i]];

            return res;
        }

        /// <summary>
        /// Updates the agentsGroupAssignment and the conflictCountsPerAgent. Warning: changes the hash!
        /// </summary>
        /// <param name="a">Index of first group representative</param>
        /// <param name="b">Index of second group representative</param>
        /// <param name="fixCounts"></param>
        public void MergeGroups(int a, int b, bool fixCounts = true)
        {
            if (b < a)
            {
                int c = a;
                a = b;
                b = c;
            }
            ProblemInstance problem = this.cbs.GetProblemInstance();
            int aAgentNum = problem.m_vAgents[a].agent.agentNum;
            int bAgentNum = problem.m_vAgents[b].agent.agentNum;

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == b)
                {
                    agentsGroupAssignment[i] = (ushort)a;
                }
            }

            if (fixCounts)
            {
                this.conflictCountsPerAgent[a].Remove(bAgentNum); // It isn't really necessary to update the conflictCountPerAgent dictionaries of the merged agents -
                // they're about to be replanned and have their dictionaries updated anyway
                this.conflictTimesPerAgent[a].Remove(bAgentNum);
                this.conflictCountsPerAgent[b].Clear();
                this.conflictTimesPerAgent[b].Clear();
                for (int i = 0; i < this.conflictCountsPerAgent.Length; i++)
                {
                    if (this.conflictCountsPerAgent[i].ContainsKey(bAgentNum))
                    {
                        if (this.conflictCountsPerAgent[i].ContainsKey(aAgentNum) == false)
                            this.conflictCountsPerAgent[i][aAgentNum] = 0;
                        this.conflictCountsPerAgent[i][aAgentNum] += this.conflictCountsPerAgent[i][bAgentNum];
                        this.conflictCountsPerAgent[i].Remove(bAgentNum);

                        if (this.conflictTimesPerAgent[i].ContainsKey(aAgentNum) == false)
                            this.conflictTimesPerAgent[i][aAgentNum] = new List<int>(this.conflictTimesPerAgent[i][bAgentNum].Count);
                        this.conflictTimesPerAgent[i][aAgentNum].AddRange(this.conflictTimesPerAgent[i][bAgentNum]);
                        this.conflictTimesPerAgent[i].Remove(bAgentNum);
                    }
                }
            }
        }

        public void PrintConflict()
        {
            if (conflict != null)
            {
                Debug.WriteLine("Conflict:");
                Debug.WriteLine("Agents:({0},{1})", conflict.agentAIndex, conflict.agentBIndex);
                Debug.WriteLine("Location:({0},{1})", conflict.agentAmove.x, conflict.agentAmove.y);
                Debug.WriteLine("Time:{0}", conflict.timeStep);
            }
            Debug.WriteLine("");
        }

        // TODO: Remove use of this method from other CBS's and delete it
        /// <summary>
        /// NOT the cost, just the length - 1.
        /// </summary>
        /// <param name="agent"></param>
        /// <returns></returns>
        public int PathLength(int agent)
        {
            List<Move> moves = allSingleAgentPlans[agent].locationAtTimes;
            Move goal = moves[moves.Count - 1];
            for (int i = moves.Count - 2; i >= 0; i--)
            {
                if (moves[i].Equals(goal) == false) // Note the move that gets to the goal is different to the move that first waits in it.
                    return  i + 1;
            }
            return 0;
        }

        public bool DoesMustConstraintAllow(CbsConstraint check)
        {
            CbsNode current = this;
            while (current != null)
            {
                if (current.mustConstraint != null && !current.mustConstraint.Allows(check))
                    return false;
                current = current.prev;
            }
            return true;
        }

        public void SetMustConstraint(CbsConstraint set)
        {
            this.mustConstraint = set;
        }

        private bool isGoal = false;

        public bool GoalTest() {
            if (this.g < this.cbs.minSolutionCost)
                return false;
            return isGoal;
        }

        /// <summary>
        /// For CBS IDA* only.
        /// TODO: Consider inheriting from CbsNode and overriding the Replan method instead.
        /// </summary>
        /// <param name="agentForReplan"></param>
        /// <param name="depthToReplan"></param>
        /// <param name="minPathCost"></param>
        /// <returns>Whether a path was successfully found</returns>
        public bool Replan3b(int agentForReplan, int depthToReplan, int minPathCost = -1)
        {
            var internalCAT = new ConflictAvoidanceTable();
            HashSet<CbsConstraint> newConstraints = this.GetConstraints();
            ProblemInstance problem = this.cbs.GetProblemInstance();
            var CAT = (Dictionary_U<TimedMove, int>)problem.parameters[CBS_LocalConflicts.CAT];
            var constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];
            var mustConstraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS];
            HashSet<CbsConstraint> newMustConstraints = this.GetMustConstraints();

            if (newConstraints.Count != 0)
            {
                int maxConstraintTimeStep = newConstraints.Max<CbsConstraint>(constraint => constraint.time);
                depthToReplan = Math.Max(depthToReplan, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
            }

            List<AgentState> subGroup = new List<AgentState>();
            int groupNum = this.agentsGroupAssignment[agentForReplan];
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                    subGroup.Add(problem.m_vAgents[i]);
                else
                    internalCAT.AddPlan(allSingleAgentPlans[i]);
            }

            this.replanSize = (ushort)subGroup.Count;

            ICbsSolver relevantSolver = this.solver;
            if (subGroup.Count == 1)
                relevantSolver = this.singleAgentSolver;

            ProblemInstance subProblem = problem.Subproblem(subGroup.ToArray());
            subProblem.parameters = problem.parameters;

            CAT.Join(internalCAT);
            constraints.Join(newConstraints);
            mustConstraints.Join(newMustConstraints);

            relevantSolver.Setup(subProblem, depthToReplan, this.cbs.runner, minPathCost);
            bool solved = relevantSolver.Solve();

            relevantSolver.AccumulateStatistics();
            relevantSolver.ClearStatistics();

            if (solved == false)
            {
                CAT.Separate(internalCAT);
                constraints.Separate(newConstraints);
                mustConstraints.Separate(newMustConstraints);
                return false;
            }

            int j = 0;
            SinglePlan[] singlePlans = relevantSolver.GetSinglePlans();
            int[] singleCosts = relevantSolver.GetSingleCosts();
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                {
                    this.allSingleAgentPlans[i] = singlePlans[j];
                    this.allSingleAgentPlans[i].agentNum = problem.m_vAgents[groupNum].agent.agentNum; // Use the group's representative
                    this.allSingleAgentCosts[i] = singleCosts[j];
                    j++;
                }
            }
            Debug.Assert(j == replanSize);

            // Calc g
            if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
            {
                this.g = (ushort)this.allSingleAgentCosts.Sum();
            }
            else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
            {
                this.g = (ushort)this.allSingleAgentCosts.Max();
            }

            // PrintPlan();

            CAT.Separate(internalCAT);
            constraints.Separate(newConstraints);
            mustConstraints.Separate(newMustConstraints);

            this.isGoal = this.countsOfInternalAgentsThatConflict.All(i => i == 0);
            //this.ChooseConflict(); 

            // PrintConflict();
            return true;
        }

        /// <summary>
        /// Assumes agents conflict at the given time, and an MDD has been built for the agent
        /// </summary>
        /// <param name="agentIndex"></param>
        /// <param name="conflictTime"></param>
        /// <param name="conflictingAgentIndex"></param>
        /// <param name="groups"></param>
        /// <returns></returns>
        public bool DoesAgentHaveNoOtherOption(int agentIndex, int conflictTime, int conflictingAgentIndex, ISet<int>[] groups)
        {
            bool stayingAtGoalConflict = conflictTime >= this.mddNarrownessValues[agentIndex].Keys.Last<int>();
            if (stayingAtGoalConflict)  // Then it must be a vertex conflict, and the agent can't have another option at same cost
                return true;
            else if (!this.mddNarrownessValues[agentIndex].ContainsKey(conflictTime))
                return false;
            else if (mddNarrownessValues[agentIndex][conflictTime] == MDD.LevelNarrowness.WIDTH_1)
                return true;
            else  // mddNarrownessValues[agentIndex][conflictTime] == ONE_LOCATION_MULTIPLE_DIRECTIONS
            {
                // If it's an edge conflict, and the MDD's width at the conflict time is 1,
                // this agent's cost will increase if split on this conflict - it's at least a semi-cardinal conflict.
                // If it's a vertex conflict, then even if the MDD's width at the conflict time is greater than 1,
                // it might still be a semi-cardinal or cardinal conflict if all of the agent's MDD's nodes at this
                // time move to the same vertex (from different directions)
                var conflict = FindConflict(agentIndex, conflictingAgentIndex, conflictTime, groups);
                bool vertexConflict = conflict.vertex;
                if (vertexConflict)
                    return true;
                else
                    return false;
            }
        }
    }

    /// <summary>
    /// Because the default tuple comparison compares the first element only :(.
    /// </summary>
    public class AgentToCheckForCardinalConflicts : IBinaryHeapItem
    {
        //public bool hasMDD;
        //int conflictingAgentsWithMDD;
        int groupSize;
        int degree;
        int planCost;
        public int index;

        public AgentToCheckForCardinalConflicts(/*bool hasMDD, int conflictingAgentsWithMDD,*/ int groupSize, int degree,
                                                int planCost, int index)
        {
            //this.hasMDD = hasMDD;
            //this.conflictingAgentsWithMDD = conflictingAgentsWithMDD;
            this.groupSize = groupSize;
            this.degree = degree;
            this.planCost = planCost;
            this.index = index;
        }

        public int CompareTo(IBinaryHeapItem item)
        {
            AgentToCheckForCardinalConflicts other = (AgentToCheckForCardinalConflicts)item;

            /*if (this.hasMDD && !other.hasMDD)
                return -1;
            else if (other.hasMDD && !this.hasMDD)
                return 1;

            if (this.conflictingAgentsWithMDD < other.conflictingAgentsWithMDD)
                return -1;
            else if (this.conflictingAgentsWithMDD > other.conflictingAgentsWithMDD)
                return 1;*/

            if (this.groupSize < other.groupSize)
                return -1;
            else if (this.groupSize > other.groupSize)
                return 1;

            if (this.degree < other.degree)
                return -1;
            else if (this.degree > other.degree)
                return 1;

            if (this.planCost < other.planCost)
                return -1;
            else if (this.planCost > other.planCost)
                return 1;

            if (this.index < other.index)
                return -1;
            else if (this.index > other.index)
                return 1;
            else
                return 0;
        }

        int binaryHeapIndex;

        /// <summary>
        /// IBinaryHeapItem implementation
        /// </summary>
        /// <returns></returns>
        public int GetIndexInHeap() { return binaryHeapIndex; }

        /// <summary>
        /// IBinaryHeapItem implementation
        /// </summary>
        /// <returns></returns>
        public void SetIndexInHeap(int index) { binaryHeapIndex = index; }
    }
}
