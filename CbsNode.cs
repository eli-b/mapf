using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;

namespace CPF_experiment
{
    public class CbsNode : IComparable<IBinaryHeapItem>, IBinaryHeapItem
    {
        public ushort totalCost;
        public ushort externalConflictsCount;
        public ushort internalConflictsCount;
        public SinglePlan[] allSingleAgentPlans;
        public int[] allSingleAgentCosts;
        private int binaryHeapIndex;
        CbsConflict conflict;
        CbsConstraint constraint;
        /// <summary>
        /// Forcing an agent to be at a certain place at a certain time
        /// </summary>
        CbsConstraint mustConstraint;
        CbsNode prev;
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
        protected ProblemInstance problem;
        protected ICbsSolver highLevelSolver;
        protected ICbsSolver lowLevelSolver;
        protected Run runner;

        public CbsNode(int numberOfAgents, ProblemInstance problem, ICbsSolver highLevelSolver, ICbsSolver lowLevelSolver, Run runner)
        {
            allSingleAgentPlans = new SinglePlan[numberOfAgents];
            allSingleAgentCosts = new int[numberOfAgents];
            depth = 0;
            replanSize = 1;
            externalConflictsCount = 0;
            internalConflictsCount = 0;
            agentAExpansion = ExpansionState.NOT_EXPANDED;
            agentBExpansion = ExpansionState.NOT_EXPANDED;
            agentsGroupAssignment = new ushort[numberOfAgents];
            for (ushort i = 0; i < numberOfAgents; i++)
            {
                agentsGroupAssignment[i] = i;
            }
            this.prev = null;
            this.constraint = null;
            this.problem = problem;
            this.highLevelSolver = highLevelSolver;
            this.lowLevelSolver = lowLevelSolver;
            this.runner = runner;
        }

        /// <summary>
        /// Child from branch action constructor
        /// </summary>
        /// <param name="father"></param>
        /// <param name="newConstraint"></param>
        /// <param name="agentToReplan"></param>
        public CbsNode(CbsNode father, CbsConstraint newConstraint, int agentToReplan)
        {
            this.allSingleAgentPlans = father.allSingleAgentPlans.ToArray<SinglePlan>();
            this.allSingleAgentCosts = father.allSingleAgentCosts.ToArray<int>();
            this.agentsGroupAssignment = father.agentsGroupAssignment.ToArray<ushort>();
            this.prev = father;
            this.constraint = newConstraint;
            this.depth = (ushort)(this.prev.depth + 1);
            externalConflictsCount = 0;
            internalConflictsCount = 0;
            agentAExpansion = ExpansionState.NOT_EXPANDED;
            agentBExpansion = ExpansionState.NOT_EXPANDED;
            replanSize = 1;
            this.problem = father.problem;
            this.highLevelSolver = father.highLevelSolver;
            this.lowLevelSolver = father.lowLevelSolver;
            this.runner = father.runner;
        }

        /// <summary>
        /// Child from merge action constructor
        /// </summary>
        /// <param name="father"></param>
        /// <param name="mergeGroupA"></param>
        /// <param name="mergeGroupB"></param>
        public CbsNode(CbsNode father, int mergeGroupA, int mergeGroupB)
        {
            this.allSingleAgentPlans = father.allSingleAgentPlans.ToArray<SinglePlan>();
            this.allSingleAgentCosts = father.allSingleAgentCosts.ToArray<int>();
            this.agentsGroupAssignment = father.agentsGroupAssignment.ToArray<ushort>();
            this.MergeGroups(mergeGroupA, mergeGroupB);
            this.prev = father;
            this.constraint = null;
            this.depth = (ushort)(this.prev.depth + 1);
            externalConflictsCount = 0;
            internalConflictsCount = 0;
            agentAExpansion = ExpansionState.NOT_EXPANDED;
            agentBExpansion = ExpansionState.NOT_EXPANDED;
            replanSize = 1;
            this.problem = father.problem;
            this.highLevelSolver = father.highLevelSolver;
            this.lowLevelSolver = father.lowLevelSolver;
            this.runner = father.runner;
        }

        /// <summary>
        /// Solve a given problem according to given constraints, sets the plans array (plan per agent).
        /// This method ignores the agentsGroupAssignment and solves for each agent separately using the low level solver, which is OK because it's only called for the root node.
        /// But on the other hand, it makes merging the method with Replan more difficult.
        /// Can this just call Replan consecutively please?
        /// </summary>
        /// <param name="depthToReplan"></param>
        /// <returns></returns>
        public bool Solve(int depthToReplan)
        {
            this.totalCost = 0;
            var newInternalCAT = new HashSet<TimedMove>();
            HashSet<CbsConstraint> newConstraints = this.GetConstraints();
            var internalCAT = (HashSet_U<TimedMove>)problem.parameters[CBS_LocalConflicts.INTERNAL_CAT];
            var constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];
            bool haveMustConstraints = problem.parameters.ContainsKey(CBS_LocalConflicts.MUST_CONSTRAINTS) == true &&
                                       ((List<CbsConstraint>)problem.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS]).Count > 0;
            Dictionary<int,int> agentsWithMustConstraints = null; // To quiet the compiler
            if (haveMustConstraints)
                agentsWithMustConstraints = ((List<CbsConstraint>)problem.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS]).Select<CbsConstraint, int>(constraint => constraint.agent).Distinct().ToDictionary<int,int>(x=>x); // ToDictionary because there's no ToSet...

            if (newConstraints.Count != 0)
            {
                int maxConstraintTimeStep = newConstraints.Max<CbsConstraint>(constraint => constraint.time);
                depthToReplan = Math.Max(depthToReplan, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
            }

            bool success = true;

            constraints.Join(newConstraints);

            for (int i = 0; i < problem.m_vAgents.Length; i++)
            {
                
                // This mechanism of adding the constraints to the possibly pre-existing constraints allows having
                // layers of CBS solver, each one adding its own constraints and respecting those of the solvers above it
                // in CbsLocalConflicts
                internalCAT.Join(newInternalCAT);

                if (constraints.Count == 0 && // TODO: Do a similar check to see if the constraints actually affect this agent
                    (haveMustConstraints == false ||
                     agentsWithMustConstraints.ContainsKey(i) == false)) // Top-most CBS with no must constraints on this agent. Shortcut available (ignoring the CAT thought)
                {
                    allSingleAgentPlans[i] = new SinglePlan(problem.m_vAgents[i]); // All moves up to starting pos
                    allSingleAgentPlans[i].ContinueWith(this.problem.GetSingleAgentOptimalPlan(problem.m_vAgents[i]));
                    allSingleAgentCosts[i] = problem.m_vAgents[i].g + this.problem.GetSingleAgentOptimalCost(problem.m_vAgents[i]);
                    totalCost += (ushort)allSingleAgentCosts[i];
                }
                else
                {
                    AgentState[] subGroup = new AgentState[] { problem.m_vAgents[i] };
                    ProblemInstance subProblem = problem.Subproblem(subGroup);
                
                    this.lowLevelSolver.Setup(subProblem, depthToReplan, runner);
                    success = this.lowLevelSolver.Solve();

                    this.lowLevelSolver.AccumulateStatistics();
                    this.lowLevelSolver.ClearStatistics();

                    if (!success) // Usually means a timeout occured.
                        break;

                    allSingleAgentPlans[i] = this.lowLevelSolver.GetSinglePlans()[0];
                    allSingleAgentCosts[i] = this.lowLevelSolver.GetSolutionCost();
                    totalCost += (ushort)allSingleAgentCosts[i];
                }

                allSingleAgentPlans[i].AddPlanToHashSet(newInternalCAT, totalCost * 2); // This is kind of an arbitrary value.
                // The next plan could be 100 times longer than the total cost so far,
                // and we won't detect the internal conflicts it causes by entering the other agents' goal long after they reached it.
                // TODO: Think of a way to mark the last step in the plan as a "forever after" step that applies to all the next steps.
                internalCAT.Seperate(newInternalCAT);
            }

            constraints.Seperate(newConstraints);

            if (success)
            {
                this.FindConflict();
                return true;
            }
            else
                return false;
        }

        /// <summary>
        /// Replan for a given agent (when constraints for that agent have changed).
        /// FIXME: Code duplication with Solve().
        /// </summary>
        /// <param name="agentForReplan"></param>
        /// <param name="depthToReplan"></param>
        /// <param name="highLevelExpanded"></param>
        /// <param name="highLevelGenerated"></param>
        /// <param name="lowLevelExpanded"></param>
        /// <param name="lowLevelGenerated"></param>
        /// <returns></returns>
        public bool Replan(int agentForReplan, int depthToReplan)
        {
            var newInternalCAT = new HashSet<TimedMove>();
            HashSet<CbsConstraint> newConstraints = this.GetConstraints();
            var internalCAT = (HashSet_U<TimedMove>)problem.parameters[CBS_LocalConflicts.INTERNAL_CAT];
            var constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];

            if (newConstraints.Count != 0)
            {
                int maxConstraintTimeStep = newConstraints.Max<CbsConstraint>(constraint => constraint.time);
                depthToReplan = Math.Max(depthToReplan, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
            }

            int maxPlanSize = this.allSingleAgentPlans.Max<SinglePlan>(plan => plan.GetSize());

            // Construct the subgroup of agents that are of the same group as agentForReplan,
            // and add the plans of all other agents to newInternalCAT
            List<AgentState> subGroup = new List<AgentState>();
            int groupNum = this.agentsGroupAssignment[agentForReplan];
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                {
                    subGroup.Add(problem.m_vAgents[i]);
                }
                else
                    allSingleAgentPlans[i].AddPlanToHashSet(newInternalCAT, maxPlanSize);
            }

            this.replanSize = (ushort)subGroup.Count;

            ICbsSolver solver = highLevelSolver;
            if (subGroup.Count == 1)
                solver = lowLevelSolver;

            ProblemInstance subProblem = problem.Subproblem(subGroup.ToArray());

            internalCAT.Join(newInternalCAT);
            constraints.Join(newConstraints);
           
            solver.Setup(subProblem, depthToReplan, runner);
            bool solved = solver.Solve();

            solver.AccumulateStatistics();
            solver.ClearStatistics();

            internalCAT.Seperate(newInternalCAT);
            constraints.Seperate(newConstraints);
            
            if (solved == false) // Usually means a timeout occured.
                return false;

            // Copy the SinglePlans for the solved agent group from the solver to the appropriate places in this.allSingleAgentPlans
            int j = 0;
            SinglePlan[] singlePlans = solver.GetSinglePlans();
            int[] singleCosts = solver.GetSingleCosts();
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                {
                    this.allSingleAgentPlans[i] = singlePlans[j];
                    this.allSingleAgentCosts[i] = singleCosts[j];
                    j++;
                }
            }
            Debug.Assert(j == replanSize);

            // Calc totalCost
            this.totalCost = (ushort) this.allSingleAgentCosts.Sum();

            // PrintPlan();

            this.FindConflict();
            // PrintConflict();
            return true;
        }

        /// <summary>
        /// Find the first conflict (timewise) for all the given plans.
        /// Assumes all agents are initially on the same timestep (no OD).
        /// Also updates the internalConflictsCount and externalConflictsCount counters.
        /// </summary>
        private void FindConflict()
        {
            if (this.allSingleAgentPlans.Length == 1) 
                return;
            int maxPlanSize = this.allSingleAgentPlans.Max<SinglePlan>(plan => plan.GetSize());
            this.conflict = null;
            HashSet<TimedMove> externalCAT = null;
            HashSet_U<TimedMove> CbsExternalCAT = (HashSet_U<TimedMove>)problem.parameters[CBS_LocalConflicts.INTERNAL_CAT];
            if (problem.parameters.ContainsKey(Trevor.CONFLICT_AVOIDANCE))
                 externalCAT = (HashSet<TimedMove>)problem.parameters[Trevor.CONFLICT_AVOIDANCE];
            TimedMove checkMove = new TimedMove();

            // Check in every time step that the plans do not collide
            for (int time = 1; time < maxPlanSize; time++)
            {
                // Check all pairs of groups if they are conflicting at the given time step
                for (int i = 0; i < allSingleAgentPlans.Length; i++)
                {
                    checkMove.setup(allSingleAgentPlans[i].GetLocationAt(time), time);
                    if (checkMove.IsColliding(externalCAT))
                        externalConflictsCount++;
                    if (checkMove.IsColliding(CbsExternalCAT))
                        externalConflictsCount++;
                    for (int j = i + 1; j < allSingleAgentPlans.Length; j++)
                    {
                        if (allSingleAgentPlans[i].IsColliding(time, allSingleAgentPlans[j]))
                        {
                            if (this.conflict == null)
                            {
                                int initialTimeStep = this.problem.m_vAgents[0].lastMove.time; // To account for solving partially solved problems.
                                                                                               // This assumes the makespan of all the agents is the same.
                                Move first = allSingleAgentPlans[i].GetLocationAt(time);
                                Move second = allSingleAgentPlans[j].GetLocationAt(time);
                                this.conflict = new CbsConflict(i, j, first, second, time + initialTimeStep);
                            }
                            internalConflictsCount++;
                        }
                    }
                }
            }
        }

        public CbsConflict GetConflict()
        {
            return this.conflict;
        }

        /// <summary>
        /// Uses the group assignments and the constraints.
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
            CbsNode other = (CbsNode)obj;

            if (this.agentsGroupAssignment.SequenceEqual<ushort>(other.agentsGroupAssignment) == false)
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
            return constraints.Count == other_constraints.Count;
        }

        /// <summary>
        /// Worth doing because the node may always be in the closed list
        /// </summary>
        public void Clear()
        {
            this.allSingleAgentPlans = null;
            this.allSingleAgentCosts = null;
        }

        public int CompareTo(IBinaryHeapItem item)
        {
            CbsNode other = (CbsNode)item;

            if (this.totalCost < other.totalCost)
                return -1;
            if (this.totalCost > other.totalCost)
                return 1;

            // Tie breaking:
            // Prefer less external conflicts, even over goal nodes, as goal nodes with less external conflicts are better.
            if (this.externalConflictsCount < other.externalConflictsCount)
                return -1;
            if (this.externalConflictsCount > other.externalConflictsCount)
                return 1;
            // Prefer goal nodes. The elaborate form is to keep the comparison consistent. Without it goalA<goalB and also goalB<goalA.
            if (this.GoalTest() == true && other.GoalTest() == false)
                return -1;
            if (other.GoalTest() == true && this.GoalTest() == false)
                return 1;

            // Not preferring more depth because it makes no sense. It's not like preferring larger g,
            // which is smart because that part of the cost isn't an estimate.

            // Prefer partially expanded nodes. They're less work because they have less constraints and only one child to generate.
            // The elaborate form, again, is to keep the comparison consistent. Without it partiallyExpandedA<partiallyExpandedB and partiallyExpandedA>partiallyExpandedB
            if ((this.agentAExpansion == CbsNode.ExpansionState.DEFERRED || this.agentBExpansion == CbsNode.ExpansionState.DEFERRED) &&
                other.agentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED && other.agentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED)
                return -1;
            if ((other.agentAExpansion == CbsNode.ExpansionState.DEFERRED || other.agentBExpansion == CbsNode.ExpansionState.DEFERRED) &&
                this.agentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED && this.agentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED)
                return 1;
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
                if (this.agentsGroupAssignment[current.prev.conflict.agentA] !=
                    this.agentsGroupAssignment[current.prev.conflict.agentB]) // Ignore constraints that deal with conflicts between
                                                                              // agents that were later merged. They're irrelevant
                                                                              // since merging fixes all conflicts between merged agents.
                                                                              // Nodes that only differ in such irrelevant conflicts will have the same single agent paths.
                                                                              // Dereferencing current.prev is safe because current isn't the root.
                    constraints.Add(current.constraint);
                current = current.prev;
            }
            return constraints;
        }

        public List<CbsConstraint> GetMustConstraints()
        {
            var constraints = new List<CbsConstraint>();
            CbsNode current = this;
            while (current.depth > 0)
            {
                if (current.mustConstraint != null)
                    constraints.Add(current.mustConstraint);
                current = current.prev;
            }
            constraints.Sort();
            return constraints;
        }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        /// <returns></returns>
        public int GetIndexInHeap() { return binaryHeapIndex; }

        /// <summary>
        /// BH_Item implementation
        /// </summary>
        /// <returns></returns>
        public void SetIndexInHeap(int index) { binaryHeapIndex = index; }

        public Plan CalculateJointPlan()
        {
            return new Plan(allSingleAgentPlans);
        }

        /// <summary>
        /// Merge agent groups that are conflicting in this node if they pass the merge threshold.
        /// </summary>
        /// <param name="mergeThreshold"></param>
        /// <returns>Whether a merge was performed.</returns>
        public bool ShouldMerge(int mergeThreshold)
        {
            int countConflicts = 1; // The agentA and agentB conflict in this node.
            ISet<int> firstGroup = this.GetGroup(this.agentsGroupAssignment[this.conflict.agentA]);
            ISet<int> secondGroup = this.GetGroup(this.agentsGroupAssignment[this.conflict.agentB]);

            CbsNode current = this.prev;
            int a, b;
            while (current != null)
            {
                a = current.conflict.agentA;
                b = current.conflict.agentB;
                if ((firstGroup.Contains(a) && secondGroup.Contains(b)) || (firstGroup.Contains(b) && secondGroup.Contains(a)))
                    countConflicts++;
                current = current.prev;
            }

            return countConflicts > mergeThreshold;
        }

        /// <summary>
        /// Merge agent groups that are conflicting in this node if they pass the merge threshold, using the given conflict counts.
        /// </summary>
        /// <param name="mergeThreshold"></param>
        /// <param name="globalConflictCounter"></param>
        /// <returns>Whether a merge was performed.</returns>
        public bool ShouldMerge(int mergeThreshold, int[][] globalConflictCounter)
        {
            int conflictCounter = 0;
            ISet<int> firstGroup = this.GetGroup(this.agentsGroupAssignment[this.conflict.agentA]);
            ISet<int> secondGroup = this.GetGroup(this.agentsGroupAssignment[this.conflict.agentB]);

            foreach (int a in firstGroup)
            {
                foreach (int b in secondGroup)
                {
                    conflictCounter += globalConflictCounter[Math.Max(a, b)][Math.Min(a, b)];
                }
            }

            return conflictCounter > mergeThreshold;
        }

        public ISet<int> GetGroup(int groupNumber)
        {
            ISet<int> group = new HashSet<int>();

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == groupNumber)
                    group.Add(i);
            }
            return group;
        }

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
        /// A bit cheaper than GetGroup(n).Count
        /// </summary>
        /// <param name="groupNumber"></param>
        /// <returns></returns>
        public int GetGroupSize(int groupNumber)
        {
            int count = 0;

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == groupNumber)
                    count += 1;
            }
            return count;
        }

        /// <summary>
        /// Warning: changes the hash!
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        private void MergeGroups(int a, int b)
        {
            if (b < a)
            {
                int c = a;
                a = b;
                b = c;
            }
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == b)
                {
                    agentsGroupAssignment[i] = (ushort)a;
                }
            }
        }

        public void PrintConflict()
        {
            if (conflict != null)
            {
                Debug.WriteLine("Conflict:");
                Debug.WriteLine("Agents:({0},{1})",conflict.agentA,conflict.agentB);
                Debug.WriteLine("Location:({0},{1})",conflict.agentAmove.x,conflict.agentAmove.y);
                Debug.WriteLine("Time:{0}",conflict.timeStep);
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

        public bool GoalTest() {
            return this.conflict == null;
        }

        /// <summary>
        /// For CBS IDA* only.
        /// Consider inheriting from CbsNode and overriding the Replan method instead.
        /// </summary>
        /// <param name="agentForReplan"></param>
        /// <param name="depthToReplan"></param>
        /// <returns></returns>
        public bool Replan3b(int agentForReplan, int depthToReplan)
        {
            var newInternalCAT = new HashSet<TimedMove>();
            HashSet<CbsConstraint> newConstraints = this.GetConstraints();
            HashSet_U<TimedMove> InternalCAT = (HashSet_U<TimedMove>)problem.parameters[CBS_LocalConflicts.INTERNAL_CAT];
            HashSet_U<CbsConstraint> Constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];
            List<CbsConstraint> mustConstraints = this.GetMustConstraints();
            problem.parameters[CBS_LocalConflicts.MUST_CONSTRAINTS] = mustConstraints;

            if (newConstraints.Count != 0)
            {
                int maxConstraintTimeStep = newConstraints.Max<CbsConstraint>(constraint => constraint.time);
                depthToReplan = Math.Max(depthToReplan, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
            }

            //Debug.WriteLine("Sub-problem:");

            List<AgentState> subGroup = new List<AgentState>();
            int groupNum = this.agentsGroupAssignment[agentForReplan];
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                {
                    subGroup.Add(problem.m_vAgents[i]);
                    // Debug.WriteLine(i);
                }
                else
                    allSingleAgentPlans[i].AddPlanToHashSet(newInternalCAT, totalCost);
            }

            this.replanSize = (ushort)subGroup.Count;

            ICbsSolver solver = highLevelSolver;
            if (subGroup.Count == 1)
                solver = lowLevelSolver;

            ProblemInstance subProblem = problem.Subproblem(subGroup.ToArray());
            subProblem.parameters = problem.parameters;

            InternalCAT.Join(newInternalCAT);
            Constraints.Join(newConstraints);

            //constraints.Print();

            solver.Setup(subProblem, depthToReplan, runner);
            bool solved = solver.Solve();

            solver.AccumulateStatistics();
            solver.ClearStatistics();

            if (solved == false)
            {
                InternalCAT.Seperate(newInternalCAT);
                Constraints.Seperate(newConstraints);
                return false;
            }

            int j = 0;
            SinglePlan[] singlePlans = solver.GetSinglePlans();
            int[] singleCosts = solver.GetSingleCosts();
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                {
                    this.allSingleAgentPlans[i] = singlePlans[j];
                    this.allSingleAgentCosts[i] = singleCosts[j];
                    j++;
                }
            }
            Debug.Assert(j == replanSize);

            // Calc totalCost
            this.totalCost = (ushort)this.allSingleAgentCosts.Sum();

            // PrintPlan();

            InternalCAT.Seperate(newInternalCAT);
            Constraints.Seperate(newConstraints);
            newConstraints.Clear();
            this.FindConflict();
            // PrintConflict();
            return true;
        }
    }
}
