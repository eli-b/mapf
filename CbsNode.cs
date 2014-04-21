using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;

namespace CPF_experiment
{
    class CbsNode : IComparable<IBinaryHeapItem>, IBinaryHeapItem
    {
        public ushort totalCost;
        public ushort externalConflictsCount;
        public ushort internalConflictsCount;
        public SinglePlan[] allSingleAgentPlans;
        private int binaryHeapIndex;
        CbsConflict conflict;
        CbsConstraint constraint;
        CbsConstraint unConstraint;
        CbsNode prev;
        public ushort depth;
        public ushort[] agentsGroupAssignment;
        public ushort replanSize;
        public enum ExpansionState: byte
        {
            NOT_EXPANDED = 0,
            A_NOT_EXPANDED,
            B_NOT_EXPANDED
        }
        /// <summary>
        /// For partial expansion
        /// </summary>
        public ExpansionState collapse;
        protected ProblemInstance problem;
        protected ICbsSolver highLevelSolver;
        protected ICbsSolver lowLevelSolver;
        protected Run runner;

        public CbsNode(int numberOfAgents, ProblemInstance problem, ICbsSolver highLevelSolver, ICbsSolver lowLevelSolver, Run runner)
        {
            allSingleAgentPlans = new SinglePlan[numberOfAgents];
            totalCost = 0;
            depth = 0;
            replanSize = 1;
            externalConflictsCount = 0;
            internalConflictsCount = 0;
            collapse = ExpansionState.NOT_EXPANDED;
            agentsGroupAssignment = new ushort[numberOfAgents];
            for (ushort i = 0; i < numberOfAgents; i++)
            {
                agentsGroupAssignment[i] = i;
            }
            this.problem = problem;
            this.highLevelSolver = highLevelSolver;
            this.lowLevelSolver = lowLevelSolver;
            this.runner = runner;
        }

        /// <summary>
        /// Child constructor
        /// </summary>
        /// <param name="father"></param>
        /// <param name="newConstraint"></param>
        /// <param name="agentToReplan"></param>
        /// <param name="problem"></param>
        /// <param name="runner"></param>
        public CbsNode(CbsNode father, CbsConstraint newConstraint, int agentToReplan)
        {
            this.totalCost = father.totalCost;
            this.allSingleAgentPlans = father.allSingleAgentPlans.ToArray<SinglePlan>();
            this.agentsGroupAssignment = father.agentsGroupAssignment.ToArray<ushort>();
            this.prev = father;
            this.constraint = newConstraint;
            // Add all other agents of the same group to the constraint:
            List<byte> group = new List<byte>();
            for (byte i = 0; i < agentsGroupAssignment.Length; i++) // FIXME: This assumes agent nums always simply start from zero and increment
            {
                if (i != agentToReplan && agentsGroupAssignment[i] == agentsGroupAssignment[agentToReplan])
                    group.Add(i);
            }
            this.constraint.AddAgents(group);
            this.depth = (ushort)(this.prev.depth + 1);
            externalConflictsCount = 0;
            internalConflictsCount = 0;
            collapse = ExpansionState.NOT_EXPANDED;
            replanSize = 1;
            this.problem = father.problem;
            this.highLevelSolver = father.highLevelSolver;
            this.lowLevelSolver = father.lowLevelSolver;
            this.runner = father.runner;
        }

        /// <summary>
        /// Solve a given problem according to given constraints, sets the plans array (plan per agent).
        /// Can this just call Replan consecutively please?
        /// </summary>
        /// <param name="depthToReplan"></param>
        /// <param name="highLevelExpanded"></param>
        /// <param name="highLevelGenerated"></param>
        /// <param name="lowLevelExpanded"></param>
        /// <param name="lowLevelGenerated"></param>
        /// <returns></returns>
        public bool Solve(int depthToReplan, ref int highLevelExpanded, ref int highLevelGenerated, ref int lowLevelExpanded, ref int lowLevelGenerated)
        {
            totalCost = 0;
            HashSet<TimedMove> newInternalCAT = (HashSet<TimedMove>)problem.parameters[CBS_LocalConflicts.NEW_INTERNAL_CAT];
            newInternalCAT.Clear();
            HashSet<CbsConstraint> newConstraints = (HashSet<CbsConstraint>)problem.parameters[CBS_LocalConflicts.NEW_CONSTRAINTS];
            newConstraints.Clear();
            HashSet_U<TimedMove> InternalCAT = (HashSet_U<TimedMove>)problem.parameters[CBS_LocalConflicts.INTERNAL_CAT];
            HashSet_U<CbsConstraint> Constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];

            for (int i = 0; i < problem.m_vAgents.Length; i++)
            {
                AgentState[] subGroup = new AgentState[] { problem.m_vAgents[i] };
                ProblemInstance subProblem = problem.Subproblem(subGroup);
                
                InternalCAT.Join(newInternalCAT);
                Constraints.Join(newConstraints);

                this.lowLevelSolver.Setup(subProblem, depthToReplan, runner);
                bool success = this.lowLevelSolver.Solve();

                highLevelExpanded += this.lowLevelSolver.GetHighLevelExpanded();
                highLevelGenerated += this.lowLevelSolver.GetHighLevelGenerated();
                lowLevelExpanded += this.lowLevelSolver.GetLowLevelExpanded();
                lowLevelGenerated += this.lowLevelSolver.GetLowLevelGenerated();
                if (!success)
                {

                    InternalCAT.Seperate(newInternalCAT);
                    Constraints.Seperate(newConstraints);
                    return false;
                }

                InternalCAT.Seperate(newInternalCAT);
                Constraints.Seperate(newConstraints);

                allSingleAgentPlans[i] = this.lowLevelSolver.GetSinglePlans()[0];
                totalCost += (ushort)this.lowLevelSolver.GetSolutionCost();
                allSingleAgentPlans[i].addPlanToHashSet(newInternalCAT, totalCost * 2);
            }
            this.FindConflict(problem);
            return true;
        }

        /// <summary>
        /// Replan for a given agent (when constraints for that agent have changed).
        /// FIXME: Code duplication with Solve().
        /// </summary>
        /// <param name="agentForRePlan"></param>
        /// <param name="depthToReplan"></param>
        /// <param name="highLevelExpanded"></param>
        /// <param name="highLevelGenerated"></param>
        /// <param name="lowLevelExpanded"></param>
        /// <param name="lowLevelGenerated"></param>
        /// <returns></returns>
        public bool Replan(int agentForRePlan, int depthToReplan, ref int highLevelExpanded, ref int highLevelGenerated, ref int lowLevelExpanded, ref int lowLevelGenerated)
        {
            HashSet<TimedMove> newInternalCAT = (HashSet<TimedMove>)problem.parameters[CBS_LocalConflicts.NEW_INTERNAL_CAT];
            HashSet<CbsConstraint> newConstraints = this.GetConstraints();
            problem.parameters[CBS_LocalConflicts.NEW_CONSTRAINTS] = newConstraints;
            HashSet_U<TimedMove> InternalCAT = (HashSet_U<TimedMove>)problem.parameters[CBS_LocalConflicts.INTERNAL_CAT];
            HashSet_U<CbsConstraint> Constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];
            ICbsSolver solver = highLevelSolver;

            newInternalCAT.Clear();

            int groupNum = this.agentsGroupAssignment[agentForRePlan];
            List<AgentState> subGroup=new List<AgentState>();

            //Debug.WriteLine("Sub-problem:");

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                {
                    subGroup.Add(problem.m_vAgents[i]);
                   // Debug.WriteLine(i);
                }
                else
                    allSingleAgentPlans[i].addPlanToHashSet(newInternalCAT, totalCost);
            }

            replanSize = (ushort)subGroup.Count;

            if (subGroup.Count == 1)
                solver = lowLevelSolver;

            ProblemInstance subProblem = problem.Subproblem(subGroup.ToArray());
            subProblem.parameters = problem.parameters;

            InternalCAT.Join(newInternalCAT);
            Constraints.Join(newConstraints);

            //Constraints.print();
           
            solver.Setup(subProblem, depthToReplan, runner);
            if (solver.Solve() == false)
            {
                highLevelExpanded += solver.GetHighLevelExpanded();
                highLevelGenerated += solver.GetHighLevelGenerated();
                lowLevelExpanded += solver.GetLowLevelExpanded();
                lowLevelGenerated += solver.GetLowLevelGenerated();
                InternalCAT.Seperate(newInternalCAT);
                Constraints.Seperate(newConstraints);
                return false;
            }
            highLevelExpanded += solver.GetHighLevelExpanded();
            highLevelGenerated += solver.GetHighLevelGenerated();
            lowLevelExpanded += solver.GetLowLevelExpanded();
            lowLevelGenerated += solver.GetLowLevelGenerated();

            int j = 0;
            SinglePlan[] singlePlans = solver.GetSinglePlans();

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                {
                    this.allSingleAgentPlans[i] = singlePlans[j];
                    j++;
                }

            }

            // PrintPlan();

            totalCost = 0;
            foreach (SinglePlan plan in allSingleAgentPlans)
            {
                totalCost += (ushort)(plan.GetSize() - 1);
            }
            InternalCAT.Seperate(newInternalCAT);
            Constraints.Seperate(newConstraints);
            newConstraints.Clear();
            this.FindConflict(problem);
            // PrintConflict();
            return true;
        }

        /// <summary>
        /// Find the first conflict (timewise) for all the given plans
        /// </summary>
        private void FindConflict(ProblemInstance problem)
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
                    if (checkMove.isColliding(externalCAT))
                        externalConflictsCount++;
                    if (checkMove.isColliding(CbsExternalCAT))
                        externalConflictsCount++;
                    for (int j = i + 1; j < allSingleAgentPlans.Length; j++)
                    {
                        if (allSingleAgentPlans[i].IsColliding(time, allSingleAgentPlans[j]))
                        {
                            if (conflict == null)
                            {
                                Move first = allSingleAgentPlans[i].GetLocationAt(time);
                                Move second = allSingleAgentPlans[j].GetLocationAt(time);
                                this.conflict = new CbsConflict(i, j, first, second, time);
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
        public override int GetHashCode() //TODO: change this
        {
            unchecked
            {
                int ans = 0;
                for (int i = 0; i < agentsGroupAssignment.Length; i++)
                {
                    ans += Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length] * agentsGroupAssignment[i];
                }

                CbsNode current = this;
                while (current.depth > 0)
                {
                    if (current.prev.conflict != null && 
                        this.agentsGroupAssignment[current.prev.conflict.agentA] != this.agentsGroupAssignment[current.prev.conflict.agentB]) // What does this check do? Is it correct?
                    {
                        ans += current.constraint.GetHashCode();
                    }
                    current = current.prev;
                }
                return ans;
            }
        }

        /// <summary>
        /// Not used anywhere.
        /// </summary>
        private void NormalizeGroups()
        {
            short[] newGroups = new short[agentsGroupAssignment.Length];
            short groupCount = 0;

            for (int i = 0; i < newGroups.Length; i++)
            {
                newGroups[i] = -1;
            }

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (newGroups[agentsGroupAssignment[i]] == -1)
                {
                    newGroups[agentsGroupAssignment[i]] = groupCount;
                    groupCount++;
                }
            }
            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                agentsGroupAssignment[i] = (ushort)newGroups[agentsGroupAssignment[i]];
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
            CbsConstraint.fullyEqual = true;
            HashSet<CbsConstraint> other_constraints = other.GetConstraints();

            int constraint_count = 0;
            while (current.depth > 0)
            {
                ++constraint_count;
                //if (this.agentsGroupAssignment[current.prev.conflict.agentAMove] != this.agentsGroupAssignment[current.prev.conflict.agentBMove])
                //{
                    //current.constraint.group = (byte)this.agentsGroupAssignment[current.constraint.getAgentNum()];
                    if (other_constraints.Contains(current.constraint) == false)
                    {
                        //if (OtherAgentsGroupAssignment != null)
                        //    other.agentsGroupAssignment = OtherAgentsGroupAssignment;
                        CbsConstraint.fullyEqual = false;
                        return false;
                    }
                //}
                current = current.prev;
            }
            //if (OtherAgentsGroupAssignment != null)
            //    other.agentsGroupAssignment = OtherAgentsGroupAssignment;
            CbsConstraint.fullyEqual = false;
            return constraint_count == other_constraints.Count;
        }

        public void Clear()
        {
            this.allSingleAgentPlans = null;
        }

        public int CompareTo(IBinaryHeapItem item)
        {
            CbsNode other = (CbsNode)item;

            if (this.totalCost < other.totalCost)
                return -1;
            if (this.totalCost > other.totalCost)
                return 1;

            // Tie breaking:
            if (this.externalConflictsCount < other.externalConflictsCount)
                return -1;
            if (this.externalConflictsCount > other.externalConflictsCount)
                return 1;
            if (this.conflict == null)
                return -1;
            if (other.conflict == null)
                return 1;
            if (this.depth > other.depth)
                return -1;
            if (this.depth < other.depth)
                return 1;
            return 0;
        }

        public CbsConstraint GetLastConstraint()
        {
            return this.constraint;
        }

        public HashSet<CbsConstraint> GetConstraints()
        {
            var constraints = new HashSet<CbsConstraint>();
            CbsNode current = this;
            while (current.depth > 0)
            {
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
                if (current.unConstraint != null)
                    constraints.Add(current.unConstraint);
                current = current.prev;
            }
            constraints.Sort();
            return constraints;
        }

        //private HashSet<CbsConstraint> GetConstraints(int group, ProblemInstance problem)
        //{
        //    var constraints = new HashSet<CbsConstraint>()
        //    CbsNode current = this;
        //    while (current.depth > 0)
        //    {
        //        if (this.agentsGroupAssignment[current.constraint.getAgentNum()] == group)
        //        {
        //            if (this.agentsGroupAssignment[current.prev.conflict.agentAMove] != this.agentsGroupAssignment[current.prev.conflict.agentBMove]) 
        //                constraints.Add(current.constraint);
        //        }
        //        current = current.prev;
        //    }
        //    return constraints;
        //}

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
            IList<SinglePlan> plans = new List<SinglePlan>();

            for (int i = 0; i < allSingleAgentPlans.Length; i++)
            {
                plans.Add(allSingleAgentPlans[i]);
            }
            return new Plan(plans);
        }

        public int GetAgentsSolutionDepth(int agent)
        {
            return allSingleAgentPlans[agent].GetSize() - 1;
        }

        /// <summary>
        /// Merge agent groups that are conflicting in this node if they pass the merge threshold.
        /// Warning: May change the hash code!
        /// </summary>
        /// <param name="mergeThreshold"></param>
        /// <returns>Whether a merge was performed.</returns>
        public bool MergeIf(int mergeThreshold)
        {
            int countConflicts = 1;
            int firstGroupNumber = this.agentsGroupAssignment[conflict.agentA];
            int secondGroupNumber = this.agentsGroupAssignment[conflict.agentB];
            List<int> firstGroup = new List<int>();
            List<int> secondGroup = new List<int>();

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == firstGroupNumber)
                    firstGroup.Add(i);
                if (agentsGroupAssignment[i] == secondGroupNumber)
                    secondGroup.Add(i);
            }

            CbsNode current = this.prev;
            int a, b;
            while (current != null)
            {
                a=current.conflict.agentA;
                b=current.conflict.agentB;
                if ((firstGroup.Contains(a) && secondGroup.Contains(b)) || (firstGroup.Contains(b) && secondGroup.Contains(a)))
                    countConflicts++;
                current = current.prev;
            }
            if (countConflicts > mergeThreshold)
            {
                MergeGroups(firstGroupNumber, secondGroupNumber);
                return true;
            }

            return false;
        }

        /// <summary>
        /// Merge agent groups that are conflicting in this node if they pass the merge threshold.
        /// FIXME: Code dup with previous method
        /// Warning: May change the hash code!
        /// </summary>
        /// <param name="mergeThreshold"></param>
        /// <param name="globalConflictCounter"></param>
        /// <returns>Whether a merge was performed.</returns>
        public bool MergeIf(int mergeThreshold, int[][] globalConflictCounter)
        {
            int conflictCounter = 0;
            int firstGroupNumber = this.agentsGroupAssignment[conflict.agentA];
            int secondGroupNumber = this.agentsGroupAssignment[conflict.agentB];
            List<int> firstGroup = new List<int>();
            List<int> secondGroup = new List<int>();

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (agentsGroupAssignment[i] == firstGroupNumber)
                    firstGroup.Add(i);
                if (agentsGroupAssignment[i] == secondGroupNumber)
                    secondGroup.Add(i);
            }

            foreach (int a in firstGroup)
            {
                foreach (int b in secondGroup)
                {
                    conflictCounter += globalConflictCounter[Math.Max(a, b)][Math.Min(a, b)];
                }
            }
            if (conflictCounter > mergeThreshold)
            {
                MergeGroups(firstGroupNumber, secondGroupNumber);
                return true;
            }

            return false;
        }

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
           // NormalizeGroups();
        }

        public void PrintPlan()
        {
            Debug.WriteLine("Plan:");
            for (int i = 0; i < allSingleAgentPlans.Length; i++)
            {
                Debug.WriteLine("agents - " + i);
                allSingleAgentPlans[i].PrintPlan();
            }
            Debug.WriteLine("End Plan");
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

        public int PathLength(int agent)
        {
            Move[] moves = allSingleAgentPlans[agent].locationAtTimes;
            Move goal = moves[moves.Length - 1];
            for (int i = moves.Length - 2; i >= 0; i--)
            {
                if (!moves[i].Equals(goal))
                    return i+1;
            }
            return 0;
        }

        public bool IsAllowedConstraint(CbsConstraint check)
        {
            CbsNode current = this;
            while (current != null)
            {
                if (current.unConstraint != null && !current.unConstraint.Allows(check))
                    return false;
                current = current.prev;
            }
            return true;
        }

        public void SetUnConstraint(CbsConstraint set)
        {
            this.unConstraint = set;
        }

        public bool GoalTest() {
            return this.conflict == null;
        }

        /// <summary>
        /// ??
        /// </summary>
        /// <param name="agentForRePlan"></param>
        /// <param name="depthToReplan"></param>
        /// <param name="highLevelExpanded"></param>
        /// <param name="highLevelGenerated"></param>
        /// <param name="lowLevelExpanded"></param>
        /// <param name="lowLevelGenerated"></param>
        /// <returns></returns>
        public bool Replan3b(int agentForRePlan, int depthToReplan, ref int highLevelExpanded, ref int highLevelGenerated, ref int lowLevelExpanded, ref int lowLevelGenerated)
        {
            HashSet<TimedMove> newInternalCAT = (HashSet<TimedMove>)problem.parameters[CBS_LocalConflicts.NEW_INTERNAL_CAT];
            newInternalCAT.Clear();
            HashSet<CbsConstraint> newConstraints = this.GetConstraints();
            problem.parameters[CBS_LocalConflicts.NEW_CONSTRAINTS] = newConstraints;
            HashSet_U<TimedMove> InternalCAT = (HashSet_U<TimedMove>)problem.parameters[CBS_LocalConflicts.INTERNAL_CAT];
            HashSet_U<CbsConstraint> Constraints = (HashSet_U<CbsConstraint>)problem.parameters[CBS_LocalConflicts.CONSTRAINTS];
            List<CbsConstraint> mustConstraints = this.GetMustConstraints();
            problem.parameters[CBS_LocalConflicts.CONSTRAINTSP] = mustConstraints;
            ICbsSolver solver = highLevelSolver;

            int groupNum = this.agentsGroupAssignment[agentForRePlan];
            List<AgentState> subGroup = new List<AgentState>();

            //Debug.WriteLine("Sub-problem:");

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                {
                    subGroup.Add(problem.m_vAgents[i]);
                    // Debug.WriteLine(i);
                }
                else
                    allSingleAgentPlans[i].addPlanToHashSet(newInternalCAT, totalCost);
            }

            replanSize = (ushort)subGroup.Count;

            if (subGroup.Count == 1)
                solver = lowLevelSolver;

            ProblemInstance subProblem = problem.Subproblem(subGroup.ToArray());
            subProblem.parameters = problem.parameters;

            InternalCAT.Join(newInternalCAT);
            Constraints.Join(newConstraints);

            //Constraints.print();

            solver.Setup(subProblem, depthToReplan, runner);
            if (solver.Solve() == false)
            {
                highLevelExpanded += solver.GetHighLevelExpanded();
                highLevelGenerated += solver.GetHighLevelGenerated();
                lowLevelExpanded += solver.GetLowLevelExpanded();
                lowLevelGenerated += solver.GetLowLevelGenerated();
                InternalCAT.Seperate(newInternalCAT);
                Constraints.Seperate(newConstraints);
                return false;
            }
            highLevelExpanded += solver.GetHighLevelExpanded();
            highLevelGenerated += solver.GetHighLevelGenerated();
            lowLevelExpanded += solver.GetLowLevelExpanded();
            lowLevelGenerated += solver.GetLowLevelGenerated();

            int j = 0;
            SinglePlan[] singlePlans = solver.GetSinglePlans();

            for (int i = 0; i < agentsGroupAssignment.Length; i++)
            {
                if (this.agentsGroupAssignment[i] == groupNum)
                {
                    this.allSingleAgentPlans[i] = singlePlans[j];
                    j++;
                }
            }

            // PrintPlan();

            totalCost = 0;
            foreach (SinglePlan plan in allSingleAgentPlans)
            {
                totalCost += (ushort)(plan.GetSize() - 1);
            }
            InternalCAT.Seperate(newInternalCAT);
            Constraints.Seperate(newConstraints);
            newConstraints.Clear();
            this.FindConflict(problem);
            // PrintConflict();
            return true;
        }
    }
}
