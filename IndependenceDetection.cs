using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using System.Reflection;
using System.Runtime.CompilerServices;
using ExtensionMethods;

namespace mapf
{
    class IndependenceDetection : ISolver
    {
        protected LinkedList<IndependenceDetectionAgentsGroup> allGroups;
        /// <summary>
        /// For each group in the problem instance, maps group nums it conflicts with to the number of conflicts betweem them.
        /// </summary>
        public Dictionary<int, int>[] conflictCountsPerGroup;
        /// <summary>
        /// For each group in the problem instance, maps group nums of groups it collides with to the time of their first collision.
        /// </summary>
        public Dictionary<int, List<int>>[] conflictTimesPerGroup;
        /// <summary>
        /// For each group in the problem instance, saves the number of agents from the problem instance that it conflicts with.
        /// Used for choosing the next conflict to resolve.
        /// </summary>
        public int[] countsOfGroupsThatConflict;
        public int totalConflictCount = 0;
        protected ProblemInstance instance;
        protected int expanded;  // TODO: remove and just rely on the subsolvers to accumulate
        protected int generated;  // TODO: remove and just rely on the subsolvers to accumulate
        protected int resolutionAttempts;
        protected int resolutionSuccesses;
        protected int merges;
        protected int accExpanded;
        protected int accGenerated;
        protected int accResolutionAttempts;
        protected int accResolutionSuccesses;
        protected int accMerges;
        public bool provideGroupCostsToSolver;
        public int totalCost;
        protected Run runner;

        /// <summary>
        /// The complete plan for all the agents that was found.
        /// </summary>
        public Plan plan;
        
        protected int maxGroupSize;
        protected int minGroupSize;
        protected int accMaxGroupSize;
        protected int accMinGroupSize;
        public IIndependenceDetectionSolver groupSolver;
        public IIndependenceDetectionSolver singleAgentSolver;
        private ISet<IndependenceDetectionConflict> resolutionAttemptedFirstGroup;
        private ISet<IndependenceDetectionConflict> resolutionAttemptedSecondGroup;
        private int solutionDepth;
        private ConflictAvoidanceTable conflictAvoidanceTable;
        private int maxSolutionCostFound;

        public IndependenceDetection(IIndependenceDetectionSolver singleAgentSolver, IIndependenceDetectionSolver groupSolver)
        {
            this.singleAgentSolver = singleAgentSolver;
            this.groupSolver = groupSolver;
        }

        public void Clear()
        {
            this.allGroups.Clear();
            this.singleAgentSolver.Clear();
            this.groupSolver.Clear();
            this.resolutionAttemptedFirstGroup.Clear();
            this.resolutionAttemptedSecondGroup.Clear();
            this.conflictAvoidanceTable.Clear();
            this.solutionDepth = -1;
            this.maxSolutionCostFound = -1;
        }

        public void Setup(ProblemInstance instance, Run runner)
        {
            this.instance = instance;
            this.runner = runner;
            this.totalCost = 0;
            this.ClearStatistics();
            this.conflictAvoidanceTable = new ConflictAvoidanceTable();
            this.conflictAvoidanceTable.avoidanceGoal = ConflictAvoidanceTable.AvoidanceGoal.MINIMIZE_CONFLICTING_GROUPS;  // The effect of a conflict between two groups is total in ID - they're either fully merged or try to fully avoid each other's plan
            this.resolutionAttemptedFirstGroup = new HashSet<IndependenceDetectionConflict>();
            this.resolutionAttemptedSecondGroup = new HashSet<IndependenceDetectionConflict>();
            this.allGroups = new LinkedList<IndependenceDetectionAgentsGroup>();
            // Initialize the agent group collection with a group for every agent
            foreach (AgentState agentStartState in instance.agents)
            {
                this.allGroups.AddLast(new IndependenceDetectionAgentsGroup(
                                            this.instance, new AgentState[1] { agentStartState },
                                            this.singleAgentSolver, this.groupSolver)
                );
            }
            conflictCountsPerGroup = new Dictionary<int, int>[instance.GetNumOfAgents()];
            conflictTimesPerGroup = new Dictionary<int, List<int>>[instance.GetNumOfAgents()];
            for (int i = 0; i < instance.GetNumOfAgents(); i++)
            {
                conflictCountsPerGroup[i] = new Dictionary<int, int>();
                conflictTimesPerGroup[i] = new Dictionary<int, List<int>>();
            }
            countsOfGroupsThatConflict = new int[instance.GetNumOfAgents()];
        }

        public virtual String GetName() { return groupSolver.GetName() + "+ID"; }

        /// <summary>
        /// Calculate the full plan for all the agents that has been found by the algorithm
        /// </summary>
        public Plan CalculateJointPlan() 
        {
            var singlePlans = new SinglePlan[this.instance.GetNumOfAgents()];
            foreach (var group in this.allGroups)
            {
                var groupPlan = group.GetPlan();
                int i = 0;
                foreach (var agentState in group.allAgentsState)
                {
                    singlePlans[agentState.agent.agentNum] = new SinglePlan(groupPlan, i, agentState.agent.agentNum);
                    i++;
                }
                    
            }
            return new Plan(singlePlans);
        }

        public Plan GetPlan()
        {
            return this.plan;
        }

        public int GetSolutionCost() { return this.totalCost; }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            // TODO: Use the solver's statistics, as done in CBS.
            output.Write(this.ToString() + " Expanded");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Max Group Size");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Min Group Size");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Resolution Attempts");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Resolution Successes");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Merges");
            output.Write(Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Prints statistics of a single run to the given output. 
        /// </summary>
        public void OutputStatistics(TextWriter output)//, BsonDocument row)
        {
            Console.WriteLine($"Total Expanded Nodes: {this.expanded}");
            Console.WriteLine($"Total Generated Nodes: {this.generated}");

            output.Write(this.expanded + Run.RESULTS_DELIMITER);
            output.Write(this.generated + Run.RESULTS_DELIMITER);

            this.minGroupSize = this.allGroups.Min(group => group.allAgentsState.Length);  // MaxGroupSize is computed every time we merge

            Console.WriteLine($"Max Group: {this.maxGroupSize}");
            Console.WriteLine($"Min Group: {this.minGroupSize}");

            output.Write(this.maxGroupSize + Run.RESULTS_DELIMITER);
            output.Write(this.minGroupSize + Run.RESULTS_DELIMITER);

            Console.WriteLine($"Resolution Attempts: {this.resolutionAttempts}");
            Console.WriteLine($"Resolution Successes: {this.resolutionAttempts}");
            Console.WriteLine($"Merges: {this.merges}");

            output.Write(this.resolutionAttempts + Run.RESULTS_DELIMITER);
            output.Write(this.resolutionSuccesses + Run.RESULTS_DELIMITER);
            output.Write(this.merges + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 7;
            }
        }

        public void ClearStatistics()
        {
            this.expanded = 0;
            this.generated = 0;
            this.maxGroupSize = 1;
            this.minGroupSize = instance.agents.Length;
            this.resolutionAttempts = 0;
            this.resolutionSuccesses = 0;
            this.merges = 0;
        }

        public void ClearAccumulatedStatistics()
        {
            this.accExpanded = 0;
            this.accGenerated = 0;
            this.accMaxGroupSize = 1;
            this.accMinGroupSize = this.instance.agents.Length;
            this.accResolutionAttempts = 0;
            this.accResolutionSuccesses = 0;
            this.accMerges = 0;
        }

        public void AccumulateStatistics()
        {
            this.accExpanded += this.expanded;
            this.accGenerated += this.generated;
            this.accMaxGroupSize = Math.Max(this.accMaxGroupSize, this.maxGroupSize);
            this.accMinGroupSize = Math.Min(this.accMinGroupSize, this.minGroupSize);
            this.accResolutionAttempts += this.resolutionAttempts;
            this.accResolutionSuccesses += this.resolutionSuccesses;
            this.accMerges += this.merges;
        }

        public void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine($"{this} Accumulated Expanded Nodes (Low-Level): {this.accExpanded}");
            Console.WriteLine($"{this} Accumulated Generated Nodes (Low-Level): {this.accGenerated}");

            output.Write(this.accExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accGenerated + Run.RESULTS_DELIMITER);

            Console.WriteLine($"{this} Accumulated Max Group (Low-Level): {this.accMaxGroupSize}");
            Console.WriteLine($"{this} Accumulated Min Group (Low-Level): {this.accMinGroupSize}");

            output.Write(this.accMaxGroupSize + Run.RESULTS_DELIMITER);
            output.Write(this.accMinGroupSize + Run.RESULTS_DELIMITER);

            Console.WriteLine($"{this} Accumulated resolution attempts (Low-Level): {this.accResolutionAttempts}");
            Console.WriteLine($"{this} Accumulated resolution successes (Low-Level): {this.accResolutionSuccesses}");
            Console.WriteLine($"{this} Accumulated merges (Low-Level): {this.accMerges}");

            output.Write(this.accResolutionAttempts + Run.RESULTS_DELIMITER);
            output.Write(this.accResolutionSuccesses + Run.RESULTS_DELIMITER);
            output.Write(this.accMerges + Run.RESULTS_DELIMITER);
        }

        /// <summary>
        /// Also calculates min group size along the way.
        /// FIXME: Code dup!
        /// </summary>
        /// <returns></returns>
        public int GetMaxGroupSize()
        {
            this.maxGroupSize = this.allGroups.Max(group => group.allAgentsState.Length);
            this.minGroupSize = this.allGroups.Min(group => group.allAgentsState.Length);
            return this.maxGroupSize;
        }

        public enum ConflictChoice : byte
        {
            FIRST = 0,
            MOST_CONFLICTING_SMALLEST_AGENTS
        }
        public ConflictChoice conflictChoice = ConflictChoice.MOST_CONFLICTING_SMALLEST_AGENTS;  // TODO: set it in the constructor.

        /// <summary>
        /// Simulates the execution of the plans found for the different groups. 
        /// If there are conflicting plans - return the conflicting groups.
        /// </summary>
        /// <returns>A conflict object with data about the found conflict, or null if no conflict exists</returns>
        public IndependenceDetectionConflict ChooseConflict()
        {
            if (this.allGroups.Count == 1)  // A single group can't conflict with itself
                return null;

            if (totalConflictCount == 0)
                return null;

            if (this.conflictChoice == IndependenceDetection.ConflictChoice.FIRST)
            {
                return this.ChooseFirstConflict();
            }
            else if (this.conflictChoice == IndependenceDetection.ConflictChoice.MOST_CONFLICTING_SMALLEST_AGENTS)
            {
                return this.ChooseConflictOfMostConflictingSmallestAgents();
            }
            else
                throw new Exception("Unknown conflict choosing method");
        }

        private int GetGroupSize(int groupNum)
        {
            foreach (var group in this.allGroups)
            {
                if (group.groupNum == groupNum)
                    return group.allAgentsState.Length;
            }
            return -1;
        }

        /// <summary>
        /// Populates the countsOfInternalGroupsThatConflict counters
        /// from the conflictCountsPerGroup values that are created while solving or replanning.
        /// Those counters are used for tie-breaking.
        /// </summary>
        protected void CountConflicts()
        {
            totalConflictCount = 0;
            for (int i = 0; i < this.conflictCountsPerGroup.Length; i++)
            {
                this.countsOfGroupsThatConflict[i] = 0;

                if (this.conflictCountsPerGroup[i] == null)
                    continue;

                foreach (KeyValuePair<int, int> conflictingGroupNumAndCount in conflictCountsPerGroup[i])
                {
                    this.countsOfGroupsThatConflict[i]++; // Counts one conflict for each agent the i'th agent conflicts with
                    totalConflictCount += conflictingGroupNumAndCount.Value;
                }
            }

            totalConflictCount /= 2;  // Each conflict was counted twice
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
        private IndependenceDetectionConflict ChooseConflictOfMostConflictingSmallestAgents()
        {
            int groupRepA = -1; // To quiet the compiler
            int groupRepB = -1; // To quiet the compiler
            int time = int.MaxValue;
            Func<int, double> formula = i => this.conflictCountsPerGroup[i] != null ?
                                                this.countsOfGroupsThatConflict[i] / ((double)(1 << (this.GetGroupSize(i) - 1)))
                                                : -1;

            int chosenGroupNum = Enumerable.Range(0, this.instance.agents.Length).MaxByKeyFunc(formula);

            // We could just look for any of this agent's conflicts,
            // but the best choice among the agents it conflicts with is the one which maximizes the formula itself.
            IEnumerable<int> conflictsWithGroupNums = this.conflictCountsPerGroup[chosenGroupNum].Keys;
            int chosenConflictingGroupNum = conflictsWithGroupNums.MaxByKeyFunc(formula);

            groupRepA = chosenGroupNum;
            groupRepB = chosenConflictingGroupNum;

            time = this.conflictTimesPerGroup[chosenGroupNum][chosenConflictingGroupNum][0];  // Choosing the earliest conflict between them - the choice doesn't matter for ID, but this is consistent with CBS' strategy
            
            IndependenceDetectionAgentsGroup groupA = null, groupB = null;
            foreach (var group in this.allGroups)
            {
                if (group.groupNum == groupRepA)
                    groupA = group;
                else if (group.groupNum == groupRepB)
                    groupB = group;
            }
            return new IndependenceDetectionConflict(groupA, groupB, time);
        }

        private IndependenceDetectionConflict ChooseFirstConflict()
        {
            int groupRepA = -1; // To quiet the compiler
            int groupRepB = -1; // To quiet the compiler
            int time = int.MaxValue;
            for (int i = 0; i < this.conflictTimesPerGroup.Length; i++)
            {
                if (conflictCountsPerGroup[i] == null)
                    continue;

                foreach (var otherGroupNumAndConflictTimes in this.conflictTimesPerGroup[i])
                {
                    if (otherGroupNumAndConflictTimes.Value[0] < time)
                    {
                        time = otherGroupNumAndConflictTimes.Value[0];
                        groupRepA = i;
                        groupRepB = otherGroupNumAndConflictTimes.Key;
                    }
                }
            }
            IndependenceDetectionAgentsGroup groupA = null, groupB = null;
            foreach (var group in this.allGroups)
            {
                if (group.groupNum == groupRepA)
                    groupA = group;
                else if (group.groupNum == groupRepB)
                    groupB = group;
            }
            return new IndependenceDetectionConflict(groupA, groupB, time);
        }

        public IndependenceDetectionConflict FindFirstConflict()
        { 
            // Find the longest plan among all the groups
            int maxPlanSize = this.allGroups.Max(group => group.GetPlan().GetSize());

            Plan[] plans = this.allGroups.Select(group => group.GetPlan()).ToArray();

            if (this.debug)
            {
                var globalPlan = new Plan(plans);
                Debug.WriteLine($"{globalPlan}");
            }

            // Check in every time step that the plans do not collide
            for (int time = 1 ; time < maxPlanSize ; time++) // Assuming no conflicts exist in time zero.
            {
                // Check all pairs of groups for a conflict at the given time step
                foreach ((int i1, var group1) in this.allGroups.Enumerate())
                {
                    Plan group1Plan = plans[i1];
                    foreach ((int i2, var group2) in this.allGroups.Enumerate())
                    {
                        Plan group2Plan = plans[i2];
                        if (i1 < i2 && group1Plan.IsColliding(time, group2Plan))
                            return new IndependenceDetectionConflict(group1, group2, time);
                    }
                }
            }
            return null;
        }

        /// <summary>
        /// Search for an optimal solution using the Simple Independence Detection algorithm from Trevor Standley's paper.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool SimpleID(Run runner)
        {
            while (true)
            {
                IndependenceDetectionConflict conflict = FindFirstConflict();
                // If there are no conflicts - can finish the run
                if (conflict == null)
                    break;
                allGroups.Remove(conflict.group1);
                allGroups.Remove(conflict.group2);
                IndependenceDetectionAgentsGroup compositeGroup = this.JoinGroups(conflict);
                ++merges;
                
                // Solve composite group with A*
                bool solved = compositeGroup.Solve(runner, conflictAvoidanceTable);
                if (solved == false)
                {
                    this.totalCost = compositeGroup.solutionCost;
                    return false;
                }

                allGroups.AddFirst(compositeGroup);
            }
            return true;
        }

        public bool debug = false;

        /// <summary>
        /// Search for an optimal solution using the Independence Detection algorithm in Standley's paper,
        /// which utilises a CAT.
        /// </summary>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool ImprovedID(Run runner)
        {
            while (true)
            {
                if (this.debug)
                {
                    Debug.WriteLine($"{this.totalConflictCount} conflicts");
                }

                IndependenceDetectionConflict conflict = ChooseConflict();
                // If there are no conflicts - can return the current plan
                if (conflict == null)
                    break;

                if (this.debug)
                {
                    Debug.WriteLine($"{this.allGroups.Count} groups: {String.Join(", ", this.allGroups)}");

                    Debug.Write("Group single agent costs: ");
                    foreach (var group in this.allGroups)
                    {
                        Debug.Write($"{{{String.Join(" ", group.GetCosts())}}}, ");
                    }
                    Debug.WriteLine("");

                    for (int j = 0; j < this.conflictTimesPerGroup.Length; j++)
                    {
                        if (this.conflictTimesPerGroup[j] != null)
                        {
                            Debug.Write($"Group {j} conflict times: ");
                            foreach (var pair in this.conflictTimesPerGroup[j])
                            {
                                Debug.Write($"{pair.Key}:[{String.Join(",", pair.Value)}], ");
                            }
                            Debug.WriteLine("");
                        }
                    }

                    var plan = this.CalculateJointPlan();
                    if (plan.GetSize() < 200)
                        Debug.WriteLine(plan);
                    else
                        Debug.WriteLine($"Plan is too long to print ({plan.GetSize()} steps)");

                    Debug.WriteLine($"Chose {conflict}");
                }

                // Try to resolve the current conflict by re-planning one of the groups' path
                if (this.resolutionAttemptedFirstGroup.Contains(conflict) == false)  // We haven't already tried to resolve this conflict
                                                                                     // without merging the groups by replanning the first group's path
                {
                    // Prevent trying to resolve this conflict this way again
                    this.resolutionAttemptedFirstGroup.Add(conflict);

                    // Add the plan of group2 to the illegal moves table and re-plan group1 with equal cost
                    if ((conflict.time < conflict.group1.GetPlan().GetSize() - 1) ||
                        (conflict.group1.Size() > 1))  // Otherwise the conflict is while a single agent
                                                       // is at its goal, no chance of an alternate path
                                                       // with the same cost that avoids the conflict - TODO: If it's an edge conflict while entering the goal it may be resolvable
                    {
                        if (this.debug)
                        {
                            Debug.WriteLine($"Trying to find an alternative path that avoids the conflict for {conflict.group1}.");
                            //Debug.WriteLine($"Old plan:\n{conflict.group1.GetPlan()}");
                        }
                        conflict.group1.removeGroupFromCAT(conflictAvoidanceTable);
                        bool resolved = conflict.group1.ReplanUnderConstraints(conflict.group2.GetPlan(), runner, this.conflictAvoidanceTable);
                        ++resolutionAttempts;
                        this.expanded += conflict.group1.expanded;
                        this.generated += conflict.group1.generated;
                        if (resolved == true)
                        {
                            if (this.debug)
                            {
                                //Debug.WriteLine($"Found an alternative path that avoids the conflict for group 1: {conflict.group1.GetPlan()}");
                                Debug.WriteLine($"Found an alternative path that avoids the conflict for {conflict.group1}");
                            }

                            UpdateConflictCounts(conflict.group1);
                            conflict.group1.addGroupToCAT(conflictAvoidanceTable);
                            ++resolutionSuccesses;

                            continue;
                        }
                        else
                            conflict.group1.addGroupToCAT(conflictAvoidanceTable);

                        if (this.debug)
                        {
                            Debug.WriteLine($"Couldn't find an alternative path that avoids the conflict for {conflict.group1}");
                        }
                    }
                    else
                    {
                        if (this.debug)
                        {
                            Debug.WriteLine($"Not trying to find an alternative path that avoids the conflict for {conflict.group1} because " +
                                             "the group contains a single agent and the conflict happens after it reaches its goal.");
                        }
                    }
                }
                else
                {
                    if (this.debug)
                    {
                        Debug.WriteLine($"Not trying to find an alternative path that avoids the conflict for {conflict.group1} - " +
                                         "we've already tried to in the past.");
                    }
                }
                
                if (this.resolutionAttemptedSecondGroup.Contains(conflict) == false)  // We haven't already tried to resolve this conflict
                                                                                      // without merging the groups by replanning the second group's path
                {
                    // Prevent trying to resolve this conflict this way again
                    this.resolutionAttemptedSecondGroup.Add(conflict);

                    // Add the plan of group1 to the illegal moves table and re-plan group2 with equal cost
                    if ((conflict.time < conflict.group2.GetPlan().GetSize() - 1) ||
                        (conflict.group2.Size() > 1))
                    {
                        if (this.debug)
                        {
                            Debug.WriteLine($"Trying to find an alternative path that avoids the conflict for {conflict.group2}");
                            //Debug.WriteLine($"Old plan: {conflict.group2.GetPlan()}");
                        }
                        conflict.group2.removeGroupFromCAT(conflictAvoidanceTable);
                        bool resolved = conflict.group2.ReplanUnderConstraints(conflict.group1.GetPlan(), runner, this.conflictAvoidanceTable);
                        ++resolutionAttempts;
                        this.expanded += conflict.group2.expanded;
                        this.generated += conflict.group2.generated;
                        if (resolved == true)
                        {
                            if (this.debug)
                            {
                                //Debug.WriteLine($"Found an alternative path that avoids the conflict for group 2: {conflict.group2.GetPlan()}");
                                Debug.WriteLine($"Found an alternative path that avoids the conflict for {conflict.group2}");
                            }

                            UpdateConflictCounts(conflict.group2);
                            conflict.group2.addGroupToCAT(conflictAvoidanceTable);
                            ++resolutionSuccesses;

                            continue;
                        }
                        else
                            conflict.group2.addGroupToCAT(conflictAvoidanceTable);

                        if (this.debug)
                        {
                            Debug.WriteLine($"Couldn't find an alternative path that avoids the conflict for {conflict.group2}");
                        }
                    }
                    else {
                        if (this.debug)
                        {
                            Debug.WriteLine($"Not trying to find an alternative path that avoids the conflict for {conflict.group2} because " +
                                             "the group contains a single agent and the conflict happens after it reaches its goal.");
                        }
                    }
                }
                else
                {
                    if (this.debug)
                    {
                        Debug.WriteLine($"Not trying to find an alternative path that avoids the conflict for {conflict.group2} - " +
                                         "we've already tried to in the past.");
                    }
                }

                int group1Size = conflict.group1.Size();
                int group1Cost = conflict.group1.solutionCost;
                int group2Cost = conflict.group2.solutionCost;
                // Groups are conflicting - need to join them to a single group
                allGroups.Remove(conflict.group1);
                allGroups.Remove(conflict.group2);
                // Remove both groups from avoidance table
                conflict.group1.removeGroupFromCAT(conflictAvoidanceTable);
                conflict.group2.removeGroupFromCAT(conflictAvoidanceTable);
                conflictCountsPerGroup[conflict.group1.groupNum] = null;
                conflictTimesPerGroup[conflict.group1.groupNum] = null;
                conflictCountsPerGroup[conflict.group2.groupNum] = null;
                conflictTimesPerGroup[conflict.group2.groupNum] = null;
                // Remove the old groups from the conflict counts - new counts will be put there after replanning
                for (int i = 0; i < this.conflictCountsPerGroup.Length; i++)
                {
                    this.conflictCountsPerGroup[i]?.Remove(conflict.group1.groupNum);
                    this.conflictCountsPerGroup[i]?.Remove(conflict.group2.groupNum);
                    this.conflictTimesPerGroup[i]?.Remove(conflict.group1.groupNum);
                    this.conflictTimesPerGroup[i]?.Remove(conflict.group2.groupNum);
                }
                if (this.debug)
                {
                    Debug.WriteLine($"Merging the agent groups that participate in {conflict}.");
                    //Debug.WriteLine($"Group1 plan before the merge: {conflict.group1.GetPlan()}");
                    //Debug.WriteLine($"Group2 plan before the merge: {conflict.group2.GetPlan()}");
                }

                IndependenceDetectionAgentsGroup compositeGroup = this.JoinGroups(conflict);
                ++merges;

                // Solve composite group with the underlying group solver
                bool solved = compositeGroup.Solve(runner, conflictAvoidanceTable,
                                                   group1Cost, group2Cost, group1Size);

                if (compositeGroup.solutionCost > maxSolutionCostFound)
                    maxSolutionCostFound = compositeGroup.solutionCost;

                this.expanded += compositeGroup.expanded;
                this.generated += compositeGroup.generated;

                if (compositeGroup.allAgentsState.Length > this.maxGroupSize)
                    this.maxGroupSize = compositeGroup.allAgentsState.Length;

                if (solved == false)
                {
                    this.totalCost = Constants.NO_SOLUTION_COST;
                    return false;
                }

                UpdateConflictCounts(compositeGroup);

                // Add the new group to conflict avoidance table
                compositeGroup.addGroupToCAT(conflictAvoidanceTable);
                allGroups.AddFirst(compositeGroup);
            }
            return true;
        }

        void UpdateConflictCounts(IndependenceDetectionAgentsGroup group)
        {
            conflictCountsPerGroup[group.groupNum] = group.conflictCounts;
            conflictTimesPerGroup[group.groupNum] = group.conflictTimes;

            // Update conflict counts with what happens after the plan finishes
            this.IncrementConflictCountsAtGoal(group, conflictAvoidanceTable);

            // Update conflictCountsPerGroup and conflictTimesPerGroup for all other groups
            for (int i = 0; i < this.conflictCountsPerGroup.Length; ++i)
            {
                if (this.conflictCountsPerGroup[i] == null || i == group.groupNum)
                    continue;

                if (group.conflictCounts.ContainsKey(i))
                {
                    this.conflictCountsPerGroup[i][group.groupNum] = group.conflictCounts[i];
                    this.conflictTimesPerGroup[i][group.groupNum] = group.conflictTimes[i];
                }
                else
                {
                    this.conflictCountsPerGroup[i].Remove(group.groupNum);
                    this.conflictTimesPerGroup[i].Remove(group.groupNum);
                }
            }

            CountConflicts();
        }

        /// <summary>
        /// Update conflict counts according to what happens after the plan finishes -
        /// needed if the plan is shorter than one of the previous plans and collides
        /// with it while at the goal.
        /// It's cheaper to do it this way than to force the solver the go more deeply.
        /// The conflict counts are saved at the group's representative.
        /// </summary>
        protected void IncrementConflictCountsAtGoal(IndependenceDetectionAgentsGroup group, ConflictAvoidanceTable CAT)
        {
            for (int i = 0; i < group.allAgentsState.Length; ++i)
            {
                var afterGoal = new TimedMove(group.allAgentsState[i].agent.Goal.x, group.allAgentsState[i].agent.Goal.y, Move.Direction.Wait, time: 0);
                for (int time = group.GetPlan().GetSize(); time < CAT.GetMaxPlanSize(); time++)
                {
                    afterGoal.time = time;
                    afterGoal.IncrementConflictCounts(CAT,
                                                      this.conflictCountsPerGroup[group.groupNum],
                                                      this.conflictTimesPerGroup[group.groupNum]);
                }
            }
        }

        /// <summary>
        /// Join the conflicting groups into a single group
        /// </summary>
        /// <param name="conflict">An object that describes the conflict</param>
        /// <returns>The composite group of agents</returns>
        protected virtual IndependenceDetectionAgentsGroup JoinGroups(IndependenceDetectionConflict conflict)
        {
            return conflict.group1.Join(conflict.group2);
        }

        /// <summary>
        /// Run the A* algorithm with Standley's ID and OD improvements.
        /// </summary>
        /// <returns>true if optimal solution has been found</returns>
        public bool Solve()
        {
            bool solved;

            // Solve the single agent problems independently
            this.maxSolutionCostFound = 0;

            foreach (var group in this.allGroups)
            {
                solved = group.Solve(runner, this.conflictAvoidanceTable);

                // Check if max time has been exceeded or search failed for another reason
                if (solved == false)
                {
                    this.totalCost = group.solutionCost; // Should be some error code from Constants.
                    this.Clear();
                    return false;
                }

                if (group.solutionCost > this.maxSolutionCostFound)
                    this.maxSolutionCostFound = group.solutionCost;

                conflictCountsPerGroup[group.groupNum] = group.conflictCounts;
                conflictTimesPerGroup[group.groupNum] = group.conflictTimes;

                this.IncrementConflictCountsAtGoal(group, conflictAvoidanceTable);

                // Add group to conflict avoidance table
                group.addGroupToCAT(this.conflictAvoidanceTable);

                this.expanded += group.expanded;
                this.generated += group.generated;
            }

            // Update conflict counts: All agents but the last saw an incomplete CAT. Update counts backwards.
            for (int i = this.conflictCountsPerGroup.Length - 1; i >= 0; i--)
            {
                foreach (KeyValuePair<int, int> pair in this.conflictCountsPerGroup[i])
                {
                    if (pair.Key < i)  // Just an optimization. Would also be correct without this check.
                    {
                        this.conflictCountsPerGroup[pair.Key][i] = pair.Value; // Collisions are symmetrical, and agent "key" didn't see the route for agent "i" when planning.
                        this.conflictTimesPerGroup[pair.Key][i] = this.conflictTimesPerGroup[i][pair.Key];
                    }
                }
            }

            CountConflicts();

            //solved = this.SimpleID(runner);  // TODO: Consider adding a parameter to choose this option
            solved = this.ImprovedID(runner);
            // Record found solution
            if (solved == true)
            {
                // Store solution details
                this.totalCost = this.allGroups.Sum(group => group.solutionCost);  // TODO: Support the makespan cost function
                this.plan = this.CalculateJointPlan();
            }
            else
            {
                this.plan = null;
            }

            // TODO: Add a statistic for the number of groups
            return solved;
        }

        public override string ToString()
        {
            return GetName();
        }

        private void print()
        {
            Console.WriteLine("Expanded - " + expanded);
            Console.WriteLine("Generated - " + generated);
            Console.WriteLine("Total cost - " + totalCost);
        }

        public int GetExpanded() { return this.expanded; }
        public int GetGenerated() { return this.generated; }
        public int GetSolutionDepth()
        {
            this.solutionDepth = this.allGroups.Sum(group => group.solutionDepth);  // TODO: Support the makespan cost function
            return this.solutionDepth; 
        }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
    }


    /// <summary>
    /// This class represents a group of agents that need to be solved together.
    /// </summary>
    class IndependenceDetectionAgentsGroup
    {
        public AgentState[] allAgentsState;
        public int solutionCost;
        public ProblemInstance instance;
        public int expanded;
        public int generated;
        public int solutionDepth;
        public int groupNum;

        private IIndependenceDetectionSolver singleAgentSolver;  // Note this allows groups to be given different solvers, according to perhaps their size
        private IIndependenceDetectionSolver groupSolver;
        private Plan plan;
        private int[] singleCosts;
        public Dictionary<int, int> conflictCounts;
        public Dictionary<int, List<int>> conflictTimes;

        public IndependenceDetectionAgentsGroup(ProblemInstance instance, AgentState[] allAgentsState,
                                                IIndependenceDetectionSolver singleAgentSolver, IIndependenceDetectionSolver groupSolver)
        {
            this.allAgentsState = allAgentsState;
            this.instance = instance.Subproblem(allAgentsState);
            this.singleAgentSolver = singleAgentSolver;
            this.groupSolver = groupSolver;
            this.groupNum = allAgentsState[0].agent.agentNum;
        }

        /// <summary>
        /// Solve the group of agents together.
        /// </summary>
        /// <param name="runner"></param>
        /// <param name="CAT"></param>
        /// <param name="group1Cost"></param>
        /// <param name="group2Cost"></param>
        /// <param name="group1Size"></param>
        /// <param name="reserved"></param>
        /// <returns>true if optimal solution for the group of agents were found, false otherwise</returns>
        public bool Solve(Run runner, ConflictAvoidanceTable CAT,
                          int group1Cost = 0, int group2Cost = 0, int group1Size = 1
                          )
        {
            IIndependenceDetectionSolver relevantSolver = this.groupSolver;
            if (this.allAgentsState.Length == 1)
                relevantSolver = this.singleAgentSolver; // TODO: Consider using CBS's root trick to really get single agent paths fast. Though it won't respect illegal moves or avoid conflicts.
            relevantSolver.Setup(this.instance, runner, CAT, group1Cost, group2Cost, group1Size);
            bool solved = relevantSolver.Solve();
            this.solutionCost = relevantSolver.GetSolutionCost();
            if (solved == false)
                return false;

            // Store the plan found by the solver
            this.plan = relevantSolver.GetPlan();
            this.singleCosts = relevantSolver.GetSingleCosts();
            this.expanded = relevantSolver.GetExpanded();
            this.generated = relevantSolver.GetGenerated();
            this.solutionDepth = relevantSolver.GetSolutionDepth();
            this.conflictCounts = relevantSolver.GetExternalConflictCounts();
            this.conflictTimes = relevantSolver.GetConflictTimes();

            // Clear memory
            relevantSolver.Clear();
            return true;
        }

        /// <summary>
        /// Returns the plan for the group of agents. This is a collection of Moves for every time step until all the agents reach their goal.
        /// </summary>
        public Plan GetPlan()
        {
            return this.plan;
        }

        public int[] GetCosts()
        {
            return this.singleCosts;
        }

        /// <summary>
        /// Joins this and another group to a single group with all of the agents together.
        /// </summary>
        /// <param name="other"></param>
        /// <returns>A new AgentsGroup object with the agents from both this and the other group</returns>
        public IndependenceDetectionAgentsGroup Join(IndependenceDetectionAgentsGroup other)
        {
            AgentState[] joinedAgentStates = new AgentState[allAgentsState.Length + other.allAgentsState.Length];
            this.allAgentsState.CopyTo(joinedAgentStates, 0);
            other.allAgentsState.CopyTo(joinedAgentStates, this.allAgentsState.Length);
            if (this.groupSolver.GetType() != typeof(CostTreeSearchSolverOldMatching))
                Array.Sort(joinedAgentStates, (x, y) => x.agent.agentNum.CompareTo(y.agent.agentNum));  // TODO: Technically could be a merge. FIXME: Is this necessary at all?
            
            return new IndependenceDetectionAgentsGroup(this.instance, joinedAgentStates, this.singleAgentSolver, this.groupSolver);
        }

        /// <summary>
        /// Returns the number of agents in the group.
        /// </summary>
        public int Size()
        {
            return this.allAgentsState.Length;
        }

        public override bool Equals(object obj)
        {
            if (obj == null)
                return false;
            IndependenceDetectionAgentsGroup other = (IndependenceDetectionAgentsGroup)obj;
            return allAgentsState.SequenceEqual(other.allAgentsState);
        }

        public override int GetHashCode()
        {
            int ret = 0;
            int i = 0;
            foreach (var agentState in allAgentsState)
            {
                ret += Constants.PRIMES_FOR_HASHING[i % 10] * agentState.GetHashCode();
                i++;
            }
            return ret;
        }

        /// <summary>
        /// Tries to find a plan for this group, that will not conflict with the given plan,
        /// and still has the same solution cost as the current solution cost.
        /// This is used in the ImprovedID() method.
        /// </summary>
        /// <param name="planToAvoid"></param>
        /// <param name="runner"></param>
        /// <returns></returns>
        public bool ReplanUnderConstraints(Plan planToAvoid, Run runner, ConflictAvoidanceTable CAT)
        {
            int oldCost = this.solutionCost;
            Plan oldPlan = this.plan;
            int[] oldCosts = this.singleCosts;
            int oldSolutionDepth = this.solutionDepth;
            Dictionary<int, int> oldConflictCounts = this.conflictCounts;
            Dictionary<int, List<int>> oldConflictTimes = this.conflictTimes;
            HashSet<TimedMove> reserved = new HashSet<TimedMove>();
            planToAvoid.AddPlanToHashSet(reserved, Math.Max(planToAvoid.GetSize(), this.plan.GetSize()));

            IIndependenceDetectionSolver relevantSolver = this.groupSolver;
            if (this.allAgentsState.Length == 1)
                relevantSolver = this.singleAgentSolver;
            relevantSolver.Setup(this.instance, runner, CAT, oldCost, reserved);
            bool solved = relevantSolver.Solve();
            this.solutionCost = relevantSolver.GetSolutionCost();

            conflictCounts = relevantSolver.GetExternalConflictCounts();
            conflictTimes = relevantSolver.GetConflictTimes();
            
            // Store the plan found by the solver
            this.plan = relevantSolver.GetPlan();
            this.singleCosts = relevantSolver.GetSingleCosts();
            this.expanded = relevantSolver.GetExpanded();
            this.generated = relevantSolver.GetGenerated();
            this.solutionDepth = relevantSolver.GetSolutionDepth();
            this.conflictCounts = relevantSolver.GetExternalConflictCounts();
            this.conflictTimes = relevantSolver.GetConflictTimes();

            // Clear memory
            relevantSolver.Clear();

            if (solved == false)
            {
                this.solutionCost = oldCost;
                this.plan = oldPlan;
                this.singleCosts = oldCosts;
                this.solutionDepth = oldSolutionDepth;
                this.conflictCounts = oldConflictCounts;
                this.conflictTimes = oldConflictTimes;
            }
            return solved;
        }

        public void addGroupToCAT(ConflictAvoidanceTable CAT)
        {
            if (this.plan == null)
                return;

            for (int i = 0; i < this.allAgentsState.Length; i++)
            {
                var singleAgentPlan = new SinglePlan(this.plan, i, this.groupNum);  // Note all the plans are inserted under the group's identifier
                CAT.AddPlan(singleAgentPlan);
            }
        }

        public void removeGroupFromCAT(ConflictAvoidanceTable CAT)
        {
            if (this.plan == null)
                return;

            for (int i = 0; i < this.allAgentsState.Length; i++)
            {
                var singleAgentPlan = new SinglePlan(this.plan, i, this.groupNum);
                CAT.RemovePlan(singleAgentPlan);
            }
        }

        public override string ToString()
        {
            string ans = "group {";
            foreach (var agentState in this.allAgentsState)
            {
                ans += agentState.agent.agentNum + ", ";
            }
            ans += "}";
            return ans;
        }
    }
}
