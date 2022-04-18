using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using ExtensionMethods;

namespace mapf;

class IndependenceDetection : ISolver
{
    protected LinkedList<IndependenceDetectionAgentsGroup> allGroups;
    /// <summary>
    /// For each group in the problem instance, maps group nums it conflicts with to the number of conflicts betweem them.
    /// (Indices that aren't group nums are null)
    /// </summary>
    public Dictionary<int, int>[] conflictCountsPerGroup;
    /// <summary>
    /// For each group in the problem instance, maps group nums of groups it collides with to the times they conflict.
    /// (Indices that aren't group nums are null)
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
    private int maxSolutionCostFound;  // FIXME: Maintained but not used
    private ConflictAvoidanceTable.AvoidanceGoal avoidanceGoal;
    private bool simple;

    public IndependenceDetection(IIndependenceDetectionSolver singleAgentSolver, IIndependenceDetectionSolver groupSolver,
                                 ConflictChoice conflictChoice = ConflictChoice.MOST_CONFLICTING_SMALLEST_AGENTS,
                                 bool provideGroupCostsToSolver = true,
                                 ConflictAvoidanceTable.AvoidanceGoal avoidanceGoal = ConflictAvoidanceTable.AvoidanceGoal.MINIMIZE_CONFLICTING_GROUPS,  // The effect of a conflict between two groups is total in ID - they're either fully merged or try to fully avoid each other's plan
                                 bool simple = false
                                 )
    {
        this.singleAgentSolver = singleAgentSolver;
        this.groupSolver = groupSolver;
        this.conflictChoice = conflictChoice;
        this.provideGroupCostsToSolver = provideGroupCostsToSolver;
        this.avoidanceGoal = avoidanceGoal;
        this.simple = simple;
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
        this.conflictAvoidanceTable.avoidanceGoal = this.avoidanceGoal;
        this.resolutionAttemptedFirstGroup = new HashSet<IndependenceDetectionConflict>();
        this.resolutionAttemptedSecondGroup = new HashSet<IndependenceDetectionConflict>();
        this.allGroups = new LinkedList<IndependenceDetectionAgentsGroup>();
        // Initialize the agent group collection with a group for every agent
        foreach (AgentState agentStartState in instance.agents)
        {
            this.allGroups.AddLast(new IndependenceDetectionAgentsGroup(
                                        this.instance, new AgentState[1] { agentStartState },
                                        this.singleAgentSolver, this.groupSolver, this)
            );
            this.conflictAvoidanceTable.agentSizes[agentStartState.agent.agentNum] = 1;
            this.conflictAvoidanceTable.agentConflictCounts[agentStartState.agent.agentNum] = 0;
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

        public virtual String GetName() { return $"{groupSolver.GetName()}+ID({conflictChoice} ProvideInfoToSubsolver={provideGroupCostsToSolver})"; }

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

        this.minGroupSize = this.allGroups.Min(group => group.Size());  // MaxGroupSize is computed every time we merge

        Console.WriteLine($"Max Group: {this.maxGroupSize}");
        Console.WriteLine($"Min Group: {this.minGroupSize}");

        output.Write(this.maxGroupSize + Run.RESULTS_DELIMITER);
        output.Write(this.minGroupSize + Run.RESULTS_DELIMITER);

        Console.WriteLine($"Resolution Attempts: {this.resolutionAttempts}");
        Console.WriteLine($"Resolution Successes: {this.resolutionSuccesses}");
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
        MOST_CONFLICTING_SMALLEST_RESULTING_GROUP,
        MOST_CONFLICTING_AND_SMALLEST_GROUP,
        LEAST_CONFLICTING_LARGEST_RESULTING_GROUP,
        MOST_CONFLICTING_SMALLEST_AGENTS
    }
    public ConflictChoice conflictChoice;

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
        else if (this.conflictChoice == IndependenceDetection.ConflictChoice.MOST_CONFLICTING_SMALLEST_RESULTING_GROUP)
        {
            return this.ChooseConflictOfMostConflictingSmallestResultingGroup();
        }
        else if (this.conflictChoice == IndependenceDetection.ConflictChoice.LEAST_CONFLICTING_LARGEST_RESULTING_GROUP)
        {
            return this.ChooseConflictOfLeastConflictingLargestResultingGroup();
        }
        else if (this.conflictChoice == IndependenceDetection.ConflictChoice.MOST_CONFLICTING_SMALLEST_AGENTS)
        {
            return this.ChooseConflictOfSmallestAgentsThatConflictWithMostOtherAgents();
        }
        else if (this.conflictChoice == IndependenceDetection.ConflictChoice.MOST_CONFLICTING_AND_SMALLEST_GROUP)
        {
            return this.ChooseConflictOfMostConflictingAndSmallestResultingGroup();
        }
        else
            throw new Exception("Unknown conflict choosing method");
    }

    private int GetGroupSize(int groupNum)
    {
        foreach (var group in this.allGroups)
        {
            if (group.groupNum == groupNum)
                return group.Size();
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
    private IndependenceDetectionConflict ChooseConflictOfSmallestAgentsThatConflictWithMostOtherAgents()
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
        if (groupA.Size() <= groupB.Size())
            return new IndependenceDetectionConflict(groupA, groupB, time);
        else
            return new IndependenceDetectionConflict(groupB, groupA, time);  // This way when attempting to resolve the conflict, the smaller group would be tried first
    }

    private IndependenceDetectionConflict ChooseConflictOfMostConflictingAndSmallestResultingGroup()
    {
        Dictionary<int, int> groupSizes = this.allGroups.ToDictionary(group => group.groupNum, group => group.Size());
        double maxScore = -1;
        int minTime = int.MaxValue;
        IndependenceDetectionAgentsGroup groupA = null;  // The must be at least one conflict
        int groupRepB = -1;  // The must be at least one conflict
        foreach (var group in this.allGroups)
        {
            int size = group.Size();
            foreach (var key in this.conflictCountsPerGroup[group.groupNum].Keys)
            {
                if (key < group.groupNum)
                    continue;  // Already checked in the reverse order
                int resultingGroupSize = groupSizes[key] + size;
                int numGroupsTheGroupsWereInConflictWith = this.countsOfGroupsThatConflict[group.groupNum] /*- 1*/ + this.countsOfGroupsThatConflict[key] /*- 1*/;
                double score = numGroupsTheGroupsWereInConflictWith / ((double)(1 << (resultingGroupSize - 1)));
                if (score > maxScore ||
                    ((score == maxScore) && (this.conflictTimesPerGroup[group.groupNum][key][0] < minTime)))  // Just to be closer to the earliest conflict strategy
                {
                    maxScore = score;
                    minTime = this.conflictTimesPerGroup[group.groupNum][key][0];
                    groupA = group;
                    groupRepB = key;
                }
            }
        }

        int time = this.conflictTimesPerGroup[groupA.groupNum][groupRepB][0];  // Choosing the earliest conflict between them - the choice doesn't matter for ID, but this is consistent with CBS' strategy

        IndependenceDetectionAgentsGroup groupB = null;
        foreach (var group in this.allGroups)
        {
            if (group.groupNum == groupRepB)
            {
                groupB = group;
                break;
            }
        }
        if (groupA.Size() <= groupB.Size())
            return new IndependenceDetectionConflict(groupA, groupB, time);
        else
            return new IndependenceDetectionConflict(groupB, groupA, time);  // This way when attempting to resolve the conflict, the smaller group would be tried first
    }

    /// <summary>
    /// Also prefers earliest time, all other things being equal, to be closer to the original strategy when possible
    /// </summary>
    /// <returns></returns>
    private IndependenceDetectionConflict ChooseConflictOfLeastConflictingLargestResultingGroup()
    {
        Dictionary<int, int> groupSizes = this.allGroups.ToDictionary(group => group.groupNum, group => group.Size());
        int maxResultingGroupSize = -1;
        int minGroupsTheyWereInConflictWith = int.MaxValue;
        int minTime = int.MaxValue;
        IndependenceDetectionAgentsGroup groupA = null;  // The must be at least one conflict
        int groupRepB = -1;  // There must be at least one conflict
        foreach (var group in this.allGroups)
        {
            int size = group.Size();
            foreach (var key in this.conflictCountsPerGroup[group.groupNum].Keys)
            {
                if (key < group.groupNum)
                    continue;  // Already checked in the reverse order
                int resultingGroupSize = groupSizes[key] + size;
                int numGroupsTheGroupsWereInConflictWith = this.countsOfGroupsThatConflict[group.groupNum] /*- 1*/ + this.countsOfGroupsThatConflict[key] /*- 1*/;
                if (resultingGroupSize > maxResultingGroupSize ||
                    ((resultingGroupSize == maxResultingGroupSize) && (numGroupsTheGroupsWereInConflictWith < minGroupsTheyWereInConflictWith)) ||
                    ((resultingGroupSize == maxResultingGroupSize) && (numGroupsTheGroupsWereInConflictWith == minGroupsTheyWereInConflictWith) && (this.conflictTimesPerGroup[group.groupNum][key][0] < minTime)))  // Just to be closer to the earliest conflict strategy
                {
                    maxResultingGroupSize = resultingGroupSize;
                    minGroupsTheyWereInConflictWith = numGroupsTheGroupsWereInConflictWith;
                    minTime = this.conflictTimesPerGroup[group.groupNum][key][0];
                    groupA = group;
                    groupRepB = key;
                }
            }
        }

        int time = this.conflictTimesPerGroup[groupA.groupNum][groupRepB][0];  // Choosing the earliest conflict between them - the choice doesn't matter for ID, but this is consistent with CBS' strategy

        IndependenceDetectionAgentsGroup groupB = null;
        foreach (var group in this.allGroups)
        {
            if (group.groupNum == groupRepB)
            {
                groupB = group;
                break;
            }
        }
        if (groupA.Size() <= groupB.Size())
            return new IndependenceDetectionConflict(groupA, groupB, time);
        else
            return new IndependenceDetectionConflict(groupB, groupA, time);  // This way when attempting to resolve the conflict, the smaller group would be tried first
    }

    /// <summary>
    /// Also prefers earliest time, all other things being equal, to be closer to the original strategy when possible
    /// </summary>
    /// <returns></returns>
    private IndependenceDetectionConflict ChooseConflictOfMostConflictingSmallestResultingGroup()
    {
        Dictionary<int, int> groupSizes = this.allGroups.ToDictionary(group => group.groupNum, group => group.Size());
        int minResultingGroupSize = this.instance.agents.Length + 1;
        int maxGroupsTheyWereInConflictWith = -1;
        int minTime = int.MaxValue;
        IndependenceDetectionAgentsGroup groupA = null;  // The must be at least one conflict
        int groupRepB = -1;  // The must be at least one conflict
        foreach (var group in this.allGroups)
        {
            int size = group.Size();
            foreach (var key in this.conflictCountsPerGroup[group.groupNum].Keys)
            {
                if (key < group.groupNum)
                    continue;  // Already checked in the reverse order
                int resultingGroupSize = groupSizes[key] + size;
                int numGroupsTheGroupsWereInConflictWith = this.countsOfGroupsThatConflict[group.groupNum] /*- 1*/ + this.countsOfGroupsThatConflict[key] /*- 1*/;
                if (resultingGroupSize < minResultingGroupSize ||
                    ((resultingGroupSize == minResultingGroupSize) && (numGroupsTheGroupsWereInConflictWith > maxGroupsTheyWereInConflictWith)) ||
                    ((resultingGroupSize == minResultingGroupSize) && (numGroupsTheGroupsWereInConflictWith == maxGroupsTheyWereInConflictWith) && (this.conflictTimesPerGroup[group.groupNum][key][0] < minTime)))  // Just to be closer to the earliest conflict strategy
                {
                    minResultingGroupSize = resultingGroupSize;
                    maxGroupsTheyWereInConflictWith = numGroupsTheGroupsWereInConflictWith;
                    minTime = this.conflictTimesPerGroup[group.groupNum][key][0];
                    groupA = group;
                    groupRepB = key;
                }
            }
        }

        int time = this.conflictTimesPerGroup[groupA.groupNum][groupRepB][0];  // Choosing the earliest conflict between them - the choice doesn't matter for ID, but this is consistent with CBS' strategy

        IndependenceDetectionAgentsGroup groupB = null;
        foreach (var group in this.allGroups)
        {
            if (group.groupNum == groupRepB)
            {
                groupB = group;
                break;
            }
        }
        if (groupA.Size() <= groupB.Size())
            return new IndependenceDetectionConflict(groupA, groupB, time);
        else
            return new IndependenceDetectionConflict(groupB, groupA, time);  // This way when attempting to resolve the conflict, the smaller group would be tried first
    }

    /// <summary>
    /// Returns the earliest conflict between the groups
    /// </summary>
    /// <returns></returns>
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
                if (otherGroupNumAndConflictTimes.Value[0] < time)  // Conflict times are sorted, so only the first one needs to be checked
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
        if (groupA.Size() <= groupB.Size())
            return new IndependenceDetectionConflict(groupA, groupB, time);
        else
            return new IndependenceDetectionConflict(groupB, groupA, time);  // This way when attempting to resolve the conflict, the smaller group would be tried first
    }

    /// <summary>
    /// For SimpleID. Doesn't use the conflictCount and conflictTimes variables.
    /// TODO: Merge duplication with FindFirstConflict.
    /// </summary>
    /// <returns></returns>
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
                
            // Solve composite group with the subsolver
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
                Debug.WriteLine($"Total cost: {this.allGroups.Sum(group => group.solutionCost)}");
                Debug.WriteLine($"{this.allGroups.Count} groups: {String.Join(", ", this.allGroups)}");

                Debug.Write("Group single agent costs: ");
                foreach (var group in this.allGroups)
                {
                    Debug.Write($"{{{String.Join(" ", group.GetCosts())}}}, ");
                }
                Debug.WriteLine("");
            }

            IndependenceDetectionConflict conflict = ChooseConflict();
            // If there are no conflicts - can return the current plan
            if (conflict == null)
                break;

            if (this.debug)
            {
                for (int j = 0; j < this.conflictTimesPerGroup.Length; j++)
                {
                    if (this.conflictTimesPerGroup[j] != null && this.conflictTimesPerGroup[j].Count != 0)
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
                plan.PrintPlanIfShort();
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
                            //Debug.WriteLine($"Found an alternative path that avoids the conflict for {conflict.group1}: {conflict.group1.GetPlan()}");
                            Debug.WriteLine($"Found an alternative path that avoids the conflict for {conflict.group1}");
                        }

                        UpdateConflictCounts(conflict.group1);
                        conflict.group1.addGroupToCAT(conflictAvoidanceTable);
                        conflictAvoidanceTable.agentConflictCounts[conflict.group1.groupNum] = conflictCountsPerGroup[conflict.group1.groupNum].Count;
                        ++resolutionSuccesses;

                        continue;
                    }
                    else
                    {
                        if (conflict.group1.solutionCost < 0)  // ReplanUnderConstraints reverts to the old cost if there was simply no solution
                        {
                            totalCost = conflict.group1.solutionCost;
                            return false;
                        }

                        conflict.group1.addGroupToCAT(conflictAvoidanceTable);

                        if (this.debug)
                        {
                            Debug.WriteLine($"Couldn't find an alternative path that avoids the conflict for {conflict.group1}");
                        }
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
                        conflictAvoidanceTable.agentConflictCounts[conflict.group2.groupNum] = conflictCountsPerGroup[conflict.group2.groupNum].Count;
                        ++resolutionSuccesses;

                        continue;
                    }
                    else
                    {
                        if (conflict.group2.solutionCost < 0)  // ReplanUnderConstraints reverts to the old cost if there was simply no solution
                        {
                            totalCost = conflict.group2.solutionCost;  // To propagate special costs
                            return false;
                        }
                            
                        conflict.group2.addGroupToCAT(conflictAvoidanceTable);
                            
                        if (this.debug)
                        {
                            Debug.WriteLine($"Couldn't find an alternative path that avoids the conflict for {conflict.group2}");
                        }
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
            conflictAvoidanceTable.agentSizes.Remove(conflict.group1.groupNum);
            conflictAvoidanceTable.agentSizes.Remove(conflict.group2.groupNum);
            conflictAvoidanceTable.agentConflictCounts.Remove(conflict.group1.groupNum);
            conflictAvoidanceTable.agentConflictCounts.Remove(conflict.group2.groupNum);
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
                this.totalCost = compositeGroup.solutionCost;  // To propagate special costs

                allGroups.AddFirst(compositeGroup);  // Important for printing the statistics
                return false;
            }

            UpdateConflictCounts(compositeGroup);

            // Add the new group to conflict avoidance table
            compositeGroup.addGroupToCAT(conflictAvoidanceTable);
            conflictAvoidanceTable.agentSizes[compositeGroup.groupNum] = compositeGroup.Size();
            conflictAvoidanceTable.agentConflictCounts[compositeGroup.groupNum] = this.conflictCountsPerGroup[compositeGroup.groupNum].Count;
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
    /// Run Standley's ID framework with the given subsolver
    /// </summary>
    /// <returns>true if optimal solution has been found</returns>
    public bool Solve()
    {
        // TODO: Add a SolveGiven method that takes the state just before the "if (this.simple == false)" line in this method and solves from it.
        //       Add a PreSolve method that prepares that state and also gives for each agent the accummulated time it took to presolve up to it.
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
                //if (pair.Key < i)  // Just an optimization. Would also be correct without this check.
                                     // Not if group order is funky...
                {
                    this.conflictCountsPerGroup[pair.Key][i] = pair.Value; // Collisions are symmetrical, and agent "key" didn't see the route for agent "i" when planning.
                    this.conflictTimesPerGroup[pair.Key][i] = this.conflictTimesPerGroup[i][pair.Key];
                }
            }
        }

        // Populate the CAT's agentConflictCounts
        foreach (var group in this.allGroups)
            this.conflictAvoidanceTable.agentConflictCounts[group.groupNum] = group.conflictCounts.Count;

        CountConflicts();

        if (this.simple == false)
            solved = this.ImprovedID(runner);
        else
            solved = this.SimpleID(runner);  // TODO: it doesn't manage all the stats. Just add flags to ImprovedID instead
        // Record found solution
        if (solved == true)
        {
            // Store solution details
            if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                this.totalCost = this.allGroups.Sum(group => group.solutionCost);
            else if (Constants.costFunction == Constants.CostFunction.MAKESPAN || Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                this.totalCost = this.allGroups.Max(group => group.solutionCost);
            else
                throw new Exception($"Unexpected cost function {Constants.costFunction}");
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
