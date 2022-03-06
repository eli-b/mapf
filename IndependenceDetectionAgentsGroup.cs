using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using ExtensionMethods;

namespace mapf;

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
    private IndependenceDetection id;
    private Plan plan;
    private int[] singleCosts;
    public Dictionary<int, int> conflictCounts;
    public Dictionary<int, List<int>> conflictTimes;

    public IndependenceDetectionAgentsGroup(ProblemInstance instance, AgentState[] allAgentsState,
                                            IIndependenceDetectionSolver singleAgentSolver, IIndependenceDetectionSolver groupSolver,
                                            IndependenceDetection id)
    {
        this.allAgentsState = allAgentsState;
        this.instance = instance.Subproblem(allAgentsState);
        this.singleAgentSolver = singleAgentSolver;
        this.groupSolver = groupSolver;
        this.id = id;
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
        if (this.id.provideGroupCostsToSolver)
            relevantSolver.Setup(this.instance, runner, CAT, group1Cost, group2Cost, group1Size);
        else  // For experiments only
            relevantSolver.Setup(this.instance, runner, CAT, 0, 0, 0);
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

        return new IndependenceDetectionAgentsGroup(this.instance, joinedAgentStates, this.singleAgentSolver, this.groupSolver, this.id);
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
            if (this.solutionCost == (int)Constants.SpecialCosts.NO_SOLUTION_COST)  // No solution is an expected option, not a failure, revert to the old cost
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
