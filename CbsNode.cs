using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics;

using ExtensionMethods;
using System.Collections;

namespace mapf;

[DebuggerDisplay("hash = {GetHashCode()}, f = {f}, g = {g}, h = {h}")]
public class CbsNode : IComparable<IBinaryHeapItem>, IBinaryHeapItem, IHeuristicSearchNode
{
    public int G { set; get; }  // Value depends on Constants.costFunction and Constants.sumOfCostsVariant, Sum of agent makespans until they reach their goal

    public int H { get; set; }

    public int HBonus { get; set; }

    /// <summary>
    /// The size of the minimum vertex cover of the node's cardinal conflict graph.
    /// Needs to be saved separately from h to allow speeding up the computation of the heuristic
    /// of the children.
    /// </summary>
    public int MinimumVertexCover { get; set; }
    public BitArray NewPlans { get; }
    public SinglePlan[] SingleAgentPlans { get; private set; }
    public int[] SingleAgentCosts { get; private set; }
    /// <summary>
    /// A lower estimate of the number of operations (replanning or merging) needed to solve the node.
    /// Used for tie-breaking.
    /// </summary>
    public int MinOpsToSolve { get; private set; }
    /// <summary>
    /// For each agent in the problem instance, saves the number of agents from the problem instance that it conflicts with.
    /// Used for choosing the next conflict to resolve by replanning/merging/shuffling, and for tie-breaking.
    /// </summary>
    private int[] _countsOfInternalAgentsThatConflict;
    /// <summary>
    /// Counts the number of external agents this node conflicts with.
    /// Used for tie-breaking.
    /// </summary>
    private int _totalExternalAgentsThatConflict;
    /// <summary>
    /// Used for tie-breaking.
    /// </summary>
    public int TotalConflictsWithExternalAgents { get; private set; }
    /// <summary>
    /// For each agent in the problem instance, maps agent _nums_ it conflicts with, internal or external,
    /// to the number of conflicts betweem them.
    /// Used for book-keeping to maintain countsOfInternalAgentsThatConflict,
    /// totalExternalAgentsThatConflict and minOpsToSolve, and other counts.
    /// </summary>
    public Dictionary<int, int>[] ConflictCountsPerAgent { get; private set; }
    /// <summary>
    /// For each agent in the problem instance, maps agent _nums_ of agents it collides with to the time of their first collision.
    /// </summary>
    public Dictionary<int, List<int>>[] ConflictTimesPerAgent { get; private set; }
    private int _binaryHeapIndex;
    public CbsConflict Conflict { get; private set; }
    private CbsConstraint _constraint;
    /// <summary>
    /// Forcing an agent to be at a certain place at a certain time
    /// </summary>
    private CbsConstraint _mustConstraint;
    public CbsNode Prev { get; }
    private ushort _depth;
    public ushort[] AgentsGroupAssignment { get; }
    public ushort ReplanSize { get; private set; }
    public enum ExpansionState: byte
    {
        NOT_EXPANDED = 0,
        DEFERRED,
        EXPANDED
    }
    /// <summary>
    /// For partial expansion
    /// </summary>
    public ExpansionState AgentAExpansion { get; set; }
    /// <summary>
    /// For partial expansion
    /// </summary>
    public ExpansionState AgentBExpansion { get; set; }
    private ICbsSolver _solver;
    private ICbsSolver _singleAgentSolver;
    public CBS CBS { get; private set; }
    public Dictionary<int, int> AgentNumToIndex { get; private set; }
    public bool ParentAlreadyLookedAheadOf { get; set; }
    /// <summary>
    /// For tie-breaking
    /// </summary>
    public int TotalInternalAgentsThatConflict { get; private set; }
    /// <summary>
    /// For tie-breaking
    /// </summary>
    private int _largerConflictingGroupSize; // TODO: is it really used?
    /// <summary>
    /// For tie-breaking
    /// </summary>
    public int TotalConflictsBetweenInternalAgents { get; private set; }

    /// <summary>
    /// For each agent, map each level (timestep) of its mdd to a narrowness degree.
    /// Non-narrow levels are omitted.
    /// </summary>
    public Dictionary<int, MDD.LevelNarrowness>[] MDDNarrownessValues { get; }

    /// <summary>
    /// FIXME: We're currently saving both the MDDs and their much smaller narrowness values in
    /// order to have a fair comparison with the past
    /// </summary>
    private MDD[] _mdds;

    /// <summary>
    /// Root node constructor
    /// </summary>
    /// <param name="numberOfAgents"></param>
    /// <param name="solver"></param>
    /// <param name="singleAgentSolver"></param>
    /// <param name="cbs"></param>
    /// <param name="agentsGroupAssignment"></param>
    public CbsNode(int numberOfAgents, ICbsSolver solver, ICbsSolver singleAgentSolver,
        CBS cbs, ushort[] agentsGroupAssignment = null, ISet<CbsConstraint> externalConstraints = null, ISet<CbsConstraint> externalPositiveConstraints = null)
    {
        CBS = cbs;
        SingleAgentPlans = new SinglePlan[numberOfAgents];
        NewPlans = new BitArray(numberOfAgents);
        SingleAgentCosts = new int[numberOfAgents];
        MDDNarrownessValues = new Dictionary<int, MDD.LevelNarrowness>[numberOfAgents];
        _mdds = new MDD[numberOfAgents];
        _countsOfInternalAgentsThatConflict = new int[numberOfAgents];
        ConflictCountsPerAgent = new Dictionary<int, int>[numberOfAgents]; // Populated after Solve()
        ConflictTimesPerAgent = new Dictionary<int, List<int>>[numberOfAgents]; // Populated after Solve()
        if (agentsGroupAssignment == null)
        {
            AgentsGroupAssignment = new ushort[numberOfAgents];
            for (ushort i = 0; i < numberOfAgents; i++)
                AgentsGroupAssignment[i] = i;
        }
        else
            AgentsGroupAssignment = [.. agentsGroupAssignment];
        AgentNumToIndex = [];
        for (int i = 0; i < numberOfAgents; i++)
        {
            AgentNumToIndex[CBS.GetProblemInstance().agents[i].agent.agentNum] = i;
        }
        _depth = 0;
        ReplanSize = 1;
        AgentAExpansion = ExpansionState.NOT_EXPANDED;
        AgentBExpansion = ExpansionState.NOT_EXPANDED;
        Prev = null;
        _constraint = null;
        _solver = solver;
        _singleAgentSolver = singleAgentSolver;
        MinimumVertexCover = (int) ConflictGraph.MinVertexCover.NOT_SET;
    }

    /// <summary>
    /// Child from branch action constructor
    /// </summary>
    /// <param name="parent"></param>
    /// <param name="newConstraint"></param>
    /// <param name="agentToReplan"></param>
    public CbsNode(CbsNode parent, CbsConstraint newConstraint, int agentToReplan)
    {
        CBS = parent.CBS;
        SingleAgentPlans = [.. parent.SingleAgentPlans];
        NewPlans = new BitArray(SingleAgentPlans.Length);
        SingleAgentCosts = [.. parent.SingleAgentCosts];
        _mdds = [.. parent._mdds];
        MDDNarrownessValues = [.. parent.MDDNarrownessValues];

        // Adapt the MDDs for the agent to replan, if possible
        // The cost may increase, so the old MDD might not be relevant anymore.
        if (_mdds[agentToReplan] != null &&
            _mdds[agentToReplan].levels.Length - 1 > newConstraint.time &&
            (MDDNarrownessValues[agentToReplan].ContainsKey(newConstraint.time) == false ||
                (MDDNarrownessValues[agentToReplan][newConstraint.time] == MDD.LevelNarrowness.ONE_LOCATION_MULTIPLE_DIRECTIONS &&
                newConstraint.move.Direction != Direction.NO_DIRECTION)))
        {
            // We have an MDD and same cost can still be achieved - adapt the existing MDD
            double startTime = CBS._stopwatch.ElapsedMilliseconds;
            _mdds[agentToReplan] = new MDD(_mdds[agentToReplan], newConstraint);
            MDDNarrownessValues[agentToReplan] = _mdds[agentToReplan].getLevelNarrownessValues();
            double endTime = CBS._stopwatch.ElapsedMilliseconds;
            CBS.MDDsAdapted++;
            CBS.TimeBuildingMdds += endTime - startTime;
        }
        else
        {
            _mdds[agentToReplan] = null;
            MDDNarrownessValues[agentToReplan] = null;
        }

        _countsOfInternalAgentsThatConflict = parent._countsOfInternalAgentsThatConflict.ToArray();
        ConflictCountsPerAgent = new Dictionary<int, int>[parent.ConflictCountsPerAgent.Length];
        for (int i = 0; i < ConflictCountsPerAgent.Length; i++)
            ConflictCountsPerAgent[i] = new Dictionary<int, int>(parent.ConflictCountsPerAgent[i]); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
        ConflictTimesPerAgent = new Dictionary<int, List<int>>[parent.ConflictTimesPerAgent.Length];
        for (int i = 0; i < ConflictTimesPerAgent.Length; i++)
        {
            ConflictTimesPerAgent[i] = []; // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
            foreach (var kvp in parent.ConflictTimesPerAgent[i])
                ConflictTimesPerAgent[i][kvp.Key] = [.. kvp.Value];
        }
        AgentsGroupAssignment = parent.AgentsGroupAssignment.ToArray();
        
        for (int i = 0; i < SingleAgentPlans.Length; i++)
        {
            NewPlans[i] = false;
        }
        ISet<int> group = GetGroup(agentToReplan);
        foreach (int i in group)
            NewPlans[i] = true;
        
        AgentNumToIndex = parent.AgentNumToIndex;
        Prev = parent;
        _constraint = newConstraint;
        _depth = (ushort)(Prev._depth + 1);
        AgentAExpansion = ExpansionState.NOT_EXPANDED;
        AgentBExpansion = ExpansionState.NOT_EXPANDED;
        ReplanSize = 1;
        _solver = parent._solver;
        _singleAgentSolver = parent._singleAgentSolver;
        MinimumVertexCover = (int) ConflictGraph.MinVertexCover.NOT_SET;
    }

    /// <summary>
    /// Child from merge action constructor. FIXME: Code dup with previous constructor.
    /// </summary>
    /// <param name="parent"></param>
    /// <param name="mergeGroupA"></param>
    /// <param name="mergeGroupB"></param>
    public CbsNode(CbsNode parent, int mergeGroupA, int mergeGroupB)
    {
        SingleAgentPlans = [.. parent.SingleAgentPlans];
        NewPlans = new BitArray(SingleAgentPlans.Length);
        SingleAgentCosts = [.. parent.SingleAgentCosts];
        _mdds = [.. parent._mdds];
        MDDNarrownessValues = [.. parent.MDDNarrownessValues];  // No new constraint was added so all of the parent's MDDs are valid
        _countsOfInternalAgentsThatConflict = [.. parent._countsOfInternalAgentsThatConflict];
        ConflictCountsPerAgent = new Dictionary<int, int>[parent.ConflictCountsPerAgent.Length];
        for (int i = 0; i < ConflictCountsPerAgent.Length; i++)
            ConflictCountsPerAgent[i] = new Dictionary<int, int>(parent.ConflictCountsPerAgent[i]); // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
        ConflictTimesPerAgent = new Dictionary<int, List<int>>[parent.ConflictTimesPerAgent.Length];
        for (int i = 0; i < ConflictTimesPerAgent.Length; i++)
        {
            ConflictTimesPerAgent[i] = []; // Need a separate copy because unlike plans, the conflict counts for agents that aren't replanned do change.
            foreach (var kvp in parent.ConflictTimesPerAgent[i])
                ConflictTimesPerAgent[i][kvp.Key] = [.. kvp.Value];
        }
        AgentsGroupAssignment = parent.AgentsGroupAssignment.ToArray();
        AgentNumToIndex = parent.AgentNumToIndex;
        Prev = parent;
        _constraint = null;
        _depth = (ushort)(Prev._depth + 1);
        AgentAExpansion = ExpansionState.NOT_EXPANDED;
        AgentBExpansion = ExpansionState.NOT_EXPANDED;
        ReplanSize = 1;
        _solver = parent._solver;
        _singleAgentSolver = parent._singleAgentSolver;
        CBS = parent.CBS;
            
        MergeGroups(mergeGroupA, mergeGroupB);

        for (int i = 0; i < SingleAgentPlans.Length; i++)
        {
            NewPlans[i] = false;
        }
        ISet<int> mergedGroup = (mergeGroupA < mergeGroupB) ? GetGroup(mergeGroupA) : GetGroup(mergeGroupB);
        foreach (int i in mergedGroup)
            NewPlans[i] = true;

        MinimumVertexCover = (int) ConflictGraph.MinVertexCover.NOT_SET;
    }

    /// <summary>
    /// Total cost + heuristic estimate
    /// </summary>
    public int F => G + H;

    public int GetTargetH(int f) => f - G;

    /// <summary>
    /// Solves the entire node - finds a plan for every agent group.
    /// This method is only called for the root of the constraint tree.
    /// </summary>
    /// <returns>Whether solving was successful. Solving fails if a timeout occurs.</returns>
    public bool Solve(int depthToReplan)
    {
        G = 0;
        ProblemInstance problem = CBS.GetProblemInstance();
        for (int i = 0; i < NewPlans.Length; i++)
        {
            NewPlans[i] = true;
        }

        ConflictAvoidanceTable internalCAT = new();
        ConflictAvoidanceTable CAT = internalCAT;
        if (CBS.ExternalCAT != null)
        {
            CAT = new CAT_U();
            ((CAT_U)CAT).Join(CBS.ExternalCAT);
            ((CAT_U)CAT).Join(internalCAT);
        }

        HashSet<CbsConstraint> newConstraints = GetConstraints(); // Probably empty as this is probably the root of the CT.
        ISet<CbsConstraint> constraints = newConstraints;
        if (CBS.ExternalConstraints != null)
        {
            constraints = new HashSet_U<CbsConstraint>();
            ((HashSet_U<CbsConstraint>)constraints).Join(CBS.ExternalConstraints);
            ((HashSet_U<CbsConstraint>)constraints).Join(newConstraints);
        }

            
        ISet<CbsConstraint> positiveConstraints = null;
        Dictionary<int,int> agentsWithPositiveConstraints = null;
        HashSet<CbsConstraint> newPositiveConstraints = null;
        if (CBS.DoMalte)
            newPositiveConstraints = GetPositiveConstraints();
        if (CBS.ExternalPositiveConstraints != null && CBS.ExternalPositiveConstraints.Count != 0 &&
            newPositiveConstraints != null && newPositiveConstraints.Count != 0)
        {
            positiveConstraints = new HashSet_U<CbsConstraint>();
            ((HashSet_U<CbsConstraint>)positiveConstraints).Join(CBS.ExternalPositiveConstraints);
            ((HashSet_U<CbsConstraint>)positiveConstraints).Join(newPositiveConstraints);
        }
        else if (CBS.ExternalPositiveConstraints != null && CBS.ExternalPositiveConstraints.Count != 0)
            positiveConstraints = CBS.ExternalPositiveConstraints;
        else if (newPositiveConstraints != null && newPositiveConstraints.Count != 0)
            positiveConstraints = newPositiveConstraints;

        if (positiveConstraints != null)
            agentsWithPositiveConstraints = positiveConstraints.Select<CbsConstraint, int>(constraint => constraint.agentNum).Distinct().ToDictionary(x => x); // ToDictionary because there's no ToSet...

        Dictionary<int, int> agentsWithConstraints = null;
        if (constraints.Count != 0)
        {
            int maxConstraintTimeStep = constraints.Max(constraint => constraint.time);
            depthToReplan = Math.Max(depthToReplan, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
            agentsWithConstraints = constraints.Select<CbsConstraint, int>(constraint => constraint.agentNum).Distinct().ToDictionary(x => x); // ToDictionary because there's no ToSet...
        }
        // This mechanism of adding the constraints to the possibly pre-existing constraints allows having
        // layers of CBS/ID solvers, each one adding its own constraints and respecting those of the solvers above it.

        // Find all the agents groups:
        List<AgentState>[] subGroups = new List<AgentState>[problem.agents.Length];
        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
        {
            if (subGroups[AgentsGroupAssignment[i]] == null)
                subGroups[AgentsGroupAssignment[i]] = [ problem.agents[i] ];
            else
                subGroups[AgentsGroupAssignment[i]].Add(problem.agents[i]);
        }

        bool success = true;

        for (int i = 0; i < subGroups.Length; i++)
        {
            if (subGroups[i] == null) // This isn't the first agent in its group - we've already solved its group.
                continue;
            List<AgentState> subGroup = subGroups[i];

            bool agentGroupHasConstraints = (agentsWithConstraints != null) && subGroup.Any<AgentState>(state => agentsWithConstraints.ContainsKey(state.agent.agentNum));
            bool agentGroupHasMustConstraints = (agentsWithPositiveConstraints != null) && subGroup.Any<AgentState>(state => agentsWithPositiveConstraints.ContainsKey(state.agent.agentNum));

            // Solve for a single agent:
            if (agentGroupHasConstraints == false  &&
                agentGroupHasMustConstraints == false &&
                subGroup.Count == 1) // No constraints on this agent. Shortcut available (that doesn't consider the CAT, though!).
            {
                SingleAgentPlans[i] = new SinglePlan(problem.agents[i]); // All moves up to starting pos, if any
                SingleAgentPlans[i].AgentNum = problem.agents[AgentsGroupAssignment[i]].agent.agentNum; // Use the group's representative
                SinglePlan optimalPlan = problem.GetSingleAgentOptimalPlan(problem.agents[i]);
                // Count conflicts:
                ConflictCountsPerAgent[i] = [];
                ConflictTimesPerAgent[i] = [];
                foreach (var move in optimalPlan.LocationAtTimes)
                {
                    var timedMove = (TimedMove)move;  // GetSingleAgentOptimalPlan actually creates a plan with TimedMove instances
                    timedMove.IncrementConflictCounts(CAT, ConflictCountsPerAgent[i], ConflictTimesPerAgent[i]);
                }
                SingleAgentPlans[i].ContinueWith(optimalPlan);
                SingleAgentCosts[i] = problem.agents[i].g + problem.GetSingleAgentOptimalCost(problem.agents[i]);
                if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                {
                    G += (ushort)SingleAgentCosts[i];
                }
                else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                    Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                {
                    G = Math.Max(G, (ushort)SingleAgentCosts[i]);
                }
                else
                    throw new NotImplementedException($"Unsupported cost function {Constants.costFunction}");

                UpdateAtGoalConflictCounts(i, CAT);
            }
            else
            {
                success = Replan(i, depthToReplan, subGroup, CAT, constraints, positiveConstraints);

                if (!success) // Usually means a timeout occured.
                    break;
            }

            // Add the group's plan to the internal CAT. In the case we use ready-made plans from the heuristic, this is still needed to allow us to track conflicts.
            foreach (AgentState agentState in subGroup)
            {
                internalCAT.AddPlan(SingleAgentPlans[AgentNumToIndex[agentState.agent.agentNum]]);
            }
        }

        if (!success)
            return false;

        // Update conflict counts: All agents but the last saw an incomplete CAT. Update counts backwards.
        for (int i = ConflictCountsPerAgent.Length - 1; i >= 0; i--)
        {
            foreach (KeyValuePair<int, int> pair in ConflictCountsPerAgent[i])
            {
                if (AgentNumToIndex.ContainsKey(pair.Key) && // An internal conflict, rather than external
                    AgentNumToIndex[pair.Key] < i)                                 // Just an optimization. Would also be correct without this check.
                {
                    ConflictCountsPerAgent[AgentNumToIndex[pair.Key]] // Yes, index here, num there
                        [problem.agents[i].agent.agentNum] = pair.Value; // Collisions are symmetrical, and agent "key" didn't see the route for agent "i" when planning.
                    ConflictTimesPerAgent[AgentNumToIndex[pair.Key]]
                        [problem.agents[i].agent.agentNum] = ConflictTimesPerAgent[i][pair.Key];
                }
            }
        }

        CountConflicts();

        CalcMinOpsToSolve();

        isGoal = _countsOfInternalAgentsThatConflict.All(i => i == 0);

        return true;
    }

    /// <summary>
    /// Replan for a given agent (when constraints for that agent have changed, or its group was enlarged).
    /// </summary>
    /// <param name="agentToReplan"></param>
    /// <param name="minPathTimeStep"></param>
    /// <param name="subGroup">If given, assume CAT, constraints and positiveConstraints are all populated too</param>
    /// <param name="CAT"></param>
    /// <param name="constraints"></param>
    /// <param name="positiveConstraints"></param>
    /// <param name="minPathCost"></param>
    /// <param name="maxPathCost"></param>
    /// <returns>Whether a path was successfully found</returns>
    public bool Replan(int agentToReplan, int minPathTimeStep,
                        List<AgentState> subGroup = null,
                        ConflictAvoidanceTable CAT = null,
                        ISet<CbsConstraint> constraints = null, ISet<CbsConstraint> positiveConstraints = null,
                        int minPathCost = -1, int maxPathCost = int.MaxValue)
    {

        ConflictAvoidanceTable internalCAT = null; // To quiet the compiler
        ProblemInstance problem = CBS.GetProblemInstance();
        int groupNum = AgentsGroupAssignment[agentToReplan];
        bool underSolve = true;

        if (subGroup == null)
        {
            underSolve = false;
            // Construct the subgroup of agents that are of the same group as agentForReplan,
            // and add the plans of all other agents to CAT
            internalCAT = new ConflictAvoidanceTable();
            subGroup = [];
            for (int i = 0; i < AgentsGroupAssignment.Length; i++)
            {
                if (AgentsGroupAssignment[i] == groupNum)
                    subGroup.Add(problem.agents[i]);
                else
                    internalCAT.AddPlan(SingleAgentPlans[i]);
            }
            if (CBS.ExternalCAT != null)
            {
                CAT = new CAT_U();
                ((CAT_U)CAT).Join(CBS.ExternalCAT);
                ((CAT_U)CAT).Join(internalCAT);
            }
            else
                CAT = internalCAT;
                

            HashSet<CbsConstraint> newConstraints = GetConstraints();
            if (CBS.ExternalConstraints != null && CBS.ExternalConstraints.Count != 0)
            {
                constraints = new HashSet_U<CbsConstraint>();
                ((HashSet_U<CbsConstraint>)constraints).Join(CBS.ExternalConstraints);
                ((HashSet_U<CbsConstraint>)constraints).Join(newConstraints);
            }
            else
                constraints = newConstraints;


            HashSet<CbsConstraint> newPositiveConstraints = null;
            if (CBS.DoMalte)
                newPositiveConstraints = GetPositiveConstraints();
            if (CBS.ExternalPositiveConstraints != null && CBS.ExternalPositiveConstraints.Count != 0 &&
                newPositiveConstraints != null && newPositiveConstraints.Count != 0)
            {
                positiveConstraints = new HashSet_U<CbsConstraint>();
                ((HashSet_U<CbsConstraint>)positiveConstraints).Join(CBS.ExternalPositiveConstraints);
                ((HashSet_U<CbsConstraint>)positiveConstraints).Join(newPositiveConstraints);
            }
            else if (CBS.ExternalPositiveConstraints != null && CBS.ExternalPositiveConstraints.Count != 0)
                positiveConstraints = CBS.ExternalPositiveConstraints;
            else if (newPositiveConstraints != null && newPositiveConstraints.Count != 0)
                positiveConstraints = newPositiveConstraints;
        }

        ReplanSize = (ushort)subGroup.Count;

        ICbsSolver relevantSolver = _solver;
        if (subGroup.Count == 1)
            relevantSolver = _singleAgentSolver;

        ProblemInstance subProblem = problem.Subproblem(subGroup.ToArray());

        Dictionary<int, int> subGroupAgentNums = subGroup.Select(state => state.agent.agentNum).ToDictionary(num => num); // No need to call Distinct(). Each agent appears at most once

        IEnumerable<CbsConstraint> myConstraints = constraints.Where(constraint => subGroupAgentNums.ContainsKey(constraint.agentNum)); // TODO: Consider passing only myConstraints to the low level to speed things up.
        if (myConstraints.Count() != 0)
        {
            int maxConstraintTimeStep = myConstraints.Max(constraint => constraint.time);
            minPathTimeStep = Math.Max(minPathTimeStep, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
        }
        if (positiveConstraints != null)
        {
            IEnumerable<CbsConstraint> myMustConstraints = positiveConstraints.Where(constraint => subGroupAgentNums.ContainsKey(constraint.agentNum));
            if (myMustConstraints.Count() != 0)
            {
                int maxMustConstraintTimeStep = myMustConstraints.Max(constraint => constraint.time);
                minPathTimeStep = Math.Max(minPathTimeStep, maxMustConstraintTimeStep); // Give all must constraints a chance to affect the plan
            }
        }

        MDD mdd = null;
        if (CBS.ReplanSameCostWithMdd)
            mdd = _mdds[agentToReplan];

        double startTime = CBS._stopwatch.ElapsedMilliseconds;
        relevantSolver.Setup(subProblem, minPathTimeStep, CBS._stopwatch, CAT, constraints, positiveConstraints,
                                minPathCost, maxPathCost, mdd);
        bool solved = relevantSolver.Solve();
        double endTime = CBS._stopwatch.ElapsedMilliseconds;
        CBS.TimePlanningPaths += endTime - startTime;

        relevantSolver.AccumulateStatistics();
        relevantSolver.ClearStatistics();

        if (solved == false) // Usually means a timeout occured.
        {
            return false;
        }

        // Copy the SinglePlans for the solved agent group from the solver to the appropriate places in allSingleAgentPlans
        SinglePlan[] singlePlans = relevantSolver.GetSinglePlans();
        int[] singleCosts = relevantSolver.GetSingleCosts();
        Dictionary<int, int> perAgent = null;  // To quiet the compiler
        Dictionary<int, List<int>> conflictTimes = null;
        if (CAT != null)
        {
            perAgent = relevantSolver.GetExternalConflictCounts();
            conflictTimes = relevantSolver.GetConflictTimes();
        }
        else
        {
            perAgent = [];
            conflictTimes = [];
            foreach (var singlePlan in singlePlans)
            {
                foreach (var move in singlePlan.LocationAtTimes)
                {
                    var timedMove = (TimedMove)move;  // The solver actually creates a plan with TimedMove instances
                    if (CAT != null)
                        timedMove.IncrementConflictCounts(CAT, perAgent, conflictTimes);
                    else
                        timedMove.IncrementConflictCounts(internalCAT, perAgent, conflictTimes);
                }
            }
        }
        for (int i = 0; i < subGroup.Count; i++)
        {
            int agentNum = subGroup[i].agent.agentNum;
            int agentIndex = AgentNumToIndex[agentNum];
            SingleAgentPlans[agentIndex] = singlePlans[i];
            SingleAgentPlans[agentIndex].AgentNum = problem.agents[groupNum].agent.agentNum; // Use the group's representative - that's how the plans will be inserted into the CAT later too.
            SingleAgentCosts[agentIndex] = singleCosts[i];
            if (i == 0) // This is the group representative
            {
                ConflictCountsPerAgent[agentIndex] = perAgent;
                ConflictTimesPerAgent[agentIndex] = conflictTimes;
            }
            else
            {
                if (underSolve == false)
                {
                    ConflictCountsPerAgent[agentIndex].Clear(); // Don't over-count. Leave it to the group's representative.
                    ConflictTimesPerAgent[agentIndex].Clear();
                }
                else
                {
                    ConflictCountsPerAgent[agentIndex] = [];
                    ConflictTimesPerAgent[agentIndex] = [];
                }
            }
        }

        // Update conflict counts with what happens after the plan finishes
        foreach (var agentNumAndAgentNum in subGroupAgentNums)
        {
            int i = AgentNumToIndex[agentNumAndAgentNum.Key];
            if (CAT != null)
                UpdateAtGoalConflictCounts(i, CAT);
                // Can't use the null coalescing operator because it requires the operands be of the same type :(
            else
                UpdateAtGoalConflictCounts(i, internalCAT);
        }

        if (underSolve == false)
        {
            // Update conflictCountsPerAgent and conflictTimes for all agents
            int representativeAgentNum = subGroup[0].agent.agentNum;
            for (int i = 0; i < ConflictCountsPerAgent.Length; i++)
            {
                int agentNum = problem.agents[i].agent.agentNum;
                if (perAgent.ContainsKey(agentNum))
                {
                    ConflictCountsPerAgent[i][representativeAgentNum] = perAgent[agentNum];
                    ConflictTimesPerAgent[i][representativeAgentNum] = conflictTimes[agentNum];
                }
                else
                {
                    ConflictCountsPerAgent[i].Remove(representativeAgentNum);  // This part could have been done before replanning
                    ConflictTimesPerAgent[i].Remove(representativeAgentNum);  // This part could have been done before replanning
                }
            }

            CountConflicts();
            CalcMinOpsToSolve();
        }

        // Calc g
        if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
        {
            G = (ushort)Math.Max(SingleAgentCosts.Sum(), G); // Conserve g from partial 
                                                                                // expansion if it's higher
                                                                                // (only happens when shuffling a partially expanded node)
        }
        else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
            Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
        {
            G = (ushort)Math.Max(SingleAgentCosts.Max(), G); // Conserve g from partial
                                                                                // expansion if it's higher
                                                                                // (only happens when shuffling a partially expanded node)
        }
        else
            throw new NotImplementedException($"Unsupported cost function {Constants.costFunction}");

        isGoal = _countsOfInternalAgentsThatConflict.All(i => i == 0);

        return true;
    }

    public void DebugPrint()
    {
        Debug.WriteLine("");
        Debug.WriteLine("");
        var hashCode = GetHashCode();
        Debug.WriteLine($"Node hash: {hashCode}");
        var parent = Prev;
        Debug.Write("Ancestor hashes (parent to root): ");
        while (parent != null)
        {
            Debug.Write($"{parent.GetHashCode()} ");
            parent = parent.Prev;
        }
        Debug.WriteLine("");
        Debug.WriteLine($"g: {G}");
        Debug.WriteLine($"h: {H}");
        Debug.WriteLine($"Min estimated ops needed: {MinOpsToSolve}");
        Debug.WriteLine($"Expansion state: {AgentAExpansion}, {AgentBExpansion}");
        Debug.WriteLine($"Num of external agents that conflict: {_totalExternalAgentsThatConflict}");
        Debug.WriteLine($"Num of internal agents that conflict: {TotalInternalAgentsThatConflict}");
        Debug.WriteLine($"Num of conflicts between internal agents: {TotalConflictsBetweenInternalAgents}");
        Debug.WriteLine($"Node depth: {_depth}");
        List<CbsConstraint> constraints = GetConstraintsOrdered();
        Debug.WriteLine($"{constraints.Count} relevant internal constraints so far (this node's, then parent's and so on): ");
        foreach (CbsConstraint constraint in constraints)
        {
            Debug.WriteLine(constraint);
        }
        HashSet<CbsConstraint> mustConstraints = GetPositiveConstraints(); // TODO: Ordered
        Debug.WriteLine($"{mustConstraints.Count} relevant internal must constraints so far: ");
        foreach (CbsConstraint mustConstraint in mustConstraints)
        {
            Debug.WriteLine(mustConstraint);
        }
        ProblemInstance problem = CBS.GetProblemInstance();
        if (CBS.ExternalConstraints != null)
        {
            Debug.WriteLine($"{CBS.ExternalConstraints.Count} external constraints: ");
            foreach (CbsConstraint constraint in CBS.ExternalConstraints)
            {
                Debug.WriteLine(constraint);
            }
        }
        Debug.WriteLine($"Conflict: {GetConflict()}");
        Debug.Write("Agent group assignments: ");
        for (int j = 0; j < AgentsGroupAssignment.Length; j++)
        {
            Debug.Write($" {AgentsGroupAssignment[j],3}");
        }
        Debug.WriteLine("");
        Debug.Write("Single agent costs:      ");  // Extra spaces to align with the group assignments line
        for (int j = 0; j < SingleAgentCosts.Length; j++)
        {
            Debug.Write($" {SingleAgentCosts[j],3}");
        }
        Debug.WriteLine("");
        Debug.Write("Internal agents that conflict with each agent: ");
        for (int j = 0; j < _countsOfInternalAgentsThatConflict.Length; j++)
        {
            Debug.Write($" {_countsOfInternalAgentsThatConflict[j]}");
        }
        Debug.WriteLine("");
        Debug.Write("New plans: ");
        for (int j = 0; j < NewPlans.Length; j++)
        {
            Debug.Write($" {(NewPlans[j] ? 1 : 0)}");
        }
        Debug.WriteLine("");
        for (int j = 0; j < ConflictCountsPerAgent.Length; j++)
        {
            if (ConflictCountsPerAgent[j].Count != 0)
            {
                Debug.Write($"Agent {problem.agents[j].agent.agentNum} conflict counts: ");
                foreach (var pair in ConflictCountsPerAgent[j])
                {
                    Debug.Write($"{pair.Key}:{pair.Value} ");
                }
                Debug.WriteLine("");

            }
        }
        for (int j = 0; j < ConflictTimesPerAgent.Length; j++)
        {
            if (ConflictCountsPerAgent[j].Count != 0)
            {
                Debug.Write($"Agent {problem.agents[j].agent.agentNum} conflict times: ");
                foreach (var pair in ConflictTimesPerAgent[j])
                {
                    Debug.Write($"{pair.Key}:[{String.Join(",", pair.Value)}], ");
                }
                Debug.WriteLine("");

            }
        }
        if (CBS.GetType() == typeof(MACBS_WholeTreeThreshold) && CBS.MergeThreshold != -1)
        {
            for (int i = 0; i < ((MACBS_WholeTreeThreshold)CBS).globalConflictsCounter.Length; i++)
            {
                Debug.Write($"Agent {i} global historic conflict counts: ");
                for (int j = 0; j < i; j++)
                {
                    Debug.Write($"a{j}:{((MACBS_WholeTreeThreshold)CBS).globalConflictsCounter[i][j]} ");
                }
                Debug.WriteLine("");
            }
        }
        var plan = CalculateJointPlan();
        plan.PrintPlanIfShort();
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
    protected void UpdateAtGoalConflictCounts(int agentIndex, ConflictAvoidanceTable CAT)
    {
        ProblemInstance problem = CBS.GetProblemInstance();
        var afterGoal = new TimedMove(
            problem.agents[agentIndex].agent.Goal.X, problem.agents[agentIndex].agent.Goal.Y,
            Direction.Wait, time: 0);
        for (int time = SingleAgentPlans[agentIndex].GetSize(); time < CAT.GetMaxPlanSize(); time++)
        {
            afterGoal.Time = time;
            afterGoal.IncrementConflictCounts(CAT,
                                            ConflictCountsPerAgent[AgentsGroupAssignment[agentIndex]],
                                            ConflictTimesPerAgent[AgentsGroupAssignment[agentIndex]]);
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
    /// Happily, it has a 2-approximation: Just choose both endpoints of each uncovered edge
    /// repeatedly until no uncovered edges are left. So we can just take half the count from
    /// that approximation.
    /// 
    /// TODO: the graph is small enough that we can try to solve optimally.
    /// 
    /// Notice a merge is like two replans in one, so we might need to take ceil(num_replans/2).
    /// Luckily, in MA-CBS which considers only conflicts in the same CT branch,
    /// a merge is only possible once every B+1 depth steps,
    /// because we only count selected conflicts (they're guaranteed to be unequal),
    /// so we can cap the number of possible merges and subtract less.
    /// 
    /// In Cbs_GlobalConflicts, we could use the global table to discount some merges.
    /// </summary>
    protected void CalcMinOpsToSolve()
    {
        if (!CBS.DisableTieBreakingByMinOpsEstimate)
        {
            HashSet<int> vertexCover = [];

            for (int i = 0; i < ConflictCountsPerAgent.Length; i++)
            {
                if (vertexCover.Contains(i)) // This node is already in the cover - all its edges are already covered.
                    continue;

                foreach (KeyValuePair<int, int> otherEndAgentNumAndCount in ConflictCountsPerAgent[i])
                {
                    if (AgentNumToIndex.ContainsKey(otherEndAgentNumAndCount.Key)) // It's an internal conflict
                    {
                        int otherEndIndex = AgentNumToIndex[otherEndAgentNumAndCount.Key];
                        if (vertexCover.Contains(otherEndAgentNumAndCount.Key) == false) // The vertex isn't covered from its other end yet
                        {
                            vertexCover.Add(i);
                            vertexCover.Add(otherEndIndex);
                            break; // All of this node's edges are now covered.
                        }
                    }
                }
            }

            int minReplansToSolve = vertexCover.Count / 2; // We have a 2-approximation of the size of the cover -
                                                            // half that is at least half the value we're trying to approximate.
                                                            // (The size of the approximation is always even)
            //if (cbs.debug)
            //    Debug.WriteLine("min replans lower estimate: " + minReplansToSolve);
            if (CBS.MergeThreshold != -1) // Merges possible, account for them
                                                // This assumes the current merging strategy is used.
            {
                if (CBS.GetType() == typeof(CBS))
                {
                    if (CBS.MergeThreshold > 0)
                    {
                        int maxPotentialMergeSavings = (int)Math.Floor(((double)minReplansToSolve) / 2);
                        int depthToGoTo = _depth + minReplansToSolve;
                        int chainSize = CBS.MergeThreshold + 1; // Every series of B+1 downwards consecutive nodes may end with a merge.
                        int maxMerges = depthToGoTo / chainSize; // Round down to discount the last unfinished chain.

                        // Count the minimum amount of merges already done and subtract it from maxMerges:
                        Dictionary<int, int> groupSizes = [];
                        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
                        {
                            if (groupSizes.ContainsKey(AgentsGroupAssignment[i]) == false)
                                groupSizes[AgentsGroupAssignment[i]] = 0;
                            groupSizes[AgentsGroupAssignment[i]]++;
                        }
                        // Not using GetGroupSizes() because what we want is actually
                        // a list of the sizes of the different groups, not the size of each agent's group

                        foreach (int groupSize in groupSizes.Values)
                            maxMerges -= (int)Math.Ceiling(Math.Log(groupSize, 2)); // A group of size 1 has had zero merges, a group of size 2 has had 1, larger groups have had at least ceil(log2) their size merges.

                        int maxMergeSavings = Math.Min(maxPotentialMergeSavings, maxMerges);

                        MinOpsToSolve = minReplansToSolve - maxMergeSavings;


                    }
                    else
                        MinOpsToSolve = (int)Math.Ceiling(((double)minReplansToSolve) / 2);
                }
                else
                    MinOpsToSolve = (int)Math.Ceiling(((double)minReplansToSolve) / 2); // TODO: We could look at the global table and maybe deduce something, but I'm not interested in that right now.
            }
            else
                MinOpsToSolve = (int)minReplansToSolve;
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
        HashSet<int> externalConflictingAgentNums = [];
        TotalInternalAgentsThatConflict = 0;
        TotalConflictsBetweenInternalAgents = 0;
        TotalConflictsWithExternalAgents = 0;

        for (int i = 0; i < ConflictCountsPerAgent.Length; i++)
        {
            _countsOfInternalAgentsThatConflict[i] = 0;

            if (ConflictCountsPerAgent[i].Count != 0)
                TotalInternalAgentsThatConflict++;

            foreach (KeyValuePair<int, int> conflictingAgentNumAndCount in ConflictCountsPerAgent[i])
            {
                if (AgentNumToIndex.ContainsKey(conflictingAgentNumAndCount.Key)) // It's an internal conflict
                {
                    _countsOfInternalAgentsThatConflict[i]++; // Counts one conflict for each agent the i'th agent conflicts with
                    TotalConflictsBetweenInternalAgents += conflictingAgentNumAndCount.Value;
                }
                else
                {
                    externalConflictingAgentNums.Add(conflictingAgentNumAndCount.Key);
                    TotalConflictsWithExternalAgents += conflictingAgentNumAndCount.Value;
                    ConflictTimesPerAgent[i].Remove(conflictingAgentNumAndCount.Key); // Not needed
                }
            }
        }

        _totalExternalAgentsThatConflict = externalConflictingAgentNums.Count;

        TotalConflictsBetweenInternalAgents /= 2; // Each conflict was counted twice
        TotalConflictsWithExternalAgents /= 2; // Each conflict was counted twice
    }

    /// <summary>
    /// Used to preserve state of conflict iteration.
    /// </summary>
    private IEnumerator<CbsConflict> _nextConflicts;

    /// <summary>
    /// The iterator holds the state of the generator, with all the different queues etc - a lot of memory.
    /// We also clear the MDD narrowness values that were computed - if no child uses them, they'll be garbage-collected.
    /// </summary>
    public void ClearConflictChoiceData() => _nextConflicts = null;

    /// <summary>
    /// Use after expanding a node and finding the conflict wasn't cardinal
    /// </summary>
    /// <returns>Whether we found a new potentially cardinal conflict to work on</returns>
    public bool ChooseNextPotentiallyCardinalConflicts()
    {
        if (nextConflictCouldBeCardinal)
        {
            bool cycled = ChooseNextConflict();
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
        bool hadNext = _nextConflicts.MoveNext();
        if (hadNext)
            Conflict = _nextConflicts.Current;
        return hadNext;
    }

    /// <summary>
    /// Chooses an internal conflict to work on.
    /// Resets conflicts iteration if it's used.
    /// </summary>
    public void ChooseConflict()
    {
        if (SingleAgentPlans.Length == 1) // A single internal agent can't conflict with anything internally
            return;

        if (isGoal) // Goal nodes don't have conflicts
            return;

        if (Conflict != null) // Conflict already chosen before
            return;

        if (CBS.ConflictChoice == ConflictChoice.FIRST)
        {
            ChooseFirstConflict();
        }
        else if (CBS.ConflictChoice == ConflictChoice.MOST_CONFLICTING_SMALLEST_AGENTS)
        {
            ChooseConflictOfMostConflictingSmallestAgents();
        }
        else if (CBS.ConflictChoice == ConflictChoice.CARDINAL_MDD)
        {
            // Choose the first (in order of looking at them), earliest (in time), cardinal
            // (if not found settle for semi-cardinal, then non-cardinal) conflict.
            // Assumes mergeThreshold == -1.
            _nextConflicts = GetConflictsCardinalFirstUsingMdd().GetEnumerator();
            bool hasConflict = _nextConflicts.MoveNext(); // This node isn't a goal node so this is expected to return true -
                                                                // a conflict should be found
            if (hasConflict == false)
            {
                DebugPrint();
                Trace.Assert(false, "Non-goal node found no conflict");
            }
            Conflict = _nextConflicts.Current;
        }
        else if (CBS.ConflictChoice == ConflictChoice.CARDINAL_LOOKAHEAD)
        {
            _nextConflicts = GetConflictsNoOrder().GetEnumerator();
            bool hasConflict = _nextConflicts.MoveNext(); // This node isn't a goal node so this is expected to return true -
                                                                // a conflict should be found
            if (hasConflict == false)
            {
                DebugPrint();
                Trace.Assert(false, "Non-goal node found no conflict");
            }
            Conflict = _nextConflicts.Current;
            //FIXME: code dup with previous option
        }
        else if (CBS.ConflictChoice == ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_SMALLEST_GROUP ||
                 CBS.ConflictChoice == ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_AND_SMALLEST_GROUP)
        {
            _nextConflicts = GetConflictsCardinalFirstUsingMddMergeFirstByNewPolicy().GetEnumerator();
            bool hasConflict = _nextConflicts.MoveNext(); // This node isn't a goal node so this is expected to return true -
                                                              // a conflict should be found
            if (hasConflict == false)
            {
                DebugPrint();
                Trace.Assert(false, "Non-goal node found no conflict");
            }
            Conflict = _nextConflicts.Current;
        }
        else
            throw new Exception("Unknown conflict-choosing method");
    }

    private void ChooseConflictOfMostConflictingSmallestAgents()
    {
        (int groupRepA, int groupRepB, int time) = GetDetailsOfConflictOfMostConflictingSmallestAgents();
        Conflict = FindConflict(groupRepA, groupRepB, time);
    }

    private void ChooseFirstConflict()
    {
        (int groupRepA, int groupRepB, int time) = GetFirstConflictDetails();
        Conflict = FindConflict(groupRepA, groupRepB, time);
    }

    /// <summary>
    /// No special ordering.
    /// </summary>
    /// <returns></returns>
    private IEnumerable<CbsConflict> GetConflictsNoOrder()
    {
        ISet<int>[] groups = GetGroups();
        nextConflictCouldBeCardinal = true; // We don't know

        for (int agentIndex = 0; agentIndex < ConflictTimesPerAgent.Length; agentIndex++)
        {
            foreach (int conflictingAgentNum in ConflictTimesPerAgent[agentIndex].Keys)
            {
                int conflictingAgentIndex = AgentNumToIndex[conflictingAgentNum];
                if (conflictingAgentIndex < agentIndex)
                    continue; // Return each conflict only once

                foreach (int conflictTime in ConflictTimesPerAgent[agentIndex][conflictingAgentNum])
                {
                    yield return FindConflict(agentIndex, conflictingAgentIndex, conflictTime, groups);
                }
            }
        }
    }

    /// <summary>
    /// Assumes mergeThreshold == -1.
    /// Builds MDDs for all agents.
    /// Not currently used.
    /// </summary>
    /// <returns></returns>
    private IEnumerable<CbsConflict> GetConflictsExhaustivelySearchingForCardinalsGreedily()
    {
        buildAllMDDs();
        return GetConflictsCardinalFirstUsingMdd();
    }

    /// <summary>
    /// Currently only used by the above unused function
    /// </summary>
    public void buildAllMDDs()
    {
        foreach (var agentIndex in Enumerable.Range(0, SingleAgentPlans.Length))
        {
            if (ConflictTimesPerAgent[agentIndex].Count == 0)
                continue;  // Agent has no conflicts
            buildMddForAgentWithItsCurrentCost(agentIndex);  // Does nothing if it's built already
        }
    }

    /// <summary>
    /// Assumes mergeThreshold == -1.
    /// Builds MDDs as necessary until a cardinal conflict is found.
    /// Also sets h to 1 if a cardinal conflict is found.
    /// Not currently used.
    /// </summary>
    /// <returns></returns>
    private IEnumerable<CbsConflict> GetConflictsExhaustivelySearchingForCardinalsLazily()
    {
        ISet<int>[] groups = GetGroups();

        foreach (var agentIndex in Enumerable.Range(0, SingleAgentPlans.Length))
        {
            if (ConflictTimesPerAgent[agentIndex].Count == 0)
                continue;  // Agent has no conflicts
            bool hasMdd = MDDNarrownessValues[agentIndex] != null ||
                CopyAppropriateMddFromParent(agentIndex);

            foreach (int conflictingAgentNum in ConflictTimesPerAgent[agentIndex].Keys)
            {
                int conflictingAgentIndex = AgentNumToIndex[conflictingAgentNum];
                bool otherHasMdd = MDDNarrownessValues[conflictingAgentIndex] != null ||
                    CopyAppropriateMddFromParent(conflictingAgentIndex);

                foreach (int conflictTime in ConflictTimesPerAgent[agentIndex][conflictingAgentNum])
                {
                    if (otherHasMdd == false || DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, agentIndex, groups))  // Other agent's MDD is narrow at this timestep.
                    {
                        buildMddForAgentWithItsCurrentCost(agentIndex);
                        hasMdd = true;
                    }
                    else
                        continue;
                    bool iNarrow = DoesAgentHaveNoOtherOption(agentIndex, conflictTime, conflictingAgentIndex, groups);
                    if (iNarrow == false)
                        continue;
                    if (otherHasMdd == false)
                    {
                        buildMddForAgentWithItsCurrentCost(conflictingAgentIndex);
                        otherHasMdd = true;
                    }
                    bool otherNarrow = DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, agentIndex, groups);
                    if (otherNarrow)  // Both narrow!
                    {
                        CbsConflict cardinal = FindConflict(agentIndex, conflictingAgentIndex, conflictTime);
                        cardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.YES;
                        cardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.YES;
                        nextConflictCouldBeCardinal = false;  // Don't cycle conflicts even if the cost doesn't increase (can happen if this is resolved via a merge operation because it also removes some constraints)
                        yield return cardinal;
                    }
                }
            }
        }

        // No cardinal conflict was found
        var FullSearchIterator = GetConflictsCardinalFirstUsingMdd();
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
        if (TotalConflictsBetweenInternalAgents == 1)
        {
            Debug.WriteLine("Single conflict. Just choosing it.");
            return GetConflictsNoOrder();
        }
        return GetConflictsCardinalFirstUsingMddInternal();
    }

    private IEnumerable<CbsConflict> GetConflictsCardinalFirstUsingMddMergeFirstByNewPolicy()
    {
        if (TotalConflictsBetweenInternalAgents == 1)
        {
            Debug.WriteLine("Single conflict. Just choosing it.");
            return GetConflictsNoOrder();
        }
        return GetConflictsCardinalFirstUsindMddMergeFirstByNewPolicyInternal();
    }

    /// <summary>
    /// Builds MDDs as lazily as possible.
    /// </summary>
    /// <returns>
    /// Iterates over conflicts in the following order: certainly cardinal (by 2 MDDs),
    /// possibly cardinal (by 1 MDD, the other agent is a meta-agent),
    /// possibly cardinal (2 meta-agents),
    /// semi-cadinal (by 2 MDDs), possibly semi-cardinal (by 1 mdd, the other agent is a meta-agent),
    /// non-cardinal
    /// </returns>
    private IEnumerable<CbsConflict> GetConflictsCardinalFirstUsingMddInternal()
    {
        ISet<int>[] groups = GetGroups();
        // Queue items are <first agent index, second agent index, time>
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> NotCardinalMaybeSemi = new(TotalConflictsBetweenInternalAgents); // Because first has an MDD
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> NotCardinalNotSemi = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> SemiCardinal = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalFirstHasMddSecondDoesNotButCan = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalFirstHasMddSecondCannot = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalBothCannotBuildMdd = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalFirstCanBuildMdd = new(TotalConflictsBetweenInternalAgents); // Going over these just get the first element, build its MDD and 
        Queue<int> AgentIndexesWaitingToCheckTheirConflictsForCardinality = new(Enumerable.Range(0, SingleAgentPlans.Length)); // Initially go over all conflicting agents.
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
                bool hasMDD = MDDNarrownessValues[i] != null ||  // No need to check if its levels is null, we don't sync MDDs and we know there's a path with the current cost for the agent
                                CopyAppropriateMddFromParent(i);
                bool canBuildMDD = groups[i].Count == 1;

                foreach (int conflictingAgentNum in ConflictTimesPerAgent[i].Keys)
                {
                    int conflictingAgentIndex = AgentNumToIndex[conflictingAgentNum];
                    bool otherCanBuildMdd = groups[conflictingAgentIndex].Count == 1 && MDDNarrownessValues[conflictingAgentIndex] == null;
                    if (allowAgentOrderFlip)
                    {
                        if (i < conflictingAgentIndex &&  // We'll see this pair again in the other order
                            canBuildMDD == false && otherCanBuildMdd)
                            continue; // We'll take care of this conflict from the other end,
                                      // because only the second agent can build an MDD and we
                                      // prefer the agent that can build an MDD to be the first one.
                        if (i > conflictingAgentIndex &&  // This is the second time we're seeing this pair
                            ((canBuildMDD && otherCanBuildMdd == false) == false))  // We didn't skip to this order earlier
                            continue;  // Already taken care of
                    }
                    bool otherHasMDD = MDDNarrownessValues[conflictingAgentIndex] != null ||
                        CopyAppropriateMddFromParent(conflictingAgentIndex);  // FIXME: If no ancestor has an appropriate MDD, this might be checked multiple times :(

                    // Reaching here means either i < conflictingAgentIndex,
                    // or the i'th agent can build an MDD and the conflictingAgentIndex'th can't.
                    foreach (int conflictTime in ConflictTimesPerAgent[i][conflictingAgentNum])
                    {
                        if (hasMDD) // Check if not cardinal
                        {
                            bool iNarrow = DoesAgentHaveNoOtherOption(i, conflictTime, conflictingAgentIndex, groups);
                            if (iNarrow == false) // Then it isn't cardinal. May still be semi cardinal.
                            {
                                if (otherHasMDD == false) // Skip building the second MDD even if it's possible
                                {
                                    NotCardinalMaybeSemi.Enqueue((i, conflictingAgentIndex, conflictTime));
                                    continue;
                                }
                                else // Other has MDD
                                {
                                    bool otherNarrow = DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, i, groups);
                                    if (otherNarrow == false)
                                    {
                                        NotCardinalNotSemi.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                    else // Other narrow but i not narrow at the time of the conflict
                                    {
                                        SemiCardinal.Enqueue((conflictingAgentIndex, i, conflictTime));  // Order important to know whose cost will increase
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
                                        PossiblyCardinalFirstHasMddSecondDoesNotButCan.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                    else // iNarrow, other cannot build an MDD
                                    {
                                        PossiblyCardinalFirstHasMddSecondCannot.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                }
                                else // Other has MDD
                                {
                                    bool otherNarrow = DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, i, groups);
                                    if (otherNarrow == false) // iNarrow but other not narrow
                                    {
                                        SemiCardinal.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                    else // Both narrow!
                                    {
                                        Debug.WriteLine("Cardinal conflict chosen.");
                                        CbsConflict cardinal = FindConflict(i, conflictingAgentIndex, conflictTime, groups);
                                        cardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.YES;
                                        cardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.YES;
                                        H = Math.Max(H, 1);  // The children's cost will be at least 1 more than this node's cost
                                        nextConflictCouldBeCardinal = false;  // We don't want CBS to cycle conflicts after this one.
                                                                                   // This could happen if the conflict is resolved via a merge
                                                                                   // and the conflicting agents already have some constraints
                                                                                   // to avoid each other that have already increased the cost
                                                                                   // of their paths
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
                                PossiblyCardinalFirstCanBuildMdd.Enqueue((i, conflictingAgentIndex, conflictTime));
                                continue;
                            }
                            else // No MDD and can't build one and other can't build one either (already checked for the latter case above)
                                    // When re-checking an agent's conflicts we'll never get here because we only recheck agents that can build an MDD
                            {
                                PossiblyCardinalBothCannotBuildMdd.Enqueue((i, conflictingAgentIndex, conflictTime));
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
                int agentToBuildAnMddFor = PossiblyCardinalFirstHasMddSecondDoesNotButCan.Dequeue().agentBIndex;
                buildMddForAgentWithItsCurrentCost(agentToBuildAnMddFor);
                //   b. Remove other conflicts from PossiblyCardinalFirstHasMddSecondDoesNotButCan where the second
                //      agent is the one we built an MDD for (in all of those, the first agent's index is lower than the second's,
                //      since we could build an MDD for it).
                PossiblyCardinalFirstHasMddSecondDoesNotButCan = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                        PossiblyCardinalFirstHasMddSecondDoesNotButCan.Where(tuple => tuple.agentBIndex != agentToBuildAnMddFor));
                //   c. Remove conflicts from PossiblyCardinalFirstCanBuildMdd where the first or second agent is the one we
                //      built the MDD for.
                PossiblyCardinalFirstCanBuildMdd = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                    PossiblyCardinalFirstCanBuildMdd.Where(
                        tuple => (tuple.agentBIndex != agentToBuildAnMddFor) && (tuple.agentAIndex != agentToBuildAnMddFor)));
                //   d. Remove conflicts from NotCardinalMaybeSemi where the second agent is the one we
                //      built the MDD for
                NotCardinalMaybeSemi = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                    NotCardinalMaybeSemi.Where(tuple => tuple.agentBIndex != agentToBuildAnMddFor));
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
                int agentToBuildAnMddFor = PossiblyCardinalFirstCanBuildMdd.Dequeue().agentAIndex;
                buildMddForAgentWithItsCurrentCost(agentToBuildAnMddFor);
                //   b. Remove other conflicts from PossiblyCardinalFirstHasMddSecondDoesNotButCan where the second
                //      agent is the one we built an MDD for (in all of those, the first agent's index is lower than the second's,
                //      since we could build an MDD for it).
                PossiblyCardinalFirstHasMddSecondDoesNotButCan = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                        PossiblyCardinalFirstHasMddSecondDoesNotButCan.Where(tuple => tuple.agentBIndex != agentToBuildAnMddFor));
                //   c. Remove conflicts from PossiblyCardinalFirstCanBuildMdd where the first or second agent is the one we
                //      built the MDD for.
                PossiblyCardinalFirstCanBuildMdd = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                    PossiblyCardinalFirstCanBuildMdd.Where(
                        tuple => (tuple.agentBIndex != agentToBuildAnMddFor) && (tuple.agentAIndex != agentToBuildAnMddFor)));
                //   d. Remove conflicts from NotCardinalMaybeSemi where the second agent is the one we
                //      built the MDD for
                NotCardinalMaybeSemi = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                    NotCardinalMaybeSemi.Where(tuple => tuple.agentBIndex != agentToBuildAnMddFor));
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
            Debug.WriteLine("Checking for cardinality via a lookahead...");
            var tuple = PossiblyCardinalFirstHasMddSecondCannot.Dequeue();
            nextConflictCouldBeCardinal = (PossiblyCardinalFirstHasMddSecondCannot.Count != 0) ||
                                                (PossiblyCardinalBothCannotBuildMdd.Count != 0);
            var possiblyCardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            possiblyCardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.YES;
            possiblyCardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.MAYBE;
            yield return possiblyCardinal;
        }

        // Yield the possibly cardinal conflicts where we can't build an MDD for either agent
        while (PossiblyCardinalBothCannotBuildMdd.Count != 0)
        {
            Debug.WriteLine("Checking for cardinality via a lookahead...");
            var tuple = PossiblyCardinalBothCannotBuildMdd.Dequeue();
            nextConflictCouldBeCardinal = PossiblyCardinalBothCannotBuildMdd.Count != 0;
            var possiblyCardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            possiblyCardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.MAYBE;
            possiblyCardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.MAYBE;
            yield return possiblyCardinal;
        }

        nextConflictCouldBeCardinal = false;

        // Yield semi cardinal conflicts
        while (SemiCardinal.Count != 0)
        {
            Debug.WriteLine("Settling for a semi-cardinal conflict.");
            var tuple = SemiCardinal.Dequeue();
            var semiCardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            semiCardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.YES;
            semiCardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.NO;
            yield return semiCardinal;
        }

        // Yield the non cardinal conflicts, possibly semi first
        while (NotCardinalMaybeSemi.Count != 0)
        {
            Debug.WriteLine("No cardinal conflict found. This one's possibly a semi cardinal conflict.");
            var tuple = NotCardinalMaybeSemi.Dequeue();
            var conflict = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            conflict.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.NO;
            conflict.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.MAYBE;
            yield return conflict;
        }

        while (NotCardinalNotSemi.Count != 0)
        {
            Debug.WriteLine("Non-cardinal conflict chosen");
            var tuple = NotCardinalNotSemi.Dequeue();
            var conflict = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            conflict.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.NO;
            conflict.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.NO;
            yield return conflict;
        }

        Trace.Assert(NotCardinalMaybeSemi.Count == 0);
        Trace.Assert(NotCardinalNotSemi.Count == 0);
        Trace.Assert(SemiCardinal.Count == 0);
        Trace.Assert(PossiblyCardinalFirstHasMddSecondDoesNotButCan.Count == 0);
        Trace.Assert(PossiblyCardinalFirstHasMddSecondCannot.Count == 0);
        Trace.Assert(PossiblyCardinalBothCannotBuildMdd.Count == 0);
        Trace.Assert(PossiblyCardinalFirstCanBuildMdd.Count == 0);
        Trace.Assert(AgentIndexesWaitingToCheckTheirConflictsForCardinality.Count == 0);
    }

    /// <summary>
    /// Finds conflicts from the highest cardinality class that's available and returns them
    /// in the order of the merge policy (most conflicting with others smallest resulting meta-agent that should be merged first).
    /// Doesn't necessarily find all conflicts - some may be skipped.
    /// </summary>
    /// <returns>
    /// Iterates over conflicts in the following order: certainly cardinal, should be merged (in order of merge policy),
    /// certainly cardinal, shouldn't be merged,
    /// semi-cardinal that should be merged (in order of merge policy),
    /// semi-cardinal that shouldn't be merged
    /// non-cardinal that should be merged (in order of merge policy),
    /// non-cardinal that shouldn't be merged
    /// </returns>
    private IEnumerable<CbsConflict> GetConflictsCardinalFirstUsindMddMergeFirstByNewPolicyInternal()
    {
        ISet<int>[] groups = GetGroups();
        // Queue items are <first agent index, second agent index, time>
        List<(int agentAIndex, int agentBIndex, int conflictTime)> CardinalShouldMerge = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> Cardinal = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> NotCardinalMaybeSemi = new(TotalConflictsBetweenInternalAgents); // Because first has an MDD
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> NonCardinalShouldMerge = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> NotCardinalNotSemi = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> SemiCardinal = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalFirstHasMddSecondCannotShouldMerge = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalBothCannotBuildMddShouldMerge = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalFirstHasMddSecondDoesNotButCan = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalFirstHasMddSecondCannot = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalBothCannotBuildMdd = new(TotalConflictsBetweenInternalAgents);
        Queue<(int agentAIndex, int agentBIndex, int conflictTime)> PossiblyCardinalFirstCanBuildMdd = new(TotalConflictsBetweenInternalAgents); // Going over these just get the first element, build its MDD and 
        Queue<int> AgentIndexesWaitingToCheckTheirConflictsForCardinality = new(Enumerable.Range(0, SingleAgentPlans.Length)); // Initially go over all conflicting agents.
                                                                                                                               // TODO: this will also go over non-conflicting agents harmlessly. Is there an easy way to get a list of agents that have conflicts?
                                                                                                                               // Positively cardinal conflicts are just yielded immediately
                                                                                                                               // Conflicting agents are only entered into a queue once. Only if the conflicting agent with the larger index
                                                                                                                               // can have an MDD built and the one with the lower can't, a pair of conflicting agents is entered in reverse.
        Dictionary<(int agentAIndex, int agentBIndex), bool> shouldMergePair = [];
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
                bool hasMDD = MDDNarrownessValues[i] != null ||  // No need to check if its levels is null, we don't sync MDDs and we know there's a path with the current cost for the agent
                                CopyAppropriateMddFromParent(i);
                bool canBuildMDD = groups[i].Count == 1;

                foreach (int conflictingAgentNum in ConflictTimesPerAgent[i].Keys)
                {
                    int conflictingAgentIndex = AgentNumToIndex[conflictingAgentNum];
                    bool otherCanBuildMdd = groups[conflictingAgentIndex].Count == 1 && MDDNarrownessValues[conflictingAgentIndex] == null;
                    if (allowAgentOrderFlip)
                    {
                        if (i < conflictingAgentIndex &&  // We'll see this pair again in the other order
                            canBuildMDD == false && otherCanBuildMdd)
                            continue; // We'll take care of this conflict from the other end,
                                      // because only the second agent can build an MDD and we
                                      // prefer the agent that can build an MDD to be the first one.
                        if (i > conflictingAgentIndex &&  // This is the second time we're seeing this pair
                            ((canBuildMDD && otherCanBuildMdd == false) == false))  // We didn't skip to this order earlier
                            continue;  // Already taken care of
                    }
                    bool otherHasMDD = MDDNarrownessValues[conflictingAgentIndex] != null ||
                        CopyAppropriateMddFromParent(conflictingAgentIndex);  // FIXME: If no ancestor has an appropriate MDD, this might be checked multiple times :(
                    bool shouldMergeThisPair;
                    if (shouldMergePair.ContainsKey((i, conflictingAgentIndex)) == false)
                    {
                        shouldMergeThisPair = ShouldMerge(CBS.MergeThreshold, i, conflictingAgentIndex);
                        shouldMergePair[(i, conflictingAgentIndex)] = shouldMergeThisPair;
                        shouldMergePair[(conflictingAgentIndex, i)] = shouldMergeThisPair;
                    }
                    else
                        shouldMergeThisPair = shouldMergePair[(i, conflictingAgentIndex)];
                    if (shouldMergeThisPair)  // Don't delay building MDDs for them
                    {
                        if (!hasMDD && canBuildMDD)
                        {
                            buildMddForAgentWithItsCurrentCost(i);
                            hasMDD = true;
                        }
                        if (!otherHasMDD && otherCanBuildMdd)
                        {
                            buildMddForAgentWithItsCurrentCost(conflictingAgentIndex);
                            otherHasMDD = true;
                        }
                    }

                    // TODO: Once we find a potentially cardinal conflict we don't need to look at other potentially-but-not-certainly cardinal conflicts,
                    // semi cardinal conflicts or non cardinal conflicts for this pair. This would only save some queue operations.

                    // Reaching here means either i < conflictingAgentIndex,
                    // or the i'th agent can build an MDD and the conflictingAgentIndex'th can't.
                    foreach (int conflictTime in ConflictTimesPerAgent[i][conflictingAgentNum])
                    {
                        if (hasMDD) // Check if not cardinal
                        {
                            bool iNarrow = DoesAgentHaveNoOtherOption(i, conflictTime, conflictingAgentIndex, groups);
                            if (iNarrow == false) // Then it isn't cardinal. May still be semi cardinal.
                            {
                                if (otherHasMDD == false) // Skip building the second MDD even if it's possible
                                {
                                    if (shouldMergeThisPair)
                                        NonCardinalShouldMerge.Enqueue((i, conflictingAgentIndex, conflictTime));  // No semi-cardinal conflicts possible
                                                                                                                          // for nodes with a single child
                                    else
                                        NotCardinalMaybeSemi.Enqueue((i, conflictingAgentIndex, conflictTime));
                                    continue;
                                }
                                else // Other has MDD
                                {
                                    bool otherNarrow = DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, i, groups);
                                    if (otherNarrow == false)
                                    {
                                        if (shouldMergeThisPair)
                                            NonCardinalShouldMerge.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        else
                                            NotCardinalNotSemi.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                    else // Other narrow but i not narrow at the time of the conflict
                                    {
                                        if (shouldMergeThisPair)
                                            NonCardinalShouldMerge.Enqueue((conflictingAgentIndex, i, conflictTime));
                                        else
                                            SemiCardinal.Enqueue((conflictingAgentIndex, i, conflictTime));  // Order important to know whose cost will increase
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
                                        Trace.Assert(shouldMergeThisPair == false, "we should have already built an mdd");
                                        PossiblyCardinalFirstHasMddSecondDoesNotButCan.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                    else // iNarrow, other cannot build an MDD
                                    {
                                        if (shouldMergeThisPair)
                                            PossiblyCardinalFirstHasMddSecondCannotShouldMerge.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        else
                                            PossiblyCardinalFirstHasMddSecondCannot.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                }
                                else // Other has MDD
                                {
                                    bool otherNarrow = DoesAgentHaveNoOtherOption(conflictingAgentIndex, conflictTime, i, groups);
                                    if (otherNarrow == false) // iNarrow but other not narrow
                                    {
                                        if (shouldMergeThisPair)
                                            NonCardinalShouldMerge.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        else
                                            SemiCardinal.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                    else // Both narrow!
                                    {
                                        if (shouldMergeThisPair)
                                        {
                                            CardinalShouldMerge.Add((i, conflictingAgentIndex, conflictTime));
                                            break;  // Once we find a cardinal conflict between agents that should be merged we don't need to look at other
                                                    // conflicts for this pair
                                        }
                                        else
                                            Cardinal.Enqueue((i, conflictingAgentIndex, conflictTime));
                                        continue;
                                    }
                                }
                            }
                        }
                        else // No MDD
                        {
                            if (canBuildMDD)
                            {
                                Trace.Assert(shouldMergeThisPair == false, "we should have already built an mdd");
                                PossiblyCardinalFirstCanBuildMdd.Enqueue((i, conflictingAgentIndex, conflictTime));
                                continue;
                            }
                            else // No MDD and can't build one and other can't build one either (already checked for the latter case above)
                                 // When re-checking an agent's conflicts we'll never get here because we only recheck agents that can build an MDD
                            {
                                if (shouldMergeThisPair)
                                    PossiblyCardinalBothCannotBuildMddShouldMerge.Enqueue((i, conflictingAgentIndex, conflictTime));
                                else
                                    PossiblyCardinalBothCannotBuildMdd.Enqueue((i, conflictingAgentIndex, conflictTime));
                                continue;
                            }
                        }
                    }
                }
            }

            allowAgentOrderFlip = false;  // We've flipped all we needed above

            // 1.5. Check the possibly cardinal conflicts between agents that should be merged where we can't build an MDD for the second agent
            while (PossiblyCardinalFirstHasMddSecondCannotShouldMerge.Count != 0)
            {
                Debug.WriteLine("Checking for cardinality via a lookahead...");
                var tuple = PossiblyCardinalFirstHasMddSecondCannotShouldMerge.Dequeue();
                var possiblyCardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
                // Lookahead to check if it's a cardinal conflict. Note: If we merge & restart, then all current constraints
                // on the agents will be discarded, so the cost of the merged agent might actually go down!
                // But this is the way to tell if this conflict is avoidable or not.
                // FIXME: Lots of code duplication here
                int groupRepA = AgentsGroupAssignment[tuple.agentAIndex];
                int groupRepB = AgentsGroupAssignment[tuple.agentBIndex];
                int aCost = GetGroupCost(groupRepA);
                int bCost = GetGroupCost(groupRepB);
                int minAndMaxNewCost;
                if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                    minAndMaxNewCost = aCost + bCost;
                else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                            Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                    minAndMaxNewCost = Math.Max(aCost, bCost);
                else
                    throw new Exception("Unexpected cost function");
                var tempNode = new CbsNode(this, groupRepA, groupRepB);
                bool success = tempNode.Replan(groupRepA, CBS.MinSolutionTimeStep,
                    minPathCost: minAndMaxNewCost, maxPathCost: minAndMaxNewCost);
                if (success) // then not cardinal
                {
                    NonCardinalShouldMerge.Enqueue(tuple);
                }
                else
                {
                    CardinalShouldMerge.Add(tuple);
                    PossiblyCardinalFirstHasMddSecondCannotShouldMerge = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                        PossiblyCardinalFirstHasMddSecondCannotShouldMerge.Where(
                        other => !(other.agentAIndex == tuple.agentAIndex && other.agentBIndex == tuple.agentBIndex) &&
                                 !(other.agentAIndex == tuple.agentBIndex && other.agentBIndex == tuple.agentAIndex)
                    ));
                }
            }

            // 1.8. Check the possibly cardinal conflicts between agents that should be merged where we can't build an MDD for either agent
            while (PossiblyCardinalBothCannotBuildMddShouldMerge.Count != 0)
            {
                Debug.WriteLine("Checking for cardinality via a lookahead...");
                var tuple = PossiblyCardinalBothCannotBuildMddShouldMerge.Dequeue();
                var possiblyCardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
                // code dup w 1.5:
                // Lookahead to check if it's a cardinal conflict. Note: If we merge & restart, then all current constraints
                // on the agents will be discarded, so the cost of the merged agent might actually go down!
                // But this is the way to tell if this conflict is avoidable or not.
                // FIXME: Lots of code duplication here
                int groupRepA = AgentsGroupAssignment[tuple.agentAIndex];
                int groupRepB = AgentsGroupAssignment[tuple.agentBIndex];
                int aCost = GetGroupCost(groupRepA);
                int bCost = GetGroupCost(groupRepB);
                int minAndMaxNewCost;
                if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                    minAndMaxNewCost = aCost + bCost;
                else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                            Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                    minAndMaxNewCost = Math.Max(aCost, bCost);
                else
                    throw new Exception("Unexpected cost function");
                var tempNode = new CbsNode(this, groupRepA, groupRepB);
                bool success = tempNode.Replan(groupRepA, CBS.MinSolutionTimeStep,
                    minPathCost: minAndMaxNewCost, maxPathCost: minAndMaxNewCost);
                if (success) // then not cardinal
                {
                    NonCardinalShouldMerge.Enqueue(tuple);
                }
                else
                {
                    CardinalShouldMerge.Add(tuple);
                    PossiblyCardinalBothCannotBuildMddShouldMerge = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                        PossiblyCardinalBothCannotBuildMddShouldMerge.Where(
                        other => !(other.agentAIndex == tuple.agentAIndex && other.agentBIndex == tuple.agentBIndex) &&
                                 !(other.agentAIndex == tuple.agentBIndex && other.agentBIndex == tuple.agentAIndex)
                    ));
                }
            }

            // 1.9. Yield CardinalShouldMerge conflicts in order of policy
            while (CardinalShouldMerge.Count != 0)
            {
                int i;
                if (CBS.ConflictChoice == ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_SMALLEST_GROUP)
                {
                    i = CardinalShouldMerge.ToArray().IndexOfMax(tuple => -1 * (GetGroupSize(tuple.agentAIndex) + GetGroupSize(tuple.agentBIndex)));
                    //var array = CardinalShouldMerge.ToArray();
                    //var indices_of_smallest = array.IndicesOfMax(tuple => -1 * (GetGroupSize(tuple.agentAIndex) + GetGroupSize(tuple.agentBIndex)));  // Max of -1*value instead of adding min variants
                    //var i = indices_of_smallest.MaxByKeyFunc(index => conflicts(array[i].agentAIndex) + conflicts(array[i].agentAIndex))
                }
                else if (CBS.ConflictChoice == ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_AND_SMALLEST_GROUP)
                {
                    //i = CardinalShouldMerge.ToArray().IndexOfMax(tuple => conflicts / (1 << (GetGroupSize(tuple.agentAIndex) + GetGroupSize(tuple.agentBIndex) - 1)));
                    //...
                    throw new Exception("Unexpected");
                }
                else
                    throw new Exception("Unexpected");
                var tuple = CardinalShouldMerge[i];
                CardinalShouldMerge.RemoveAt(i);
                Debug.WriteLine("Chose a cardinal conflict between agents that should be merged");
                nextConflictCouldBeCardinal = (Cardinal.Count != 0) ||
                                                    (PossiblyCardinalFirstHasMddSecondCannot.Count != 0) ||
                                                    (PossiblyCardinalBothCannotBuildMdd.Count != 0);
                var cardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
                cardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.YES;
                cardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.YES;
                yield return cardinal;
            }

            // 1.99. yield cardinal conflicts
            while (Cardinal.Count != 0)
            {
                Debug.WriteLine("Chose a cardinal conflict");
                var tuple = Cardinal.Dequeue();
                nextConflictCouldBeCardinal = (Cardinal.Count != 0) ||
                                                    (PossiblyCardinalFirstHasMddSecondCannot.Count != 0) ||
                                                    (PossiblyCardinalBothCannotBuildMdd.Count != 0);
                var cardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
                cardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.YES;
                cardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.YES;
                yield return cardinal;
            }

            // 2. No cardinal conflicts that should be resolved with a merge and no cardinal conflicts.
            //    Build an MDD and re-check its agent's conflicts.
            if (PossiblyCardinalFirstHasMddSecondDoesNotButCan.Count != 0)
            {
                //   a. Get one conflict from PossiblyCardinalFirstHasMddSecondDoesNotButCan and build the second agent's
                //      MDD.
                int agentToBuildAnMddFor = PossiblyCardinalFirstHasMddSecondDoesNotButCan.Dequeue().agentBIndex;
                buildMddForAgentWithItsCurrentCost(agentToBuildAnMddFor);
                //   b. Remove other conflicts from PossiblyCardinalFirstHasMddSecondDoesNotButCan where the second
                //      agent is the one we built an MDD for (in all of those, the first agent's index is lower than the second's,
                //      since we could build an MDD for it).
                PossiblyCardinalFirstHasMddSecondDoesNotButCan = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                        PossiblyCardinalFirstHasMddSecondDoesNotButCan.Where(tuple => tuple.agentBIndex != agentToBuildAnMddFor));
                //   c. Remove conflicts from PossiblyCardinalFirstCanBuildMdd where the first or second agent is the one we
                //      built the MDD for.
                PossiblyCardinalFirstCanBuildMdd = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                    PossiblyCardinalFirstCanBuildMdd.Where(
                        tuple => (tuple.agentBIndex != agentToBuildAnMddFor) && (tuple.agentAIndex != agentToBuildAnMddFor)));
                //   d. Remove conflicts from NotCardinalMaybeSemi where the second agent is the one we
                //      built the MDD for
                NotCardinalMaybeSemi = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                    NotCardinalMaybeSemi.Where(tuple => tuple.agentBIndex != agentToBuildAnMddFor));
                //   e. No need to check for the agent's conflicts in PossiblyCardinalFirstHasMddSecondCannot,
                //      PossiblyCardinalBothCannotBuildMdd, NotCardinalNotSemi, SemiCardinal
                //   f. Enter the agent into AgentIndexesWaitingToCheckTheirConflictsForCardinality. 
                AgentIndexesWaitingToCheckTheirConflictsForCardinality.Enqueue(agentToBuildAnMddFor);
                continue;
            }

            // 3. Build an MDD for an agent and re-check its conflicts
            if (PossiblyCardinalFirstCanBuildMdd.Count != 0)
            {
                //   a. Get one conflict from PossiblyCardinalFirstCanBuildMdd and build the first agent's
                // MDD.
                int agentToBuildAnMddFor = PossiblyCardinalFirstCanBuildMdd.Dequeue().agentAIndex;
                buildMddForAgentWithItsCurrentCost(agentToBuildAnMddFor);
                //   b. Remove other conflicts from PossiblyCardinalFirstHasMddSecondDoesNotButCan where the second
                //      agent is the one we built an MDD for (in all of those, the first agent's index is lower than the second's,
                //      since we could build an MDD for it).
                PossiblyCardinalFirstHasMddSecondDoesNotButCan = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                        PossiblyCardinalFirstHasMddSecondDoesNotButCan.Where(tuple => tuple.agentBIndex != agentToBuildAnMddFor));
                //   c. Remove conflicts from PossiblyCardinalFirstCanBuildMdd where the first or second agent is the one we
                //      built the MDD for.
                PossiblyCardinalFirstCanBuildMdd = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                    PossiblyCardinalFirstCanBuildMdd.Where(
                        tuple => (tuple.agentBIndex != agentToBuildAnMddFor) && (tuple.agentAIndex != agentToBuildAnMddFor)));
                //   d. Remove conflicts from NotCardinalMaybeSemi where the second agent is the one we
                //      built the MDD for
                NotCardinalMaybeSemi = new Queue<(int agentAIndex, int agentBIndex, int conflictTime)>(
                    NotCardinalMaybeSemi.Where(tuple => tuple.agentBIndex != agentToBuildAnMddFor));
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
            Debug.WriteLine("Checking for cardinality via a lookahead...");
            var tuple = PossiblyCardinalFirstHasMddSecondCannot.Dequeue();
            nextConflictCouldBeCardinal = (PossiblyCardinalFirstHasMddSecondCannot.Count != 0) ||
                                                (PossiblyCardinalBothCannotBuildMdd.Count != 0);
            var possiblyCardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            possiblyCardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.YES;
            possiblyCardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.MAYBE;
            yield return possiblyCardinal;
        }

        // Yield the possibly cardinal conflicts where we can't build an MDD for either agent
        while (PossiblyCardinalBothCannotBuildMdd.Count != 0)
        {
            Debug.WriteLine("Checking for cardinality via a lookahead...");
            var tuple = PossiblyCardinalBothCannotBuildMdd.Dequeue();
            nextConflictCouldBeCardinal = PossiblyCardinalBothCannotBuildMdd.Count != 0;
            var possiblyCardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            possiblyCardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.MAYBE;
            possiblyCardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.MAYBE;
            yield return possiblyCardinal;
        }

        nextConflictCouldBeCardinal = false;

        // Yield semi cardinal conflicts
        while (SemiCardinal.Count != 0)
        {
            Debug.WriteLine("Settling for a semi-cardinal conflict.");
            var tuple = SemiCardinal.Dequeue();
            var semiCardinal = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            semiCardinal.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.YES;
            semiCardinal.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.NO;
            yield return semiCardinal;
        }

        // Yield the non cardinal conflicts, possibly semi first
        while (NotCardinalMaybeSemi.Count != 0)
        {
            Debug.WriteLine("No cardinal conflict found. This one's possibly a semi cardinal conflict.");
            var tuple = NotCardinalMaybeSemi.Dequeue();
            var conflict = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            conflict.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.NO;
            conflict.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.MAYBE;
            yield return conflict;
        }

        while (NotCardinalNotSemi.Count != 0)
        {
            Debug.WriteLine("Non-cardinal conflict chosen");
            var tuple = NotCardinalNotSemi.Dequeue();
            var conflict = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            conflict.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.NO;
            conflict.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.NO;
            yield return conflict;
        }

        while (NonCardinalShouldMerge.Count != 0)  // Yield those last to give them a chance to be resolved indirectly
        {
            Debug.WriteLine("A non-cardinal conflict between agents that should be merged was chosen");
            var tuple = NonCardinalShouldMerge.Dequeue();
            var conflict = FindConflict(tuple.agentAIndex, tuple.agentBIndex, tuple.conflictTime, groups);
            conflict.willCostIncreaseForAgentA = CbsConflict.WillCostIncrease.NO;
            conflict.willCostIncreaseForAgentB = CbsConflict.WillCostIncrease.NO;
            yield return conflict;
        }

        Trace.Assert(NotCardinalMaybeSemi.Count == 0);
        Trace.Assert(NotCardinalNotSemi.Count == 0);
        Trace.Assert(SemiCardinal.Count == 0);
        Trace.Assert(PossiblyCardinalFirstHasMddSecondDoesNotButCan.Count == 0);
        Trace.Assert(PossiblyCardinalFirstHasMddSecondCannot.Count == 0);
        Trace.Assert(PossiblyCardinalBothCannotBuildMdd.Count == 0);
        Trace.Assert(PossiblyCardinalFirstCanBuildMdd.Count == 0);
        Trace.Assert(AgentIndexesWaitingToCheckTheirConflictsForCardinality.Count == 0);
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
        FindConflicting(aConflictingGroupMemberIndex, bConflictingGroupMemberIndex, time,
                                out specificConflictingAgentA, out specificConflictingAgentB,
                                groups);
        ProblemInstance problem = CBS.GetProblemInstance();
        int initialTimeStep = problem.agents[0].lastMove.Time; // To account for solving partially solved problems.
        // This assumes the makespan of all the agents is the same.
        Move first = SingleAgentPlans[specificConflictingAgentA].GetLocationAt(time);
        Move second = SingleAgentPlans[specificConflictingAgentB].GetLocationAt(time);
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
        if (CBS.MergeThreshold == -1) // An optimization for CBS. We assume they collide.
        {
            a = aConflictingGroupMemberIndex;
            b = bConflictingGroupMemberIndex;
            return;
        }

        ISet<int> groupA;
        ISet<int> groupB;

        if (groups == null)
        {
            groupA = GetGroup(aConflictingGroupMemberIndex);
            groupB = GetGroup(bConflictingGroupMemberIndex);
        }
        else
        {
            groupA = groups[aConflictingGroupMemberIndex];
            groupB = groups[bConflictingGroupMemberIndex];
        }

        _largerConflictingGroupSize = Math.Max(groupA.Count, groupB.Count);  // TODO: explain why

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
                if (SingleAgentPlans[varA].IsColliding(time, SingleAgentPlans[varB]))
                {
                    a = varA;
                    b = varB;
                    return;
                }
            }
        }

        // A conflict should have been found
        DebugPrint();
        throw new Exception("Conflict not found");
    }

    /// <summary>
    /// Copy the MDD down the CT branch from an ancestor with an MDD of the same cost.
    /// Delete nodes from it as necessary.
    /// </summary>
    /// <returns>True if an appropriate MDD was found and copied,
    /// or if an MDD of the same cost was adapted</returns>
    private bool CopyAppropriateMddFromParent(int agentIndex)
    {
        int targetCost = SingleAgentCosts[agentIndex];
        CbsNode node = this;
        CbsNode ancestorWithMddOfSameCost = null;
        Stack<CbsNode> stack = new();
        while (node != null)
        {
            if (node._mdds[agentIndex] != null)
            {
                if (node._mdds[agentIndex].cost == targetCost)
                {
                    ancestorWithMddOfSameCost = node;
                    break;
                }
                else
                {
                    stack.Clear();
                    break;
                }
            }
            stack.Push(node);
            node = node.Prev;
        }
        if (ancestorWithMddOfSameCost != null)
        {
            // Copy the MDD down the CT branch, deleting nodes as necessary when constraints
            // make them invalid.
            MDD mdd = ancestorWithMddOfSameCost._mdds[agentIndex];
            Dictionary<int, MDD.LevelNarrowness> mddValues = ancestorWithMddOfSameCost.MDDNarrownessValues[agentIndex];
            while (stack.Count > 0)
            {
                CbsNode nodeToGiveAnMdd = stack.Pop();
                if (nodeToGiveAnMdd._constraint != null &&
                    AgentNumToIndex[nodeToGiveAnMdd._constraint.agentNum] == agentIndex)
                {
                    double startTime = CBS._stopwatch.ElapsedMilliseconds;
                    mdd = new MDD(mdd, nodeToGiveAnMdd._constraint);
                    mddValues = mdd.getLevelNarrownessValues();
                    double endTime = CBS._stopwatch.ElapsedMilliseconds;
                    CBS.TimeBuildingMdds += endTime - startTime;
                    CBS.MDDsAdapted++;
                    if (CBS.CacheMdds)
                    {
                        CbsCacheEntry entry = new(nodeToGiveAnMdd, agentIndex);
                        CBS.MDDCache[agentIndex][entry] = mdd;
                        CBS.MDDNarrownessValuesCache[agentIndex][entry] = mddValues;
                    }
                }
                nodeToGiveAnMdd._mdds[agentIndex] = mdd;
                nodeToGiveAnMdd.MDDNarrownessValues[agentIndex] = mddValues;
            }
            return true;
        }
        else
            stack.Clear();
        return false;
    }

    /// <summary>
    /// Builds an MDD for the specified agent with its current cost
    /// </summary>
    /// <param name="agentIndex"></param>
    /// <returns>Whether an MDD was built</returns>
    public bool buildMddForAgentWithItsCurrentCost(int agentIndex)
    {
        if (MDDNarrownessValues[agentIndex] != null)  // Already have an MDD with the current cost (they're nulled when the cost increases)
            return false;

        if (!CBS.CacheMdds || !CBS.MDDCache[agentIndex].ContainsKey(new CbsCacheEntry(this, agentIndex)))
        {
            // Caching not enabled or no cache hit
            if (CopyAppropriateMddFromParent(agentIndex))
                return false;  // Not built, only copied from an ancestor

            // Build the MDD
            // TODO: Code dup with Replan, Solve
            ProblemInstance problem = CBS.GetProblemInstance();
            HashSet<CbsConstraint> newConstraints = GetConstraints();
            ISet<CbsConstraint> constraints = null;
            if (CBS.ExternalConstraints != null)
            {
                constraints = new HashSet_U<CbsConstraint>();
                ((HashSet_U<CbsConstraint>)constraints).Join(CBS.ExternalConstraints);
                ((HashSet_U<CbsConstraint>)constraints).Join(newConstraints);
            }
            else
                constraints = newConstraints;

            ISet<CbsConstraint> positiveConstraints = null;
            HashSet<CbsConstraint> newPositiveConstraints = null;
            if (CBS.DoMalte)
                newPositiveConstraints = GetPositiveConstraints();
            if (CBS.ExternalPositiveConstraints != null && CBS.ExternalPositiveConstraints.Count != 0 &&
                newPositiveConstraints != null && newPositiveConstraints.Count != 0)
            {
                positiveConstraints = new HashSet_U<CbsConstraint>();
                ((HashSet_U<CbsConstraint>)positiveConstraints).Join(CBS.ExternalPositiveConstraints);
                ((HashSet_U<CbsConstraint>)positiveConstraints).Join(newPositiveConstraints);
            }
            else if (CBS.ExternalPositiveConstraints != null && CBS.ExternalPositiveConstraints.Count != 0)
                positiveConstraints = CBS.ExternalPositiveConstraints;
            else if (newPositiveConstraints != null && newPositiveConstraints.Count != 0)
                positiveConstraints = newPositiveConstraints;

            int depth = SingleAgentCosts.Max();

            IEnumerable<CbsConstraint> myConstraints = constraints.Where(
                constraint => constraint.agentNum == problem.agents[agentIndex].agent.agentNum);
            if (myConstraints.Count() != 0)
            {
                int maxConstraintTimeStep = myConstraints.Max(constraint => constraint.time);
                depth = Math.Max(depth, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
            }
            if (positiveConstraints != null && positiveConstraints.Count != 0)
            {
                IEnumerable<CbsConstraint> myMustConstraints = positiveConstraints.Where(
                    constraint => constraint.agentNum == problem.agents[agentIndex].agent.agentNum);
                if (myMustConstraints.Count() != 0)
                {
                    int maxMustConstraintTimeStep = myMustConstraints.Max(constraint => constraint.time);
                    depth = Math.Max(depth, maxMustConstraintTimeStep); // Give all must constraints a chance to affect the plan
                }
            }

            Debug.WriteLine($"Building MDD for agent index {agentIndex} of cost {SingleAgentCosts[agentIndex]} and depth {depth}");

            double startTime = CBS._stopwatch.ElapsedMilliseconds;
            _mdds[agentIndex] = new MDD(agentIndex, problem.agents[agentIndex].agent.agentNum,
                                            problem.agents[agentIndex].GetMove(), SingleAgentCosts[agentIndex],
                                            depth, problem.GetNumOfAgents(), problem,
                                            ignoreConstraints: false, supportPruning: false,
                                            constraints: constraints, positiveConstraints: positiveConstraints);
            MDDNarrownessValues[agentIndex] = _mdds[agentIndex].getLevelNarrownessValues();
            double endTime = CBS._stopwatch.ElapsedMilliseconds;
            CBS.TimeBuildingMdds += endTime - startTime;
            if (CBS.CacheMdds)
            {
                CbsCacheEntry entry = new(this, agentIndex);
                CBS.MDDCache[agentIndex][entry] = _mdds[agentIndex];
                CBS.MDDNarrownessValuesCache[agentIndex][entry] = MDDNarrownessValues[agentIndex];
            }
            CBS.MDDsBuilt++;
        }
        else
        {
            // The MDD is in the cache!
            CbsCacheEntry entry = new(this, agentIndex);
            _mdds[agentIndex] = CBS.MDDCache[agentIndex][entry];
            MDDNarrownessValues[agentIndex] = CBS.MDDNarrownessValuesCache[agentIndex][entry];
            CBS.MDDCacheHits++;
        }

        // Copy the MDD up to ancestors where appropriate
        CbsNode node = this;
        while (node != null)
        {
            node.MDDNarrownessValues[agentIndex] = MDDNarrownessValues[agentIndex];
            if (node._constraint != null &&
                AgentNumToIndex[node._constraint.agentNum] == agentIndex)
                break;  // This is where the last contraint on the agent was added.
                        // Ancestors will have inappropriate MDDs for this agent - no need to check them.
            node = node.Prev;
        }

        return true;
    }

    private (int groupRepA, int groupRepB, int time) GetFirstConflictDetails()
    {
        int groupRepA = -1; // To quiet the compiler
        int groupRepB = -1; // To quiet the compiler
        int time = int.MaxValue;
        for (int i = 0; i < ConflictTimesPerAgent.Length; i++)
        {
            foreach (var otherAgentNumAndConflictTimes in ConflictTimesPerAgent[i])
            {
                if (otherAgentNumAndConflictTimes.Value[0] < time)
                {
                    time = otherAgentNumAndConflictTimes.Value[0];
                    groupRepA = i;
                    groupRepB = AgentNumToIndex[otherAgentNumAndConflictTimes.Key];
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
        Func<int, double> formula = i => _countsOfInternalAgentsThatConflict[i] / ((double)(1 << (GetGroupSize(i) - 1)));

        int chosenAgentIndex = Enumerable.Range(0, SingleAgentPlans.Length).MaxByKeyFunc(formula);

        // We could just look for any of this agent's conflicts,
        // but the best choice among the agents it conflicts with is the one which maximizes the formula itself.
        IEnumerable<int> conflictsWithAgentNums = ConflictCountsPerAgent[chosenAgentIndex].Keys;
        IEnumerable<int> conflictsWithInternallyAgentNums = conflictsWithAgentNums.Where(agentNum => AgentNumToIndex.ContainsKey(agentNum));
        IEnumerable<int> conflictsWithInternallyAgentIndices = conflictsWithInternallyAgentNums.Select(agentNum => AgentNumToIndex[agentNum]);
        int chosenConflictingAgentIndex = conflictsWithInternallyAgentIndices.MaxByKeyFunc(formula);

        groupRepA = chosenAgentIndex;
        groupRepB = chosenConflictingAgentIndex;

        ProblemInstance problem = CBS.GetProblemInstance();
        time = ConflictTimesPerAgent[chosenAgentIndex] // Yes, the index of the first and the num of the second
                                                [problem.agents[chosenConflictingAgentIndex].agent.agentNum][0];
        return (groupRepA, groupRepB, time);
    }

    public CbsConflict GetConflict()
    {
        return Conflict;
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
        Trace.Assert(G == child.G, "Tried to adopt node of a different cost");
        AgentAExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
        AgentBExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
        SingleAgentCosts = child.SingleAgentCosts;
        SingleAgentPlans = child.SingleAgentPlans;
        Conflict = child.Conflict;  // Probably null, see below
        isGoal = child.isGoal;
        _countsOfInternalAgentsThatConflict = child._countsOfInternalAgentsThatConflict;
        ConflictCountsPerAgent = child.ConflictCountsPerAgent;
        ConflictTimesPerAgent = child.ConflictTimesPerAgent;
        _totalExternalAgentsThatConflict = child._totalExternalAgentsThatConflict;
        MinOpsToSolve = child.MinOpsToSolve;
        TotalInternalAgentsThatConflict = child.TotalInternalAgentsThatConflict;
        TotalConflictsWithExternalAgents = child.TotalConflictsWithExternalAgents;
        TotalConflictsBetweenInternalAgents = child.TotalConflictsBetweenInternalAgents;
        _largerConflictingGroupSize = child._largerConflictingGroupSize;
        for (int i = 0; i < child.NewPlans.Length; i++)
        {
            NewPlans[i] = NewPlans[i] || child.NewPlans[i];
        }
        // We don't adopt the child's constraints, nor its agents groups assignment
        // mdds is kept unchanged too since the cost of the replanned (meta-)agent
        // didn't change and we added no constraints.

        ChooseConflict();  // child probably hasn't chosen a conflict (and will never get a chance to),
                                // need to choose the new conflict to work on.
                                // (if child somehow had a conflict already, ChooseConflict does nothing)
                                // We can't just continue the node's conflict iteration since
                                // some conflicts may have been eliminated by the new plans
    }

    /// <summary>
    /// Uses the group assignments and the constraints (ignoring their order).
    /// Irrelevant constraints stemming from conflicts between merged agents are ignored.
    /// </summary>
    /// <returns></returns>
    public override int GetHashCode()
    {
        unchecked
        {
            int ans = 0;
            for (int i = 0; i < AgentsGroupAssignment.Length; i++)
            {
                ans += Constants.PRIMES_FOR_HASHING[i % Constants.PRIMES_FOR_HASHING.Length] * AgentsGroupAssignment[i];
            }

            HashSet<CbsConstraint> constraints = GetConstraints();

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

        if (AgentsGroupAssignment.SequenceEqual(other.AgentsGroupAssignment) == false)
            return false;

        CbsNode current = this;
        HashSet<CbsConstraint> other_constraints = other.GetConstraints();
        HashSet<CbsConstraint> constraints = GetConstraints();

        foreach (CbsConstraint constraint in constraints)
        {
            if (other_constraints.Contains(constraint) == false)
                return false;
            current = current.Prev;
        }
        // TODO: Consider replacing the above foreach with constraints.IsSubsetOf(other_constraints)

        return constraints.Count == other_constraints.Count;
    }

    /// <summary>
    /// Worth doing because the node may always be in the closed list
    /// </summary>
    public void Clear()
    {
        SingleAgentPlans = null;
        SingleAgentCosts = null;
        _countsOfInternalAgentsThatConflict = null;
        ConflictCountsPerAgent = null;
        ConflictTimesPerAgent = null;
        AgentNumToIndex = null;
    }

    public int CompareTo(IBinaryHeapItem item)
    {
        CbsNode other = (CbsNode)item;

        if (F < other.F)
            return -1;
        if (F > other.F)
            return 1;

        return TieBreak(other);
    }

    public int TieBreak(CbsNode other, bool ignorePartialExpansion = false, bool ignoreDepth = false)
    {
        // Tie breaking:

        // Prefer fewer external conflicts, even over goal nodes, as goal nodes with less external conflicts are better.
        // External conflicts are also taken into account by the low level solver to prefer fewer conflicts between fewer agents.
        // This only helps when this CBS is used as a low level solver, of course.
        if (_totalExternalAgentsThatConflict < other._totalExternalAgentsThatConflict)
            return -1;
        if (_totalExternalAgentsThatConflict > other._totalExternalAgentsThatConflict)
            return 1;

        if (TotalConflictsWithExternalAgents < other.TotalConflictsWithExternalAgents)
            return -1;
        if (TotalConflictsWithExternalAgents > other.TotalConflictsWithExternalAgents)
            return 1;
            
        // Prefer goal nodes. The elaborate form is to keep the comparison consistent. Without it goalA<goalB and also goalB<goalA.
        if (GoalTest() == true && other.GoalTest() == false)
            return -1;
        if (other.GoalTest() == true && GoalTest() == false)
            return 1;

        // Prefer larger cost? Higher h means more work needs to be done, but lower h sometimes
        // means work needs to be done to discover dependencies between agents which would then
        // increase the h
        //if (g > other.g)
        //    return -1;
        //if (g < other.g)
        //    return 1;

        // Prefer nodes which would possibly require less work.
        // Remember replans and merges don't necessarily enlarge the total cost, so the number of operations needed to solve
        // sadly can't be added to the node's total cost.
        if (!CBS.DisableTieBreakingByMinOpsEstimate)
        {
            if (MinOpsToSolve < other.MinOpsToSolve)
                return -1;
            if (MinOpsToSolve > other.MinOpsToSolve)
                return 1;
        }
        else
        {
            if (TotalInternalAgentsThatConflict < other.TotalInternalAgentsThatConflict)
                return -1;
            if (TotalInternalAgentsThatConflict > other.TotalInternalAgentsThatConflict)
                return 1;
        }

        // Prefer fewer internal conflicts if the minOpsToSolve is the same (or turned off)
        // More conflicts - bigger chance some of them are cardinal (in case they weren't checked already).
        if (TotalConflictsBetweenInternalAgents < other.TotalConflictsBetweenInternalAgents)
            return -1;
        if (TotalConflictsBetweenInternalAgents > other.TotalConflictsBetweenInternalAgents)
            return 1;

        // If same number of internal conflicts and agents that conflict - prefer more depth.
        // More work was done on deeper nodes, so they're more probable to either finally resolve
        // a dependency between agents or find a cardinal conflict
        if (ignoreDepth == false)
        {
            if (_depth > other._depth)
                return -1;
            if (_depth < other._depth)
                return 1;
        }

        if (ignorePartialExpansion == false)
        {
            // Prefer partially expanded nodes, in addition to their H bonus.
            // They're less work because they have less constraints and only one child to generate.
            // The elaborate form, again, is to keep the comparison consistent. Without it partiallyExpandedA<partiallyExpandedB and partiallyExpandedA>partiallyExpandedB
            if ((AgentAExpansion == CbsNode.ExpansionState.DEFERRED || AgentBExpansion == CbsNode.ExpansionState.DEFERRED) &&
                other.AgentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED && other.AgentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED)
                return -1;
            if ((other.AgentAExpansion == CbsNode.ExpansionState.DEFERRED || other.AgentBExpansion == CbsNode.ExpansionState.DEFERRED) &&
                AgentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED && AgentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED)
                return 1;
        }

        // Prefer nodes with conflicts between smaller groups of agents (irrelevant for goal nodes)
        // This requires that the conflict be chosen already, but we defer choosing the conflict to when
        // it comes *out* of OPEN, as choosing the conflict may be expensive.
        // TODO: enable this when both nodes have a chosen conflict. Nodes can be re-entered
        // into OPEN.
        //if (largerConflictingGroupSize < other.largerConflictingGroupSize)
        //    return -1;
        //if (largerConflictingGroupSize > other.largerConflictingGroupSize)
        //    return 1;

        return 0;
    }

    /// <summary>
    /// Not used.
    /// </summary>
    /// <returns></returns>
    public CbsConstraint GetLastConstraint() => _constraint;

    public HashSet<CbsConstraint> GetConstraints()
    {
        HashSet<CbsConstraint> constraints = [];
        CbsNode current = this;
        while (current._depth > 0) // The root has no constraints
        {
            if (current._constraint != null && // Last check not enough if "surprise merges" happen (merges taken from adopted child)
                current.Prev.Conflict != null && // Can only happen for temporary lookahead nodes that were created and then
                                                    // later the parent adopted a goal node
                AgentsGroupAssignment[current.Prev.Conflict.agentAIndex] !=
                AgentsGroupAssignment[current.Prev.Conflict.agentBIndex]) // Ignore constraints that deal with conflicts between
                                                                                // agents that were later merged. They're irrelevant
                                                                                // since merging fixes all conflicts between merged agents.
                                                                                // Nodes that only differ in such irrelevant conflicts will have the same single agent paths.
                                                                                // Dereferencing current.prev is safe because current isn't the root.
                                                                                // Also, merging creates a non-root node with a null constraint, and this helps avoid adding the null to the answer.
                constraints.Add(current._constraint);
            current = current.Prev;
        }
        return constraints;
    }

    /// <summary>
    /// For printing
    /// </summary>
    /// <returns></returns>
    public List<CbsConstraint> GetConstraintsOrdered()
    {
        List<CbsConstraint> constraints = [];
        CbsNode current = this;
        while (current._depth > 0) // The root has no constraints
        {
            if (current._constraint != null && // Next check not enough if "surprise merges" happen (merges taken from adopted child)
                current.Prev.Conflict != null && // Can only happen for temporary lookahead nodes the were created and then later the parent adopted a goal node
                AgentsGroupAssignment[current.Prev.Conflict.agentAIndex] !=
                AgentsGroupAssignment[current.Prev.Conflict.agentBIndex]) // Ignore constraints that deal with conflicts between
                // agents that were later merged. They're irrelevant
                // since merging fixes all conflicts between merged agents.
                // Nodes that only differ in such irrelevant conflicts will have the same single agent paths.
                // Dereferencing current.prev is safe because current isn't the root.
                // Also, merging creates a non-root node with a null constraint, and this helps avoid adding the null to the answer.
                constraints.Add(current._constraint);
            current = current.Prev;
        }
        return constraints;
    }

    public HashSet<CbsConstraint> GetPositiveConstraints()
    {
        HashSet<CbsConstraint> constraints = [];
        CbsNode current = this;
        while (current._depth > 0)
        {
            if (current._mustConstraint != null) // TODO: Ignore positive constraints from merged agents
                constraints.Add(current._mustConstraint);
            current = current.Prev;
        }
        return constraints;
    }

    /// <summary>
    /// IBinaryHeapItem implementation
    /// </summary>
    /// <returns></returns>
    public int GetIndexInHeap()  => _binaryHeapIndex;

    /// <summary>
    /// IBinaryHeapItem implementation
    /// </summary>
    /// <returns></returns>
    public void SetIndexInHeap(int index)  => _binaryHeapIndex = index;

    public Plan CalculateJointPlan() => new Plan(SingleAgentPlans);

    /// <summary>
    /// Check if the agent groups that participate in the conflict of this node should be merged.
    /// The merge criterion is the old one - counts how many times a conflict between the agents was chosen
    /// as the conflict to resolve along the branch to this node.
    /// </summary>
    /// <param name="mergeThreshold"></param>
    /// <returns>Whether to merge.</returns>
    public bool ShouldMerge(int mergeThreshold)
    {
        int conflictsCount = 1; // The agentA and agentB conflict in this node.
        ISet<int> firstGroup = GetGroup(Conflict.agentAIndex);
        ISet<int> secondGroup = GetGroup(Conflict.agentBIndex);

        CbsNode current = Prev;
        int a, b;
        while (current != null)
        {
            a = current.Conflict.agentAIndex;
            b = current.Conflict.agentBIndex;
            if ((firstGroup.Contains(a) && secondGroup.Contains(b)) || (firstGroup.Contains(b) && secondGroup.Contains(a)))
                conflictsCount++;
            current = current.Prev;
        }

        return conflictsCount > mergeThreshold;
    }

    /// <summary>
    /// New merge criterion: counts how many times the agents conflicted in a new plan.
    /// </summary>
    /// <param name="mergeThreshold"></param>
    /// <param name="agentAIndex"></param>
    /// <param name="agentBIndex"></param>
    /// <returns></returns>
    public bool ShouldMerge(int mergeThreshold, int agentAIndex, int agentBIndex)
    {
        int conflictsCount = 0;
        ISet<int> firstGroup = GetGroup(agentAIndex);
        ISet<int> secondGroup = GetGroup(agentBIndex);
        ProblemInstance problem = CBS.GetProblemInstance();

        CbsNode current = this;
        while (current != null)
        {
            for (int i = 0; i < ConflictCountsPerAgent.Length; i++)
            {
                if (firstGroup.Contains(i) == false && secondGroup.Contains(i) == false)
                    continue;
                foreach (var kvp in ConflictCountsPerAgent[i])
                {
                    if (i > kvp.Key)
                        continue;  // Count each conflict once
                    if (NewPlans[i] == false && NewPlans[kvp.Key] == false)
                        continue;  // Only count conflicts of new plans
                    if (firstGroup.Contains(i) || secondGroup.Contains(i))
                        conflictsCount += kvp.Value;
                }
            }

            current = current.Prev;
        }


        // TODO: Make a permanent branchConflictCounts instead of computing this every time?
        //       OTOH, I compute this ~once per agent pair per node.
        return conflictsCount > mergeThreshold;
    }

    /// <summary>
    /// Check if the agent groups that participate in the conflict of this node pass the merge threshold,
    /// using the given conflict counts. 
    /// The merge criterion is the old one - count how many times a conflict between the agents was chosen
    /// as the conflict to resolve in the nodes of the tree.
    /// </summary>
    /// <param name="mergeThreshold"></param>
    /// <param name="globalConflictCounter"></param>
    /// <returns>Whether to merge.</returns>
    public bool ShouldMerge(int mergeThreshold, int[][] globalConflictCounter)
    {
        int conflictCounter = 0;
        ISet<int> firstGroup = GetGroup(Conflict.agentAIndex);
        ISet<int> secondGroup = GetGroup(Conflict.agentBIndex);

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
    /// 
    /// </summary>
    /// <param name="mergeThreshold"></param>
    /// <param name="agentAIndex"></param>
    /// <param name="agentBIndex"></param>
    /// <returns></returns>
    public bool ShouldMerge(int mergeThreshold, int[][] globalConflictCounter, int agentAIndex, int agentBIndex)
    {
        int conflictsCount = 0;
        ISet<int> firstGroup = GetGroup(agentAIndex);
        ISet<int> secondGroup = GetGroup(agentBIndex);
        ProblemInstance problem = CBS.GetProblemInstance();

        for (int i = 0; i < ConflictCountsPerAgent.Length; i++)
        {
            if (firstGroup.Contains(i) == false && secondGroup.Contains(i) == false)
                continue;
            foreach (var kvp in ConflictCountsPerAgent[i])
            {
                if (i > kvp.Key)
                    continue;  // Count each conflict once
                if (NewPlans[i] == false && NewPlans[kvp.Key] == false)
                    continue;  // Only count conflicts of new plans
                if (firstGroup.Contains(i) || secondGroup.Contains(i))
                {
                    conflictsCount += globalConflictCounter[kvp.Key][i];
                }
            }
        }

        return conflictsCount > mergeThreshold;
    }

    /// <summary>
    /// Returns a list of indices of agents in the group
    /// </summary>
    /// <param name="agentIndex"></param>
    /// <returns></returns>
    public ISet<int> GetGroup(int agentIndex)
    {
        int groupNumber = AgentsGroupAssignment[agentIndex];
        SortedSet<int> group = [];

        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
        {
            if (AgentsGroupAssignment[i] == groupNumber)
                group.Add(i);
        }
        return group;
    }

    /// <summary>
    /// Get the combined cost for the group, either for the sum-of-costs or makespan variant.
    /// </summary>
    /// <param name="groupNumber"></param>
    /// <returns></returns>
    public int GetGroupCost(int groupNumber)
    {
        if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
        {
            int cost = 0;
            for (int i = 0; i < AgentsGroupAssignment.Length; i++)
            {
                if (AgentsGroupAssignment[i] == groupNumber)
                    cost += SingleAgentCosts[i];
            }
            return cost;
        }
        else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
            Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
        {
            int cost = 0;
            for (int i = 0; i < AgentsGroupAssignment.Length; i++)
            {
                if (AgentsGroupAssignment[i] == groupNumber)
                    if (SingleAgentCosts[i] > cost)
                        cost = SingleAgentCosts[i];
            }
            return cost;
        }
        else
            throw new NotImplementedException($"Unsupported cost function {Constants.costFunction}");
    }

    /// <summary>
    /// A bit cheaper than GetGroup(n).Count. Still O(n).
    /// </summary>
    /// <param name="agentIndex"></param>
    /// <returns></returns>
    public int GetGroupSize(int agentIndex)
    {
        int groupNumber = AgentsGroupAssignment[agentIndex];
        int count = 0;

        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
        {
            if (AgentsGroupAssignment[i] == groupNumber)
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
        Span<int> counts = stackalloc int[AgentsGroupAssignment.Length];

        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
            counts[AgentsGroupAssignment[i]]++;

        int[] groupSizes = new int[AgentsGroupAssignment.Length];

        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
            groupSizes[i] = counts[AgentsGroupAssignment[i]];
            
        return groupSizes;
    }

    /// <summary>
    /// In O(n)
    /// </summary>
    /// <returns></returns>
    public ISet<int>[] GetGroups()
    {
        Dictionary<int, ISet<int>> repsToGroups = [];

        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
        {
            int groupRep = AgentsGroupAssignment[i];
            if (repsToGroups.ContainsKey(groupRep))
                repsToGroups[groupRep].Add(i);
            else
            {
                HashSet<int> newGroup = [];
                newGroup.Add(i);
                repsToGroups[groupRep] = newGroup;

            }
        }

        ISet<int>[] res = new HashSet<int>[AgentsGroupAssignment.Length];
        for (int i = 0; i < res.Length; i++)
            res[i] = repsToGroups[AgentsGroupAssignment[i]];

        return res;
    }

    /// <summary>
    /// Updates the agentsGroupAssignment and the conflictCountsPerAgent. Warning: changes the hash!
    /// </summary>
    /// <param name="a">Index of first group representative</param>
    /// <param name="b">Index of second group representative</param>
    /// <param name="fixCounts">Can be set to false as an optimization when counts aren't needed</param>
    public void MergeGroups(int a, int b, bool fixCounts = true)
    {
        if (b < a)
            (a, b) = (b, a);

        ProblemInstance problem = CBS.GetProblemInstance();
        int aAgentNum = problem.agents[a].agent.agentNum;
        int bAgentNum = problem.agents[b].agent.agentNum;

        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
        {
            if (AgentsGroupAssignment[i] == b)
            {
                AgentsGroupAssignment[i] = (ushort)a;
            }
        }

        if (fixCounts)
        {
            ConflictCountsPerAgent[a].Remove(bAgentNum); // It isn't really necessary to update the conflictCountPerAgent dictionaries of the merged agents -
            // they're about to be replanned and have their dictionaries updated anyway
            ConflictTimesPerAgent[a].Remove(bAgentNum);
            ConflictCountsPerAgent[b].Clear();
            ConflictTimesPerAgent[b].Clear();
            for (int i = 0; i < ConflictCountsPerAgent.Length; i++)
            {
                if (ConflictCountsPerAgent[i].ContainsKey(bAgentNum))
                {
                    if (ConflictCountsPerAgent[i].ContainsKey(aAgentNum) == false)
                        ConflictCountsPerAgent[i][aAgentNum] = 0;
                    ConflictCountsPerAgent[i][aAgentNum] += ConflictCountsPerAgent[i][bAgentNum];
                    ConflictCountsPerAgent[i].Remove(bAgentNum);

                    if (ConflictTimesPerAgent[i].ContainsKey(aAgentNum) == false)
                        ConflictTimesPerAgent[i][aAgentNum] = new List<int>(ConflictTimesPerAgent[i][bAgentNum].Count);
                    ConflictTimesPerAgent[i][aAgentNum].AddRange(ConflictTimesPerAgent[i][bAgentNum]);
                    ConflictTimesPerAgent[i].Remove(bAgentNum);
                }
            }
        }
    }

    public void PrintConflict()
    {
        if (Conflict != null)
        {
            Debug.WriteLine("Conflict:");
            Debug.WriteLine("Agents:({0},{1})", Conflict.agentAIndex, Conflict.agentBIndex);
            Debug.WriteLine("Location:({0},{1})", Conflict.agentAmove.X, Conflict.agentAmove.Y);
            Debug.WriteLine("Time:{0}", Conflict.timeStep);
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
        List<Move> moves = SingleAgentPlans[agent].LocationAtTimes;
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
            if (current._mustConstraint != null && !current._mustConstraint.Allows(check))
                return false;
            current = current.Prev;
        }
        return true;
    }

    public void SetMustConstraint(CbsConstraint set)
    {
        _mustConstraint = set;
    }

    private bool isGoal = false;

    public bool GoalTest() {
        if (G < CBS.MinSolutionCost)
            return false;
        return isGoal;
    }

    /// <summary>
    /// For CBS IDA* only.
    /// TODO: Consider inheriting from CbsNode and overriding the Replan method instead.
    /// </summary>
    /// <returns>Whether a path was successfully found</returns>
    public bool Replan3b(int agentToReplan, int depthToReplan, int minPathCost = -1,
                            int maxPathCost = int.MaxValue)
    {
        ProblemInstance problem = CBS.GetProblemInstance();

        var internalCAT = new ConflictAvoidanceTable();
        ConflictAvoidanceTable CAT;
        if (CBS.ExternalCAT != null)
        {
            CAT = new CAT_U();
            ((CAT_U)CAT).Join(CBS.ExternalCAT);
            ((CAT_U)CAT).Join(internalCAT);
        }
        else
            CAT = internalCAT;


        HashSet<CbsConstraint> newConstraints = GetConstraints();
        ISet<CbsConstraint> constraints;
        if (CBS.ExternalConstraints != null)
        {
            constraints = new HashSet_U<CbsConstraint>();
            ((HashSet_U<CbsConstraint>)constraints).Join(CBS.ExternalConstraints);
            ((HashSet_U<CbsConstraint>)constraints).Join(newConstraints);
        }
        else
            constraints = newConstraints;


        ISet<CbsConstraint> positiveConstraints = null;
        HashSet<CbsConstraint> newPositiveConstraints = null;
        if (CBS.DoMalte)
            newPositiveConstraints = GetPositiveConstraints();
        if (CBS.ExternalPositiveConstraints != null && CBS.ExternalPositiveConstraints.Count != 0 &&
            newPositiveConstraints != null && newPositiveConstraints.Count != 0)
        {
            positiveConstraints = new HashSet_U<CbsConstraint>();
            ((HashSet_U<CbsConstraint>)positiveConstraints).Join(CBS.ExternalPositiveConstraints);
            ((HashSet_U<CbsConstraint>)positiveConstraints).Join(newPositiveConstraints);
        }
        else if (CBS.ExternalPositiveConstraints != null && CBS.ExternalPositiveConstraints.Count != 0)
            positiveConstraints = CBS.ExternalPositiveConstraints;
        else if (newPositiveConstraints != null && newPositiveConstraints.Count != 0)
            positiveConstraints = newPositiveConstraints;

        if (newConstraints.Count != 0)
        {
            int maxConstraintTimeStep = newConstraints.Max<CbsConstraint>(constraint => constraint.time);
            depthToReplan = Math.Max(depthToReplan, maxConstraintTimeStep); // Give all constraints a chance to affect the plan
        }

        List<AgentState> subGroup = [];
        int groupNum = AgentsGroupAssignment[agentToReplan];
        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
        {
            if (AgentsGroupAssignment[i] == groupNum)
                subGroup.Add(problem.agents[i]);
            else
                internalCAT.AddPlan(SingleAgentPlans[i]);
        }

        ReplanSize = (ushort)subGroup.Count;

        ICbsSolver relevantSolver = _solver;
        if (subGroup.Count == 1)
            relevantSolver = _singleAgentSolver;

        ProblemInstance subProblem = problem.Subproblem(subGroup.ToArray());
        subProblem.parameters = problem.parameters;

        MDD mdd = null;
        if (CBS.ReplanSameCostWithMdd)
            mdd = _mdds[agentToReplan];
        double startTime = CBS._stopwatch.ElapsedMilliseconds;
        relevantSolver.Setup(subProblem, depthToReplan, CBS._stopwatch, CAT, constraints, positiveConstraints,
                                minPathCost, maxPathCost, mdd);
        bool solved = relevantSolver.Solve();
        double endTime = CBS._stopwatch.ElapsedMilliseconds;
        CBS.TimePlanningPaths += endTime - startTime;

        relevantSolver.AccumulateStatistics();
        relevantSolver.ClearStatistics();

        if (solved == false)
            return false;

        int j = 0;
        SinglePlan[] singlePlans = relevantSolver.GetSinglePlans();
        int[] singleCosts = relevantSolver.GetSingleCosts();
        for (int i = 0; i < AgentsGroupAssignment.Length; i++)
        {
            if (AgentsGroupAssignment[i] == groupNum)
            {
                SingleAgentPlans[i] = singlePlans[j];
                SingleAgentPlans[i].AgentNum = problem.agents[groupNum].agent.agentNum; // Use the group's representative
                SingleAgentCosts[i] = singleCosts[j];
                j++;
            }
        }
        Trace.Assert(j == ReplanSize);

        // Calc g
        if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
        {
            G = (ushort)SingleAgentCosts.Sum();
        }
        else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
            Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
        {
            G = (ushort)SingleAgentCosts.Max();
        }
        else
            throw new NotImplementedException($"Unsupported cost function {Constants.costFunction}");

        // PrintPlan();

        isGoal = _countsOfInternalAgentsThatConflict.All(i => i == 0);
        //ChooseConflict(); 

        // PrintConflict();
        return true;
    }

    /// <summary>
    /// Assumes agents conflict at the given time, and an MDD has been built for the agent
    /// </summary>
    public bool DoesAgentHaveNoOtherOption(int agentIndex, int conflictTime, int conflictingAgentIndex, ISet<int>[] groups)
    {
        bool stayingAtGoalConflict = conflictTime > MDDNarrownessValues[agentIndex].Keys.Max();  // The time step of reaching the goal must be present here, possibly together with extra later time steps the MDD was built with
        if (stayingAtGoalConflict)  // Then it must be a vertex conflict, and the agent can't have another option at same cost
            return true;
        else if (!MDDNarrownessValues[agentIndex].ContainsKey(conflictTime))
            return false;
        else if (MDDNarrownessValues[agentIndex][conflictTime] == MDD.LevelNarrowness.WIDTH_1)
            return true;
        else  // mddNarrownessValues[agentIndex][conflictTime] == ONE_LOCATION_MULTIPLE_DIRECTIONS
        {
            // If it's an edge conflict, and the MDD's width at the conflict time is 1,
            // this agent's cost will increase if split on this conflict - it's at least a semi-cardinal conflict.
            // If it's a vertex conflict, then even if the MDD's width at the conflict time is greater than 1,
            // it might still be a semi-cardinal or cardinal conflict if all of the agent's MDD's nodes at this
            // time move to the same vertex (from different directions)
            var conflict = FindConflict(agentIndex, conflictingAgentIndex, conflictTime, groups);
            bool vertexConflict = conflict.isVertexConflict;
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
    private readonly int _groupSize;
    private readonly int _degree;
    private readonly int _planCost;
    private readonly int _index;

    public AgentToCheckForCardinalConflicts(int groupSize, int degree, int planCost, int index)
    {
        _groupSize = groupSize;
        _degree = degree;
        _planCost = planCost;
        _index = index;
    }

    public int CompareTo(IBinaryHeapItem item)
    {
        AgentToCheckForCardinalConflicts other = (AgentToCheckForCardinalConflicts)item;

        if (_groupSize < other._groupSize)
            return -1;
        else if (_groupSize > other._groupSize)
            return 1;

        if (_degree < other._degree)
            return -1;
        else if (_degree > other._degree)
            return 1;

        if (_planCost < other._planCost)
            return -1;
        else if (_planCost > other._planCost)
            return 1;

        if (_index < other._index)
            return -1;
        else if (_index > other._index)
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
