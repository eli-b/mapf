using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using System.Linq;

namespace mapf;

public enum ConflictChoice : byte
{
    FIRST = 0,
    MOST_CONFLICTING_SMALLEST_AGENTS,
    CARDINAL_MDD,
    CARDINAL_LOOKAHEAD,
    CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_SMALLEST_GROUP,
    CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_AND_SMALLEST_GROUP
}

/// <summary>
/// Merges agents if they conflict more times than the given threshold in the CT nodes
/// from the root to the current CT nodes only.
/// </summary>
public class CBS : ICbsSolver, IHeuristicSolver<CbsNode>, IIndependenceDetectionSolver
{
    /// <summary>
    /// For each agent, map sets of constraints to MDDs
    /// </summary>
    public Dictionary<CbsCacheEntry, MDD>[] MDDCache { get; private set; }

    /// <summary>
    /// For each agent, map sets of constraints to a dictionary that
    /// maps each level (timestep) of its mdd to a narrowness degree.
    /// Non-narrow levels are omitted.
    /// </summary>
    public Dictionary<CbsCacheEntry, Dictionary<int, MDD.LevelNarrowness>>[] MDDNarrownessValuesCache { get; private set; }

    private ProblemInstance _instance;
    public OpenList<CbsNode> OpenList { get; }
    /// <summary>
    /// Might as well be a HashSet. We don't need to retrieve from it.
    /// TODO: Consider a closedList for single agent paths under an unordered set of constraints.
    ///       It would get more hits.
    /// </summary>
    private readonly Dictionary<CbsNode, CbsNode> _closedList = [];
    private readonly ILazyHeuristic<CbsNode> _heuristic;
    private int _highLevelExpanded;
    private int _highLevelGenerated;
    private int _closedListHits;
    private int _partialExpansions;
    private int _bypasses;
    private int _nodesExpandedWithGoalCost;
    private int _bypassLookAheadNodesCreated;
    private int _cardinalLookAheadNodesCreated;
    private int _conflictsBypassed;
    private int _cardinalConflictSplits;
    private int _semiCardinalConflictSplits;
    private int _nonCardinalConflictSplits;
    public int MDDsBuilt { get; set; }
    public int MDDsAdapted { get; set; }
    private int _restarts;
    private int _pathMaxBoosts;
    private int _reversePathMaxBoosts;
    private int _pathMaxPlusBoosts;
    private int _surplusNodesAvoided;
    public int MDDCacheHits { get; set; }
    public double TimePlanningPaths { get; set; }
    public double TimeBuildingMdds { get; set; }
    // TODO: Count shuffles
    private int _accHLExpanded;
    private int _accHLGenerated;
    private int _accClosedListHits;
    private int _accPartialExpansions;
    private int _accBypasses;
    private int _accNodesExpandedWithGoalCost;
    private int _accBypassLookAheadNodesCreated;
    private int _accCardinalLookAheadNodesCreated;
    private int _accConflictsBypassed;
    private int _accCardinalConflictSplits;
    private int _accSemiCardinalConflictSplits;
    private int _accNonCardinalConflictSplits;
    private int _accMddsBuilt;
    private int _accMddsAdapted;
    private int _accRestarts;
    private int _accPathMaxBoosts;
    private int _accReversePathMaxBoosts;
    private int _accPathMaxPlusBoosts;
    private int _accSurplusNodesAvoided;
    private int _accMddCacheHits;
    private double _accTimePlanningPaths;
    private double _accTimeBuildingMdds;

    public int SolutionCost { get; private set; }
    /// <summary>
    /// The difference between the solution's cost and the f of the root node.
    /// Notice root.g != 0 in CBS.
    /// </summary>
    private int _solutionDepth;
    public Run _runner; // TODO: remove this dependency
    private CbsNode _goalNode;
    private Plan _solution;

    /// <summary>
    /// Nodes with a higher F aren't generated. As a result, goal nodes with a higher cost
    /// won't be found.
    /// </summary>
    private int _maxSolutionCost;
    /// <summary>
    /// Goal Nodes with with a lower cost aren't considered a goal. Used directly by CbsNode.
    /// </summary>
    public int MinSolutionCost { get; private set; }
    /// <summary>
    /// Search is stopped when the minimum F in the open list reaches the target,
    /// regardless of whether a goal node was found. Note maxSolutionCost stops the search when
    /// the same F value is exhausted from the open list later.
    /// </summary>
    public int TargetF 
    {
        get => _targetF;
        set
        {
            _maxSolutionCost = Math.Min(_maxSolutionCost, value);  // No need to generate nodes with a higher F
            _targetF = value;
        }
    }
    private int _targetF;
    /// <summary>
    /// Search is stopped when the low level generated nodes count exceeds the cap
    /// </summary>
    public int LowLevelGeneratedCap { set; get; }
    /// <summary>
    /// Search is stopped when the millisecond count exceeds the cap
    /// </summary>
    public int MilliCap { set; get; }
    private ICbsSolver _solver;
    private ICbsSolver _singleAgentSolver;
    public int MergeThreshold { get; set; }
    public int MinSolutionTimeStep { get; private set; }
    private int _maxSizeGroup;
    private int _accMaxSizeGroup;
    private BypassStrategy _bypassStrategy;
    public bool DoMalte { get; private set; }
    public ConflictChoice ConflictChoice { get; private set; }
    public bool DisableTieBreakingByMinOpsEstimate { get; private set; }
    private int _lookaheadMaxExpansions;

    public enum BypassStrategy : byte
    {
        NONE = 0,
        FIRST_FIT_LOOKAHEAD,
        BEST_FIT_LOOKAHEAD
    }
    protected bool _mergeCausesRestart;
    private readonly bool _useOldCost;
    public bool ReplanSameCostWithMdd { get; private set; }
    public bool CacheMdds { get; private set; }
    private bool _useCAT;

    public ConflictAvoidanceTable ExternalCAT { get; private set; }
    public ISet<CbsConstraint> ExternalConstraints { get; private set; }
    /// <summary>
    /// Note that in this implementation, positive constraints currenly don't act as negative constraints for all other agents, regretably.
    /// They only rule out every other location for the agent at that time step.
    /// </summary>
    public ISet<CbsConstraint> ExternalPositiveConstraints { get; private set; }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="singleAgentSolver"></param>
    /// <param name="generalSolver"></param>
    /// <param name="mergeThreshold"></param>
    /// <param name="bypassStrategy"></param>
    /// <param name="doMalte"></param>
    /// <param name="conflictChoice"></param>
    /// <param name="heuristic">Assumed to be expensive to compute. Used as late as possible</param>
    /// <param name="disableTieBreakingByMinOpsEstimate"></param>
    /// <param name="lookaheadMaxExpansions"></param>
    /// <param name="mergeCausesRestart"></param>
    /// <param name="replanSameCostWithMdd"></param>
    /// <param name="cacheMdds"></param>
    /// <param name="useOldCost"></param>
    /// <param name="useCAT"></param>
    public CBS(ICbsSolver singleAgentSolver, ICbsSolver generalSolver,
                                int mergeThreshold = -1,
                                BypassStrategy bypassStrategy = BypassStrategy.NONE,
                                bool doMalte = false,
                                ConflictChoice conflictChoice = ConflictChoice.FIRST,
                                ILazyHeuristic<CbsNode> heuristic = null,
                                bool disableTieBreakingByMinOpsEstimate = true,
                                int lookaheadMaxExpansions = 1,
                                bool mergeCausesRestart = false,
                                bool replanSameCostWithMdd = false,
                                bool cacheMdds = false,
                                bool useOldCost = false,
                                bool useCAT = true
        )
    {
        if (heuristic == null)
            OpenList = new OpenList<CbsNode>(this);
        else
            OpenList = new DynamicLazyOpenList<CbsNode>(this, heuristic);
        MergeThreshold = mergeThreshold;
        _solver = generalSolver;
        _singleAgentSolver = singleAgentSolver;
        _bypassStrategy = bypassStrategy;
        DoMalte = doMalte;
        ConflictChoice = conflictChoice;
        _heuristic = heuristic;
        if (Constants.costFunction != Constants.CostFunction.SUM_OF_COSTS)
        {
            Trace.Assert(conflictChoice != ConflictChoice.CARDINAL_MDD &&
                         conflictChoice != ConflictChoice.CARDINAL_LOOKAHEAD &&
                         conflictChoice != ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_SMALLEST_GROUP &&
                         conflictChoice != ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_AND_SMALLEST_GROUP,  // TODO: Might be OK. Need to look at it.
		                 "Under makespan, increasing the cost for a single agent might not increase the cost for the solution." +
		                 "Before this strategy is enabled we need to add a consideration of whether the agent whose cost will " +
		                 "increase has the highest cost in the solution first");
        }
        DisableTieBreakingByMinOpsEstimate = disableTieBreakingByMinOpsEstimate;
        _lookaheadMaxExpansions = lookaheadMaxExpansions;
        _mergeCausesRestart = mergeCausesRestart;
        ReplanSameCostWithMdd = replanSameCostWithMdd;
        CacheMdds = cacheMdds;
        _useOldCost = useOldCost;
        _useCAT = useCAT;
    }

    /// <summary>
    /// Implements IIndependenceDetection.Setup for new groups
    /// </summary>
    public void Setup(ProblemInstance problemInstance, Run runner, ConflictAvoidanceTable CAT,
                        int parentGroup1Cost, int parentGroup2Cost, int parentGroup1Size)
    {
        Setup(problemInstance, -1, runner, CAT, null, null, parentGroup1Cost + parentGroup2Cost);  // TODO: Support a makespan cost function
        // I think we don't want to merge the agents in each group. We'd be left with two large meta-agents,
        // and in CBS one conflict isn't a reason to merge agents. The groups are arbitrarily large so we can't
        // even increment their conflict counts toward the merge threshold.
    }

    /// <summary>
    /// Implements IIndependenceDetection.Setup for replanning groups to resolve a conflict
    /// </summary>
    public void Setup(ProblemInstance problemInstance, Run runner, ConflictAvoidanceTable CAT,
                        int targetCost, ISet<TimedMove> illegalMoves)
    {
        // Turn the set of reserved/illegal moves into constraints for every agent. Not the prettiest solution.
        // TODO: Instead, maybe add the agents in the reservation table into the problem instance, and add positive
        //       constraints for them along their entire path. Would require changing IIndependenceDetectionSolver.Setup
        //       to specify the agents the reserved moves belong to.
        var constraints = new HashSet<CbsConstraint>(illegalMoves.Count * problemInstance.agents.Length);
        foreach (var illegalMove in illegalMoves)
        {
            foreach (var agentState in problemInstance.agents)
            {
                constraints.Add(new CbsConstraint(agentState.agent.agentNum, illegalMove));
            }
        }
        Setup(problemInstance, illegalMoves.Max(move => move.Time), runner, CAT, constraints, null, targetCost, targetCost);
    }

    /// <summary>
    /// Implements ICbsSolver.Setup.
    /// </summary>
    /// <param name="problemInstance"></param>
    /// <param name="minSolutionTimeStep"></param>
    /// <param name="runner"></param>
    /// <param name="minSolutionCost"></param>
    /// <param name="maxSolutionCost"></param>
    /// <param name="mdd">Currently ignored. FIXME: Need to convert to array of MDDs to use.</param>
    public virtual void Setup(ProblemInstance problemInstance, int minSolutionTimeStep, Run runner,
        ConflictAvoidanceTable externalCAT, ISet<CbsConstraint> externalConstraints,
        ISet<CbsConstraint> externalPositiveConstraints,
        int minSolutionCost = -1, int maxSolutionCost = int.MaxValue, MDD mdd = null)
    {
        _instance = problemInstance;
        _runner = runner;
        if (OpenList is DynamicLazyOpenList<CbsNode> list)
            list.runner = runner;

        ClearPrivateStatistics();
        SolutionCost = 0;
        _solutionDepth = -1;
        LowLevelGeneratedCap = int.MaxValue;
        TargetF = int.MaxValue;
        MilliCap = int.MaxValue;
        _goalNode = null;
        _solution = null;

        // CBS parameters
        MinSolutionTimeStep = minSolutionTimeStep;
        MinSolutionCost = minSolutionCost;
        _maxSolutionCost = Math.Max(_maxSolutionCost, maxSolutionCost);
        if (ReplanSameCostWithMdd)
            Trace.Assert(MergeThreshold == -1, "Using MDDs to replan same-cost paths is currently only supported for single agents");

        if (CacheMdds)
        {
            MDDCache = new Dictionary<CbsCacheEntry, MDD>[_instance.agents.Length];
            MDDNarrownessValuesCache = new Dictionary<CbsCacheEntry, Dictionary<int, MDD.LevelNarrowness>>[_instance.agents.Length];
            for (int i = 0; i < _instance.agents.Length; i++)
            {
                MDDCache[i] = [];
                MDDNarrownessValuesCache[i] = [];
            }
        }

        ExternalCAT = externalCAT;
        CAT_U CAT = null;
        if (_useCAT)
        {
            CAT = new CAT_U();
            if (externalCAT != null)
                CAT.Join(externalCAT);
        }

        ExternalConstraints = externalConstraints;
        ExternalPositiveConstraints = externalPositiveConstraints;
        if (externalPositiveConstraints == null && DoMalte)
            externalPositiveConstraints = new HashSet<CbsConstraint>();

        SetGlobals();

        CbsNode root = new(_instance.agents.Length, _solver, _singleAgentSolver, this);  // Problem instance and various strategy data is all passed under 'this'.
        // Solve the root node
        bool solved = root.Solve(minSolutionTimeStep);

        if (solved)
        {
            if (root.F <= _maxSolutionCost)
            {
                addToGlobalConflictCount(root);  // TODO: Make MACBS_WholeTreeThreshold use nodes that do this automatically after choosing a conflict
                OpenList.Add(root);
                _highLevelGenerated++;
                _closedList.Add(root, root);
            }
            else
            {
                _surplusNodesAvoided++;
            }
        }
    }

    /// <summary>
    /// Implements ISolver.Setup 
    /// </summary>
    /// <param name="problemInstance"></param>
    /// <param name="runner"></param>
    public virtual void Setup(ProblemInstance problemInstance, Run runner) => Setup(problemInstance, 0, runner, null, null, null);

    public IHeuristicCalculator<CbsNode> GetHeuristic() => _heuristic;

    public ICbsSolver GetSolver() => _solver;

    public Dictionary<int, int> GetExternalConflictCounts()
    {
        throw new NotImplementedException(); // For now. Also need to take care of generalised goal nodes!
    }

    public Dictionary<int, List<int>> GetConflictTimes()
    {
        throw new NotImplementedException(); // For now. Also need to take care of generalised goal nodes!
    }

    public ProblemInstance GetProblemInstance() => _instance;

    public void Clear()
    {
        OpenList.Clear();
        _closedList.Clear();
        _solver.Clear();
        // Statistics are reset on Setup.
    }

    public virtual string GetName() 
    {
        string lowLevelSolvers;
        if (MergeThreshold == -1 || Object.ReferenceEquals(_singleAgentSolver, _solver))
            lowLevelSolvers = $"({_singleAgentSolver})";
        else
            lowLevelSolvers = $"(single:{_singleAgentSolver} multi:{_solver})";
        string variants = "";
        if (_bypassStrategy == BypassStrategy.FIRST_FIT_LOOKAHEAD)
        {
            if (_lookaheadMaxExpansions != int.MaxValue)
            {
                if (_lookaheadMaxExpansions != 1)
                    variants += $" with first fit adoption max expansions: {_lookaheadMaxExpansions}";
                else
                    variants += " + BP";
            }
            else
                variants += " with first fit adoption max expansions: $\\infty$"; // LaTeX infinity symbol
        }
        if (_bypassStrategy == BypassStrategy.BEST_FIT_LOOKAHEAD)
        {
            if (_lookaheadMaxExpansions == int.MaxValue)
                variants += " with infinite lookahead best fit adoption";
            else
                variants += $" with {_lookaheadMaxExpansions} lookahead best fit adoption";
        }
        if (DoMalte)
            variants += " with Malte";

        if (ConflictChoice == ConflictChoice.FIRST)
            variants += " choosing the first conflict in CBS nodes";
        else if (ConflictChoice == ConflictChoice.CARDINAL_MDD)
            variants += " + PC";
        else if (ConflictChoice == ConflictChoice.CARDINAL_LOOKAHEAD)
            variants += " + PC choosing cardinal conflicts using lookahead";
        else if (ConflictChoice == ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_SMALLEST_GROUP)
            variants += " + PC MERGE EARLY MOST CONFLICTING SMALLEST GROUP";
        else if (ConflictChoice == ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_AND_SMALLEST_GROUP)
            variants += " + PC MERGE EARLY MOST CONFLICTING & SMALLEST GROUP";

        if (DisableTieBreakingByMinOpsEstimate)
            variants += " without smart tie breaking";

        if (_mergeCausesRestart == true && MergeThreshold != -1)
            variants += " with merge&restart";

        if (ReplanSameCostWithMdd)
            variants += " with replanning same cost paths with MDDs";

        if (CacheMdds)
            variants += " with caching MDDs";

        if (_useOldCost)
            variants += " with using old path costs";

        if (_useCAT == false)
            variants += " without a CAT";

        if (OpenList.GetType() != typeof(OpenList<CbsNode>))
        {
            variants += $" using {OpenList.GetName()}";
        }

        if (MergeThreshold == -1)
            return $"CBS/{lowLevelSolvers}{variants}";
        return $"MA-CBS-Local-{MergeThreshold}/{lowLevelSolvers}{variants}";
    }

    public override string ToString()
    {
        return GetName();
    }

    public int GetSolutionCost() { return SolutionCost; }

    protected void ClearPrivateStatistics()
    {
        _highLevelExpanded = 0;
        _highLevelGenerated = 0;
        _closedListHits = 0;
        _partialExpansions = 0;
        _bypasses = 0;
        _nodesExpandedWithGoalCost = 0;
        _bypassLookAheadNodesCreated = 0;
        _cardinalLookAheadNodesCreated = 0;
        _conflictsBypassed = 0;
        _cardinalConflictSplits = 0;
        _semiCardinalConflictSplits = 0;
        _nonCardinalConflictSplits = 0;
        MDDsBuilt = 0;
        MDDsAdapted = 0;
        _restarts = 0;
        _pathMaxBoosts = 0;
        _reversePathMaxBoosts = 0;
        _pathMaxPlusBoosts = 0;
        _maxSizeGroup = 1;
        _surplusNodesAvoided = 0;
        MDDCacheHits = 0;
        TimePlanningPaths = 0;
        TimeBuildingMdds = 0;
    }

    public virtual void OutputStatisticsHeader(TextWriter output)
    {
        output.Write(ToString() + " Expanded (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Generated (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Closed List Hits (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Partial Expansions (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Adoptions (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Nodes Expanded With Goal Cost (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Bypass Look Ahead Nodes Created (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Cardinal Look Ahead Nodes Created (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Conflicts Bypassed With Adoption (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Cardinal Conflict Splits (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Semi-Cardinal Conflict Splits (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Non-Cardinal Conflict Splits (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " MDDs Built (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " MDDs Adapted (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Restarts (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Path-Max Boosts (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Reverse Path-Max Boosts (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Path-Max Plus Boosts (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Max Group Size (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Surplus Nodes Avoided (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " MDD Cache Hits (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Time Planning Paths (HL)");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(ToString() + " Time Building MDDs (HL)");
        output.Write(Run.RESULTS_DELIMITER);

        _solver.OutputStatisticsHeader(output);
        if (Object.ReferenceEquals(_singleAgentSolver, _solver) == false)
            _singleAgentSolver.OutputStatisticsHeader(output);

        OpenList.OutputStatisticsHeader(output);
    }

    public virtual void OutputStatistics(TextWriter output)
    {
        Console.WriteLine("Total Expanded Nodes (High-Level): {0}", GetHighLevelExpanded());
        Console.WriteLine("Total Generated Nodes (High-Level): {0}", GetHighLevelGenerated());
        Console.WriteLine("Closed List Hits (High-Level): {0}", _closedListHits);
        Console.WriteLine("Partial Expansions (High-Level): {0}", _partialExpansions);
        Console.WriteLine("Adoptions (High-Level): {0}", _bypasses);
        Console.WriteLine("Nodes expanded with goal cost (High-Level): {0}", _nodesExpandedWithGoalCost);
        Console.WriteLine("Bypass lookahead nodes created (High-Level): {0}", _bypassLookAheadNodesCreated);
        Console.WriteLine("Cardinal lookahead nodes created (High-Level): {0}", _cardinalLookAheadNodesCreated);
        Console.WriteLine("Conflicts Bypassed With Adoption (High-Level): {0}", _conflictsBypassed);
        Console.WriteLine("Cardinal Conflicts Splits (High-Level): {0}", _cardinalConflictSplits);
        Console.WriteLine("Semi-Cardinal Conflicts Splits (High-Level): {0}", _semiCardinalConflictSplits);
        Console.WriteLine("Non-Cardinal Conflicts Splits (High-Level): {0}", _nonCardinalConflictSplits);
        Console.WriteLine("MDDs Built (High-Level): {0}", MDDsBuilt);
        Console.WriteLine("MDDs adapted (High-Level): {0}", MDDsAdapted);
        Console.WriteLine("Restarts (High-Level): {0}", _restarts);
        Console.WriteLine("Path-Max Boosts (High-Level): {0}", _pathMaxBoosts);
        Console.WriteLine("Reverse Path-Max Boosts (High-Level): {0}", _reversePathMaxBoosts);
        Console.WriteLine("Path-Max Plus Boosts (High-Level): {0}", _pathMaxPlusBoosts);
        Console.WriteLine("Max Group Size (High-Level): {0}", _maxSizeGroup);
        Console.WriteLine("Surplus Nodes Avoided (High-Level): {0}", _surplusNodesAvoided);
        Console.WriteLine("MDD cache hits (High-Level): {0}", MDDCacheHits);
        Console.WriteLine("Time planning paths (High-Level): {0}", TimePlanningPaths);
        Console.WriteLine("Time building mdds (High-Level): {0}", TimeBuildingMdds);

        output.Write(_highLevelExpanded + Run.RESULTS_DELIMITER);
        output.Write(_highLevelGenerated + Run.RESULTS_DELIMITER);
        output.Write(_closedListHits + Run.RESULTS_DELIMITER);
        output.Write(_partialExpansions + Run.RESULTS_DELIMITER);
        output.Write(_bypasses + Run.RESULTS_DELIMITER);
        output.Write(_nodesExpandedWithGoalCost + Run.RESULTS_DELIMITER);
        output.Write(_bypassLookAheadNodesCreated + Run.RESULTS_DELIMITER);
        output.Write(_cardinalLookAheadNodesCreated + Run.RESULTS_DELIMITER);
        output.Write(_conflictsBypassed + Run.RESULTS_DELIMITER);
        output.Write(_cardinalConflictSplits + Run.RESULTS_DELIMITER);
        output.Write(_semiCardinalConflictSplits + Run.RESULTS_DELIMITER);
        output.Write(_nonCardinalConflictSplits + Run.RESULTS_DELIMITER);
        output.Write(MDDsBuilt + Run.RESULTS_DELIMITER);
        output.Write(MDDsAdapted + Run.RESULTS_DELIMITER);
        output.Write(_restarts + Run.RESULTS_DELIMITER);
        output.Write(_pathMaxBoosts + Run.RESULTS_DELIMITER);
        output.Write(_reversePathMaxBoosts + Run.RESULTS_DELIMITER);
        output.Write(_pathMaxPlusBoosts + Run.RESULTS_DELIMITER);
        output.Write(_maxSizeGroup + Run.RESULTS_DELIMITER);
        output.Write(_surplusNodesAvoided + Run.RESULTS_DELIMITER);
        output.Write(MDDCacheHits + Run.RESULTS_DELIMITER);
        output.Write(TimePlanningPaths + Run.RESULTS_DELIMITER);
        output.Write(TimeBuildingMdds + Run.RESULTS_DELIMITER);

        _solver.OutputAccumulatedStatistics(output);
        if (Object.ReferenceEquals(_singleAgentSolver, _solver) == false)
            _singleAgentSolver.OutputAccumulatedStatistics(output);

        OpenList.OutputStatistics(output);
    }

    public virtual int NumStatsColumns
    {
        get
        {
            int numSolverStats = _solver.NumStatsColumns;
            if (Object.ReferenceEquals(_singleAgentSolver, _solver) == false)
                numSolverStats += _singleAgentSolver.NumStatsColumns;
            return 23 + numSolverStats + OpenList.NumStatsColumns;
        }
    }

    public virtual void ClearStatistics()
    {
        _solver.ClearAccumulatedStatistics(); // Is this correct? Or is it better not to do it?
        if (Object.ReferenceEquals(_singleAgentSolver, _solver) == false)
            _singleAgentSolver.ClearAccumulatedStatistics();
        ClearPrivateStatistics();
        OpenList.ClearStatistics();
    }

    public virtual void ClearAccumulatedStatistics()
    {
        _accHLExpanded = 0;
        _accHLGenerated = 0;
        _accClosedListHits = 0;
        _accPartialExpansions = 0;
        _accBypasses = 0;
        _accNodesExpandedWithGoalCost = 0;
        _accBypassLookAheadNodesCreated = 0;
        _accCardinalLookAheadNodesCreated = 0;
        _accConflictsBypassed = 0;
        _accCardinalConflictSplits = 0;
        _accSemiCardinalConflictSplits = 0;
        _accNonCardinalConflictSplits = 0;
        _accMddsBuilt = 0;
        _accMddsAdapted = 0;
        _accRestarts = 0;
        _accPathMaxBoosts = 0;
        _accReversePathMaxBoosts = 0;
        _accPathMaxPlusBoosts = 0;
        _accMaxSizeGroup = 1;
        _accSurplusNodesAvoided = 0;
        _accMddCacheHits = 0;
        _accTimePlanningPaths = 0;
        _accTimeBuildingMdds = 0;

        _solver.ClearAccumulatedStatistics();
        if (Object.ReferenceEquals(_singleAgentSolver, _solver) == false)
            _singleAgentSolver.ClearAccumulatedStatistics();

        OpenList.ClearAccumulatedStatistics();
    }

    public virtual void AccumulateStatistics()
    {
        _accHLExpanded += _highLevelExpanded;
        _accHLGenerated += _highLevelGenerated;
        _accClosedListHits += _closedListHits;
        _accPartialExpansions += _partialExpansions;
        _accBypasses += _bypasses;
        _accNodesExpandedWithGoalCost += _nodesExpandedWithGoalCost;
        _accBypassLookAheadNodesCreated += _bypassLookAheadNodesCreated;
        _accCardinalLookAheadNodesCreated += _cardinalLookAheadNodesCreated;
        _accConflictsBypassed += _conflictsBypassed;
        _accCardinalConflictSplits += _cardinalConflictSplits;
        _accSemiCardinalConflictSplits += _semiCardinalConflictSplits;
        _accNonCardinalConflictSplits += _nonCardinalConflictSplits;
        _accMddsBuilt += MDDsBuilt;
        _accMddsAdapted += MDDsAdapted;
        _accRestarts += _restarts;
        _accPathMaxBoosts += _pathMaxBoosts;
        _accReversePathMaxBoosts += _reversePathMaxBoosts;
        _accPathMaxPlusBoosts += _pathMaxPlusBoosts;
        _accMaxSizeGroup = Math.Max(_accMaxSizeGroup, _maxSizeGroup);
        _accSurplusNodesAvoided += _surplusNodesAvoided;
        _accMddCacheHits += MDDCacheHits;
        _accTimePlanningPaths += TimePlanningPaths;
        _accTimeBuildingMdds += TimeBuildingMdds;

        // solver statistics are accumulated every time it's used.

        OpenList.AccumulateStatistics();
    }

    public virtual void OutputAccumulatedStatistics(TextWriter output)
    {
        Console.WriteLine("{0} Accumulated Expanded Nodes (High-Level): {1}", this, _accHLExpanded);
        Console.WriteLine("{0} Accumulated Generated Nodes (High-Level): {1}", this, _accHLGenerated);
        Console.WriteLine("{0} Accumulated Closed List Hits (High-Level): {1}", this, _accClosedListHits);
        Console.WriteLine("{0} Accumulated Partial Expansions (High-Level): {1}", this, _accPartialExpansions);
        Console.WriteLine("{0} Accumulated Adoptions (High-Level): {1}", this, _accBypasses);
        Console.WriteLine("{0} Accumulated Nodes Expanded With Goal Cost (High-Level): {1}", this, _accNodesExpandedWithGoalCost);
        Console.WriteLine("{0} Accumulated Look Ahead Nodes Created (High-Level): {1}", this, _accNodesExpandedWithGoalCost);
        Console.WriteLine("{0} Accumulated Conflicts Bypassed With Adoption (High-Level): {1}", this, _accConflictsBypassed);
        Console.WriteLine("{0} Accumulated Cardinal Conflicts Splits (High-Level): {1}", this, _accCardinalConflictSplits);
        Console.WriteLine("{0} Accumulated Semi-Cardinal Conflicts Splits (High-Level): {1}", this, _accSemiCardinalConflictSplits);
        Console.WriteLine("{0} Accumulated Non-Cardinal Conflicts Splits (High-Level): {1}", this, _accNonCardinalConflictSplits);
        Console.WriteLine("{0} Accumulated MDDs Built (High-Level): {1}", this, _accMddsBuilt);
        Console.WriteLine("{0} Accumulated MDDs adapted (High-Level): {1}", this, _accMddsAdapted);
        Console.WriteLine("{0} Accumulated Restarts (High-Level): {1}", this, _accRestarts);
        Console.WriteLine("{0} Accumulated Path-Max Boosts (High-Level): {1}", this, _accPathMaxBoosts);
        Console.WriteLine("{0} Accumulated Reverse Path-Max Boosts (High-Level): {1}", this, _accReversePathMaxBoosts);
        Console.WriteLine("{0} Accumulated Path-Max Plus Boosts (High-Level): {1}", this, _accPathMaxPlusBoosts);
        Console.WriteLine("{0} Max Group Size (High-Level): {1}", this, _accMaxSizeGroup);
        Console.WriteLine("{0} Accumulated Surplus Nodes Avoided (High-Level): {1}", this, _accSurplusNodesAvoided);
        Console.WriteLine("{0} Accumulated MDD cache hits (High-Level): {1}", this, _accMddCacheHits);
        Console.WriteLine("{0} Accumulated time planning paths (High-Level): {1}", this, _accTimePlanningPaths);
        Console.WriteLine("{0} Accumulated time building MDDs (High-Level): {1}", this, _accTimeBuildingMdds);

        output.Write(_accHLExpanded + Run.RESULTS_DELIMITER);
        output.Write(_accHLGenerated + Run.RESULTS_DELIMITER);
        output.Write(_accClosedListHits + Run.RESULTS_DELIMITER);
        output.Write(_accPartialExpansions + Run.RESULTS_DELIMITER);
        output.Write(_accBypasses + Run.RESULTS_DELIMITER);
        output.Write(_accNodesExpandedWithGoalCost + Run.RESULTS_DELIMITER);
        output.Write(_accBypassLookAheadNodesCreated + Run.RESULTS_DELIMITER);
        output.Write(_accCardinalLookAheadNodesCreated + Run.RESULTS_DELIMITER);
        output.Write(_accConflictsBypassed + Run.RESULTS_DELIMITER);
        output.Write(_accCardinalConflictSplits + Run.RESULTS_DELIMITER);
        output.Write(_accSemiCardinalConflictSplits + Run.RESULTS_DELIMITER);
        output.Write(_accNonCardinalConflictSplits + Run.RESULTS_DELIMITER);
        output.Write(_accMddsBuilt + Run.RESULTS_DELIMITER);
        output.Write(_accMddsAdapted + Run.RESULTS_DELIMITER);
        output.Write(_accRestarts + Run.RESULTS_DELIMITER);
        output.Write(_accPathMaxBoosts + Run.RESULTS_DELIMITER);
        output.Write(_accReversePathMaxBoosts + Run.RESULTS_DELIMITER);
        output.Write(_accPathMaxPlusBoosts + Run.RESULTS_DELIMITER);
        output.Write(_accMaxSizeGroup + Run.RESULTS_DELIMITER);
        output.Write(_accSurplusNodesAvoided + Run.RESULTS_DELIMITER);
        output.Write(_accMddCacheHits + Run.RESULTS_DELIMITER);
        output.Write(_accTimePlanningPaths + Run.RESULTS_DELIMITER);
        output.Write(_accTimeBuildingMdds + Run.RESULTS_DELIMITER);

        _solver.OutputAccumulatedStatistics(output);
        if (Object.ReferenceEquals(_singleAgentSolver, _solver) == false)
            _singleAgentSolver.OutputAccumulatedStatistics(output);

        OpenList.OutputAccumulatedStatistics(output);
    }

    private bool _debug = false;
    private bool _equivalenceWasOn;

    protected void SetGlobals()
    {
        _equivalenceWasOn = AgentState.EquivalenceOverDifferentTimes;
        AgentState.EquivalenceOverDifferentTimes = false;
    }

    protected void CleanGlobals()
    {
        AgentState.EquivalenceOverDifferentTimes = _equivalenceWasOn;
    }

    public bool Solve()
    {
        //SetGlobals(); // Again, because we might be resuming a search that was stopped.

        int initialEstimate = 0;
        if (OpenList.Count > 0)
            initialEstimate = OpenList.Peek().F;

        int maxExpandedNodeF = -1;
        int currentCost = -1;

        while (OpenList.Count > 0)
        {
            // Check if max time has been exceeded
            if (_runner.ElapsedMilliseconds() > Constants.MAX_TIME)
            {
                SolutionCost = (int) Constants.SpecialCosts.TIMEOUT_COST;
                Console.WriteLine("Out of time");
                _solutionDepth = maxExpandedNodeF - initialEstimate; // A minimum estimate.
                                                                         // Can't use top of OPEN's f-value instead of openList.Peek().f because we may have
                                                                         // timed out while expanding and before generating nodes that would have led to the
                                                                         // optimal solution, and the remaining nodes in OPEN are junk
                Clear(); // Total search time exceeded - we're not going to resume this search.
                CleanGlobals();
                return false;
            }

            Debug.WriteLine("Getting the next node from OPEN");
            CbsNode currentNode = OpenList.Remove();

            if (currentNode.F > _maxSolutionCost)  // A late heuristic application may have increased the node's cost
            {
                continue;
                // Don't expand the node.
                // This will exhaust the open list, assuming Fs of nodes chosen for expansions
                // are monotonically increasing.
                // TODO: Pass maxSolutionCost to the DynamicLazyOpenList so it can inform the 
                //       heuristic not to try to improve its estimate to make the node's f more
                //       than 1 + maxSolutionCost.
                //       OTOH, DynamicLazyOpenList only tries to increase the cost by 1 over the
                //       current minimum, and it would never reach maxSolutionCost + 1 to try to
                //       go above it.
            }


            currentNode.ChooseConflict();  // Does nothing if this node has already been partially expanded before

            // Notice that even though we may discover cardinal conflicts in hindsight later,
            // there would be no point in pushing their node back at that point,
            // as we would've already made the split by then.

            Debug.WriteLine("Expanding node: (or returning its solution, if it's a goal node)");
            currentNode.DebugPrint();

            // Update nodesExpandedWithGoalCost statistic
            if (currentNode.G > currentCost) // Needs to be here because the goal may have a cost unseen before
            {
                currentCost = currentNode.G;
                _nodesExpandedWithGoalCost = 0;
            }
            else if (currentNode.G == currentCost) // check needed because macbs node cost isn't exactly monotonous
            {
                _nodesExpandedWithGoalCost++;
            }

            // Check if node is the goal
            if (currentNode.GoalTest())
            {
                Trace.Assert(currentNode.G >= maxExpandedNodeF,
                                $"CBS goal node found with lower cost than the max cost node ever expanded ({currentNode.G} < {maxExpandedNodeF})");
                // This is subtle, but MA-CBS may expand nodes in a non non-decreasing order:
                // If a non-optimal constraint is expanded upon and we decide to merge the agents,
                // the resulting node can have a lower cost than before, since we ignore the non-optimal constraint
                // because the conflict it addresses is between merged nodes.
                // The resulting lower-cost node will have other constraints, that will raise the cost of its children back to at least its original cost,
                // since the node with the non-optimal constraint was only expanded because its competitors that had an optimal
                // constraint to deal with the same conflict apparently found the other conflict that I promise will be found,
                // and so their cost was not smaller than this sub-optimal node.
                // To make MA-CBS costs non-decreasing, we can choose not to ignore constraints that deal with conflicts between merged nodes.
                // That way, the sub-optimal node will find a sub-optimal merged solution and get a high cost that will push it deep into the open list.
                // But the cost would be to create a possibly sub-optimal merged solution where an optimal solution could be found instead, and faster,
                // since constraints make the low-level heuristic perform worse.
                // For an example for this subtle case happening, see problem instance 63 of the random grid with 4 agents,
                // 55 grid cells and 9 obstacles.

                Debug.WriteLine("-----------------");
                SolutionCost = currentNode.G;
                _solutionDepth = SolutionCost - initialEstimate;
                _goalNode = currentNode; // Saves the single agent plans and costs
                // The joint plan is calculated on demand.
                Clear(); // Goal found - we're not going to resume this search
                CleanGlobals();
                return true;
            }

            if (maxExpandedNodeF < currentNode.F)
            {
                maxExpandedNodeF = currentNode.F;
                Debug.WriteLine("New max F: {0}", maxExpandedNodeF);
            }

            // Check conditions that stop the search before a goal is found
            if (currentNode.F >= TargetF || // Node is good enough
                _singleAgentSolver.GetAccumulatedGenerated() + _solver.GetAccumulatedGenerated() > 
                    LowLevelGeneratedCap || // Stop because this is taking too long.
                                                    // We're looking at _generated_ low level nodes since that's an indication to the amount of work done,
                                                    // while expanded nodes is an indication of the amount of good work done.
                (MilliCap != int.MaxValue && // (This check is much cheaper than the method call)
                    _runner.ElapsedMilliseconds() > MilliCap)) // Search is taking too long.
            {
                Debug.WriteLine("-----------------");
                SolutionCost = maxExpandedNodeF; // This is the min possible cost so far.
                OpenList.Add(currentNode); // To be able to continue the search later
                CleanGlobals();
                return false;
            }

            // Expand
            bool wasUnexpandedNode = (currentNode.AgentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED &&
                                        currentNode.AgentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED);
            Expand(currentNode);
            if (wasUnexpandedNode)
                _highLevelExpanded++;
            // Consider moving the following into Expand()
            if (currentNode.AgentAExpansion == CbsNode.ExpansionState.EXPANDED &&
                currentNode.AgentBExpansion == CbsNode.ExpansionState.EXPANDED) // Fully expanded
                currentNode.Clear();
        }

        // Check if max time has been exceeded
        if (_runner.ElapsedMilliseconds() > Constants.MAX_TIME)
        {
            SolutionCost = (int)Constants.SpecialCosts.TIMEOUT_COST;
            Console.WriteLine("Out of time");
            _solutionDepth = maxExpandedNodeF - initialEstimate; // A minimum estimate
        }
        else
            SolutionCost = (int) Constants.SpecialCosts.NO_SOLUTION_COST;
        Clear(); // we're not going to resume this search - it either timed out or the problem is unsolvable
        CleanGlobals();
        return false;
    }

    protected virtual bool ShouldMerge(CbsNode node)
    {
        return node.ShouldMerge(MergeThreshold, node.Conflict.agentAIndex, node.Conflict.agentBIndex);  //TODO: Add an option to use the old merge criterion
    }

    public virtual void Reset()
    {
        _restarts++;
        OpenList.Clear();
        _closedList.Clear();
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="node"></param>
    /// <param name="adopt"></param>
    /// <param name="children"></param>
    /// <param name="reinsertParent">If it was only partially expanded</param>
    /// <param name="adoptBy">If not given, adoption is done by expanded node</param>
    /// <returns>true if adopted - need to rerun this method, ignoring the returned children from this call, bacause adoption was performed</returns>
    protected (bool adopted, IList<CbsNode> children, bool reinsertParent) ExpandImpl(CbsNode node, bool adopt, CbsNode adoptBy = null)
    {
        CbsConflict conflict = node.GetConflict();
        List<CbsNode> children = [];
        CbsNode child;
        int closedListHitChildCost;
        if (adoptBy == null)
            adoptBy = node;
        int adoptByH = adoptBy.H;

        bool leftSameCost = false; // To quiet the compiler
        bool rightSameCost = false;

        if (MergeThreshold != -1 && ShouldMerge(node))
        {
            if (_mergeCausesRestart == false)
            {
                (child, closedListHitChildCost) = MergeExpand(node);
                if (child == null)
                    return (adopted: false, children, reinsertParent: false); // A timeout occured,
                                                                                // or the child was already in the closed list,
                                                                                // or there were just too many constraints
                                                                                // (happens with ID, which adds whole paths as constraints)
            }
            else
            {
                // TODO: What if planning a path for the merged agents finds a path with the same
                // cost as the sum of their current paths and no other conflicts exist? Should just
                // adopt this solution and get a goal node.
                // TODO: Save the cost of the group in a table, and use it as a heuristic in the future!
                child = new CbsNode(_instance.agents.Length, _solver,
                                    _singleAgentSolver, this, node.AgentsGroupAssignment);  // This will be the new root node
                child.MergeGroups(node.AgentsGroupAssignment[conflict.agentAIndex], node.AgentsGroupAssignment[conflict.agentBIndex],
                                  fixCounts: false  // This is a new root node, it doesn't have conflict counts yet
                                  );
                _maxSizeGroup = Math.Max(_maxSizeGroup, child.GetGroupSize(conflict.agentAIndex));
                bool solved = child.Solve(MinSolutionTimeStep);
                child.H = node.F - child.G;
                
                //if (debug)
                    Debug.WriteLine($"Restarting the search with agents {node.AgentsGroupAssignment[conflict.agentAIndex]} and" +
                                    $" {node.AgentsGroupAssignment[conflict.agentBIndex]} merged.");
                Reset();

                if (solved == false)  // Likely due to a time-out
                    return (adopted: false, children, reinsertParent: false);
            }
            // No need to try to adopt the child - there's only one so we're not branching.
            children.Add(child);
            return (adopted: false, children, reinsertParent: false);
        }

        bool reinsertParent = false;

        // Generate left child:
        (child, closedListHitChildCost) = ConstraintExpand(node, doLeftChild: true);
        if (child != null)
        {
            if (child == node) // Expansion deferred
                reinsertParent = true;
            else // New child
            {
                // First fit adoption: Greedily adopt the first child that's better than the parent
                if (adopt &&
                    AdoptConditionally(adoptBy, child, adoptByH))
                    return (adopted: true, children, reinsertParent);
                children.Add(child);
                leftSameCost = child.G == node.G;
                if (leftSameCost)
                {
                    Trace.Assert(node.Conflict.willCostIncreaseForAgentA != CbsConflict.WillCostIncrease.YES);
                }
                else
                {
                    Trace.Assert(node.Conflict.willCostIncreaseForAgentA != CbsConflict.WillCostIncrease.NO);
                }
            }
        }
        else  // A timeout occured, or the child was already in the closed list.
        {
            if (closedListHitChildCost != -1)
                leftSameCost = closedListHitChildCost == node.G;
        }

        if (_runner.ElapsedMilliseconds() > Constants.MAX_TIME)
            return (adopted: false, children, reinsertParent);

        // Generate right child:
        (child, closedListHitChildCost) = ConstraintExpand(node, doLeftChild: false);
        if (child != null)
        {
            if (child == node) // Expansion deferred
                reinsertParent = true;
            else // New child
            {
                // First fit adoption: Greedily adopt the first child that's better than the parent
                if (adopt &&
                    AdoptConditionally(adoptBy, child, adoptByH))
                    return (adopted: true, children, reinsertParent);
                children.Add(child);
                rightSameCost = child.G == node.G;
                if (rightSameCost)
                {
                    Trace.Assert(node.Conflict.willCostIncreaseForAgentB != CbsConflict.WillCostIncrease.YES);
                }
                else
                {
                    Trace.Assert(node.Conflict.willCostIncreaseForAgentB != CbsConflict.WillCostIncrease.NO);
                }
            }
        }
        else  // A timeout occured, or the child was already in the closed list.
        {
            if (closedListHitChildCost != -1)
                rightSameCost = closedListHitChildCost == node.G;
        }

        if (node.AgentAExpansion == CbsNode.ExpansionState.DEFERRED || node.AgentBExpansion == CbsNode.ExpansionState.DEFERRED)
            _semiCardinalConflictSplits++; // Count only on the first expansion of the node. On the second expansion no count will be incremented.
        else
        {
            if (leftSameCost && rightSameCost)
                _nonCardinalConflictSplits++;
            else if (leftSameCost || rightSameCost)
                _semiCardinalConflictSplits++;
            else
                _cardinalConflictSplits++;
        }

        return (adopted: false, children, reinsertParent);
    }
        
    private void ExpandIgnoringCardinalsButSupportingBP2(CbsNode node)
    {
        int parentCost = node.G;
        int parentH = node.H;
        IList<CbsNode> children = new List<CbsNode>(2);
        bool reinsertParent = false;
        bool adopted = false;
             
        int origCardinalConflictSplits = _cardinalConflictSplits;
        int origSemiCardinalConflictSplits = _semiCardinalConflictSplits;
        int origNonCardinalConflictSplits = _nonCardinalConflictSplits;

        if (_bypassStrategy == BypassStrategy.NONE)
        {
            (adopted, children, reinsertParent) = ExpandImpl(node, adopt: false, adoptBy: null);
        }
        else if (_bypassStrategy == BypassStrategy.FIRST_FIT_LOOKAHEAD || node.ParentAlreadyLookedAheadOf) // Do bypass but don't look ahead
        {
            bool adoptionPerformedBefore = false;
            while (true) // Until a node with a higher cost is found or a goal is found
            {
                OpenList<CbsNode> lookAheadOpenList = new(this);
                HashSet<CbsNode> lookAheadSameCostNodes = [];
                HashSet<CbsNode> lookAheadLargerCostNodes = [];
                HashSet<CbsNode> lookAheadSameCostNodesToReinsertWithHigherCost = [];
                lookAheadOpenList.Add(node);

                if (lookAheadOpenList.Count != 0)
                    Debug.WriteLine("Starting lookahead:");
                IList<CbsNode> lookAheadChildren;
                bool lookAheadReinsertParent;
                int expansions = 0;
                while (lookAheadOpenList.Count != 0)
                {
                    if (expansions + 1 > _lookaheadMaxExpansions) // + 1 because we're checking before the coming expansion. lookaheadMaxExpansions = 1 is the minimum working value.
                    {
                        Debug.WriteLine("Lookahead count exceeded. Stopping lookahead.");
                        break;
                    }

                    if (_runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                        return;

                    CbsNode lookAheadNode = lookAheadOpenList.Remove();
                    lookAheadNode.ChooseConflict();

                    Debug.WriteLine($"Looking ahead from node hash: {lookAheadNode.GetHashCode()}.");

                    (adopted, lookAheadChildren, lookAheadReinsertParent) = ExpandImpl(lookAheadNode, adopt: true, adoptBy: node);
                    expansions++;

                    if (adopted == true)
                    {
                        _bypassLookAheadNodesCreated += lookAheadChildren.Count + 1; // Either 1, if the first child was adopted, or 2 otherwise. +1 for the node itself before it adopted another solution.
                        _cardinalConflictSplits = origCardinalConflictSplits;  // No split was done
                        _semiCardinalConflictSplits = origSemiCardinalConflictSplits;
                        _nonCardinalConflictSplits = origNonCardinalConflictSplits;
                        break; // Insert children into open list
                    }

                    if (lookAheadReinsertParent)
                        lookAheadSameCostNodesToReinsertWithHigherCost.Add(lookAheadNode);

                    foreach (var child in lookAheadChildren)
                    {
                        _bypassLookAheadNodesCreated++;
                        _closedList.Add(child, child); // Temporarily! Just so look ahead expansion can use this data
                        if (child.G == node.G)
                        {
                            lookAheadOpenList.Add(child);
                            lookAheadSameCostNodes.Add(child);
                        }
                        else
                            lookAheadLargerCostNodes.Add(child); // No need to check if it's already there since the closed list is properly maintained
                    }
                }

                if (adopted == false && lookAheadOpenList.Count == 0)
                    Debug.WriteLine("Lookahead exhausted all same cost nodes.");

                if (adopted && adoptionPerformedBefore == false)
                {
                    adoptionPerformedBefore = true;
                    _bypasses++; // Only count one adoption for the entire look ahead subtree branch traversed.
                }

                if (node.GoalTest()) // Lookahead found an admissible solution! (The original node to expand wasn't a goal)
                {
                    Debug.WriteLine("Goal found with same cost - stopping lookahead.");
                    OpenList.Add(node);
                    return;
                }

                if (adopted == false) // Insert children into open list
                {
                    if (node.H < 1 && lookAheadOpenList.Count == 0) // Looked ahead exhaustively
                        node.H = 1; // We know the goal isn't under it with the same cost

                    // We already expanded this whole subtree of same cost nodes (at least partially). Insert the frontier to the open list.
                    reinsertParent = false; // It may be reinserted as an element of the lookAheadSameCostNodesToReinsertWithHigherCost
                    children = lookAheadLargerCostNodes.ToList(); // Larger cost nodes aren't expanded so they're always on the frontier
                    int unexpandedSameCostNodes = lookAheadOpenList.Count;
                    while (lookAheadOpenList.Count != 0)
                    {
                        CbsNode child = lookAheadOpenList.Remove();
                        children.Add(child); // Unexapnded so it's also on the frontier
                        _closedList.Remove(child); // Just so it'll be inserted into the open list at the end of the method
                    }
                    foreach (var reInsertNode in lookAheadSameCostNodesToReinsertWithHigherCost)
                    {
                        if (reInsertNode == node)
                        {
                            reinsertParent = true;
                            continue;
                        }
                        children.Add(reInsertNode);
                        _closedList.Remove(reInsertNode); // Just so it'll be inserted into the open list at the end of the method
                    }
                    foreach (CbsNode lookAheadNode in lookAheadLargerCostNodes)
                        _closedList.Remove(lookAheadNode); // Just so they'll be inserted into the open list at the end of the method
                    _bypassLookAheadNodesCreated -= lookAheadSameCostNodes.Count + lookAheadLargerCostNodes.Count; // None of these nodes weren't wasted effort, they were properly generated
                    _highLevelGenerated += lookAheadSameCostNodes.Count
                                                - lookAheadSameCostNodesToReinsertWithHigherCost.Count
                                                - unexpandedSameCostNodes; // You could say they were generated and not looked ahead at, and they won't be counted later.
                    _highLevelExpanded += lookAheadSameCostNodes.Count - unexpandedSameCostNodes;

                    break;
                }
                else // Adopted. Do a new round of expansions.
                {
                    foreach (CbsNode lookAheadNode in lookAheadLargerCostNodes)
                        _closedList.Remove(lookAheadNode); // Just so they'll be inserted into the open list at the end of the method
                    foreach (CbsNode lookAheadNode in lookAheadSameCostNodes)
                        _closedList.Remove(lookAheadNode); // Just so they'll be inserted into the open list at the end of the method
                                                                // The only difference in closed list cleanup after adoption is that expanded same cost nodes are also removed from the closed list.
                }
            }
        }
        else // bypassStrategy == BypassStrategy.BEST_FIT_LOOKAHEAD and this set of costs not already done
        {
            // FIXME: lookaheadMaxExpansions isn't respected correctly here. We actually limit the number of same-cost nodes generated, which is similar but not the same.
            OpenList<CbsNode> lookAheadOpenList = new(this);
            HashSet<CbsNode> lookAheadSameCostNodes = [];
            HashSet<CbsNode> lookAheadLargerCostNodes = [];
            HashSet<CbsNode> lookAheadSameCostNodesToReinsertWithHigherCost = [];
            lookAheadOpenList.Add(node);
            node.ParentAlreadyLookedAheadOf = true;

            if (lookAheadOpenList.Count != 0)
                Debug.WriteLine("Starting lookahead:");
            IList<CbsNode> lookAheadChildren;
            bool lookAheadReinsertParent;
            while (lookAheadOpenList.Count != 0)
            {
                if (lookAheadSameCostNodes.Count + 1 >= _lookaheadMaxExpansions) // + 1 for the root
                {
                    Debug.WriteLine("Lookahead count exceeded. Stopping lookahead.");
                    break;
                }

                CbsNode lookAheadNode = lookAheadOpenList.Remove();
                lookAheadNode.ChooseConflict();

                if (_runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                    return;

                Debug.WriteLine($"Looking ahead from node hash: {lookAheadNode.GetHashCode()}.");

                (adopted, lookAheadChildren, lookAheadReinsertParent) = ExpandImpl(lookAheadNode, adopt: false);

                if (lookAheadReinsertParent)
                    lookAheadSameCostNodesToReinsertWithHigherCost.Add(lookAheadNode);

                foreach (var lookaheadChild in lookAheadChildren)
                {
                    _bypassLookAheadNodesCreated++;
                    _closedList.Add(lookaheadChild, lookaheadChild); // Temporarily! Just so look ahead expansion can use this data
                    if (lookaheadChild.G == node.G)
                    {
                        if (lookaheadChild.GoalTest()) // Lookahead found an admissible solution!
                        {
                            Debug.WriteLine("Goal found with same cost - stopping lookahead.");
                            OpenList.Add(lookaheadChild); // Technically should have just breaked and let node adopt child. This is just a short-cut.
                            _bypasses++; // Just a technicality needed to make the adoption count not lower than if we didn't use immediate adoption. You could say we adopt the goal's solution.
                            return;
                        }

                        if (lookAheadSameCostNodes.Contains(lookaheadChild) == false)
                        {
                            lookAheadOpenList.Add(lookaheadChild);
                            lookAheadSameCostNodes.Add(lookaheadChild);
                        }
                    }
                    else
                        lookAheadLargerCostNodes.Add(lookaheadChild); // No need to check if it's already there :)
                }
            }

            if (node.H < 1 && lookAheadOpenList.Count == 0) // Looked ahead exhaustively
                node.H = 1; // We know the goal isn't under it with the same cost

            bool bypassPerformed = false;
            // Find the best look ahead node and check if it's worth adopting
            if (lookAheadSameCostNodes.Count != 0)
            {
                IList<CbsNode> lookAheadSameCostNodesSerialzed = [.. lookAheadSameCostNodes];
                CbsNode adoptionCandidate = lookAheadSameCostNodesSerialzed[0];
                for (int i = 1; i < lookAheadSameCostNodesSerialzed.Count; i++)
                {
                    if (lookAheadSameCostNodesSerialzed[i].TieBreak(adoptionCandidate) == -1)
                        adoptionCandidate = lookAheadSameCostNodesSerialzed[i];
                }
                if (AdoptConditionally(node, adoptionCandidate, parentH))
                {
                    bypassPerformed = true;
                    _bypasses++;

                    _cardinalConflictSplits = origCardinalConflictSplits;
                    _semiCardinalConflictSplits = origSemiCardinalConflictSplits;
                    _nonCardinalConflictSplits = origNonCardinalConflictSplits;

                    // Remove cancelled look-ahead nodes from closed list
                    foreach (CbsNode lookAheadNode in lookAheadSameCostNodes)
                        _closedList.Remove(lookAheadNode);
                    foreach (CbsNode lookAheadNode in lookAheadLargerCostNodes)
                        _closedList.Remove(lookAheadNode);

                    if (OpenList.Count != 0 && OpenList.Peek().F < node.F)
                    // This is a new unexpanded node, and we just raised its h, so we can push it back
                    // FIXME: More duplication of the push back logic
                    {
                        reinsertParent = true;
                        children = []; // Children will be generated when this node comes out of the open list

                        if (_debug)
                            Debug.WriteLine("Reinserting node into the open list with h=1, since the goal wasn't found with the same cost under it.");
                    }
                    else
                    {
                        // We updated the node, need to re-expand it. Surprisingly, re-expansion can produce even better nodes for adoption, so we need to allow immediate expansion too.
                        // TODO: Just re-run the infinite lookahead? That would be the immediate adoption variant above
                        Expand(node);
                        return;
                    }
                }
            }

            if (bypassPerformed == false)
            {
                // Adoption not performed, and we already expanded this whole subtree of same cost nodes (at least partially)
                reinsertParent = false; // It may reinserted as an element of the children list
                children = [.. lookAheadLargerCostNodes];
                int unexpandedSameCostNodes = lookAheadOpenList.Count;
                while (lookAheadOpenList.Count != 0)
                {
                    CbsNode child = lookAheadOpenList.Remove();
                    children.Add(child);
                    _closedList.Remove(child); // Just so it'll be inserted into the open list at the end of the method
                }
                foreach (var reInsertNode in lookAheadSameCostNodesToReinsertWithHigherCost)
                {
                    if (reInsertNode == node)
                    {
                        reinsertParent = true;
                        continue;
                    }
                    children.Add(reInsertNode);
                    _closedList.Remove(reInsertNode); // Just so it'll be inserted into the open list at the end of the method
                    if (reInsertNode == node) // Re-inserting the original node that was to be expanded, with a higher h
                        _highLevelGenerated--; // It'll be counted as a new generated child later, and we don't need to count it twice
                }
                foreach (CbsNode lookAheadNode in lookAheadLargerCostNodes)
                    _closedList.Remove(lookAheadNode); // Just so they'll be inserted into the open list at the end of the method
                _bypassLookAheadNodesCreated -= lookAheadSameCostNodes.Count + lookAheadLargerCostNodes.Count; // These nodes weren't wasted effort
                _highLevelGenerated += lookAheadSameCostNodes.Count - lookAheadSameCostNodesToReinsertWithHigherCost.Count - unexpandedSameCostNodes; // You could say they were generated and not looked ahead at, and they won't be counted later.
                _highLevelExpanded += lookAheadSameCostNodes.Count - unexpandedSameCostNodes;
            }
        }

        // Both children considered. None adopted. Add them to the open list, and re-insert the partially expanded parent too if necessary.
        if (reinsertParent)
            OpenList.Add(node); // Re-insert node into open list with higher cost, don't re-increment global conflict counts

        foreach (var child in children)
        {
            _closedList.Add(child, child);

            // Bequeath remainder of h from parent
            int remainingParentH = parentH - (child.G - parentCost);
            if (child.H < remainingParentH)
                child.H = (ushort) remainingParentH;

            if (_bypassStrategy == BypassStrategy.BEST_FIT_LOOKAHEAD)
            {
                if (child.G == parentCost) // Total cost didn't increase (yet)
                    child.ParentAlreadyLookedAheadOf = true;
                else
                    child.ParentAlreadyLookedAheadOf = false;
            }

            if (child.F <= _maxSolutionCost)
            // Assuming h is an admissible heuristic, no need to generate nodes that won't get us
            // to the goal within the budget
            {
                _highLevelGenerated++;
                addToGlobalConflictCount(child); // TODO: Make MACBS_WholeTreeThreshold use nodes that do this automatically after choosing a conflict
                OpenList.Add(child);
            }
            else
            {
                _surplusNodesAvoided++;
            }
        }
    }

    /// <summary>
    /// Supports Prefering cardinal conflicts and BP1 with lookaheadMaxExpansions == 1,
    /// assumes we choose cardinal conflicts using one of the methods
    /// </summary>
    /// <param name="node"></param>
    private void IcbsExpand(CbsNode node)
    {
        int parentCost = node.G;
        int parentH = node.H;
        IList<CbsNode> children = new List<CbsNode>(2);
        bool reinsertParent;
        bool adopted = false;

        int origCardinalConflictSplits = _cardinalConflictSplits;
        int origSemiCardinalConflictSplits = _semiCardinalConflictSplits;
        int origNonCardinalConflictSplits = _nonCardinalConflictSplits;

        Trace.Assert(_bypassStrategy == BypassStrategy.NONE || _lookaheadMaxExpansions == 1, "Only BP1 is supported with cardinal conflict choice");  // Assumed in order to simplify code
        Trace.Assert(_bypassStrategy != BypassStrategy.BEST_FIT_LOOKAHEAD, "Only first-fit adoption is supported with cardinal conflict choice"); // Assumed in order to simplify code

        bool adoptionPerformedBefore = false;

        // Adoption and cardinal-lookahead cycle
        while (true)
        {
            (adopted, children, reinsertParent) = ExpandImpl(node, adopt: _bypassStrategy != BypassStrategy.NONE);

            if (_mergeCausesRestart && (_closedList.Count == 0)) // HACK: Means a restart was triggered
                break;

            if (adopted) // Either we found a goal, or we need to re-expand using a remaining conflict
            {
                if (adoptionPerformedBefore == false)
                {
                    adoptionPerformedBefore = true;
                    _bypasses++; // Only count one adoption for the entire look ahead subtree branch traversed.
                }
                _bypassLookAheadNodesCreated += children.Count + 1; // Either 1, if the first child was adopted, or 2 otherwise, +1 because the adopted child isn't returned.
                _cardinalConflictSplits = origCardinalConflictSplits;
                _semiCardinalConflictSplits = origSemiCardinalConflictSplits;
                _nonCardinalConflictSplits = origNonCardinalConflictSplits;

                if (node.GoalTest()) // Adoption found an admissible solution! (The original node to expand wasn't a goal)
                {
                    Debug.WriteLine("Bypass found a goal! Inserting it into OPEN.");
                    OpenList.Add(node);
                    return;
                }

                Trace.Assert(node.Conflict != null, "A conflict should have been found");
                // We might re-encounter conflicts we've already found aren't cardinal, but
                // since at least one path changed, the number of conflicts after adopting a child
                // might become smaller after adopting a child's solution this time
                continue;
            }

            if (
                children.All(child => child.G > node.G) && // All generated nodes have a higher cost
                ((node.AgentAExpansion == CbsNode.ExpansionState.EXPANDED && node.AgentBExpansion == CbsNode.ExpansionState.EXPANDED) || // Both children generated by now (possibly one in the past and one deferred to now)
                (node.AgentAExpansion == CbsNode.ExpansionState.EXPANDED && node.AgentBExpansion == CbsNode.ExpansionState.DEFERRED) || // One child still deferred, will surely have a higher cost (that's why it was deferred)
                (node.AgentAExpansion == CbsNode.ExpansionState.DEFERRED && node.AgentBExpansion == CbsNode.ExpansionState.EXPANDED)) // One child still deferred, will surely have a higher cost (that's why it was deferred)
                    ) // A cardinal conflict, or deferred expansion of a cardinal or semi-cardinal conflict.
            {
                if (_debug)
                    Debug.WriteLine("This was a cardinal conflict, or deferred expansion of a cardinal or semi-cardinal conflict. Inserting the generated children into OPEN.");

                break; // Look no further
            }
            else
            {
                // This wasn't a cardinal conflict, continue cycling through conflicts
            }

            if (children.Any(child => child.GoalTest() && child.G == node.G)) // Admissable goal found (and adoption isn't enabled, otherwise the goal would have been adopted above)
            {
                if (_debug)
                    Debug.WriteLine("Admissable goal found! Inserting the generated children into OPEN.");

                break; // Look no further
            }

            // This wasn't a cardinal conflict. Try to find a different conflict to try.
            if (node.ChooseNextPotentiallyCardinalConflicts() == false)
            {
                Debug.WriteLine("This was a non-cardinal conflict but there aren't more " +
                                "promising conflicts to try. Inserting the generated children into OPEN.");
                break; // Look no further
            }
            else
            {
                Debug.WriteLine($"This was a non-cardinal conflict. Trying potentially-cardinal conflict: {node.Conflict}");
            }

            // Prepare to restart the loop
            // Cancel partial expansion effects:
            node.AgentAExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
            node.AgentBExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
            node.H = parentH;
            // Take care of counts:
            _cardinalLookAheadNodesCreated += children.Count;
        }

        if (reinsertParent)
        {
            OpenList.Add(node); // Re-insert node into open list with higher cost, don't re-increment global conflict counts
        }
        else
        {
            node.ClearConflictChoiceData(); // We could do it even if there was a partial expansion and the node is
                                            // being re-inserted into OPEN.
                                            // If a lookahead chooser settled on
                                            // a semi-cardinal or non-cardinal, then there are no other conflicts left
                                            // so the conflict choosing state is unnecessary, but then ChooseNextConflict
                                            // would need to allow the iterator to be null.
                                            // If an MDD building chooser settled on a semi-cardinal or non-cardinal,
                                            // there could still be other conflicts, but they are known not to be cardinal.
        }

        // Path-Max stage
        // Reverse Path-Max (Mero, 1984) - operators aren't invertible, so we can only take min(h(Ci) + dist(P, Ci)) for h(P) (if it's higher)
        int minChildSumHAndOperatorCost = children.Max(child => child.H + (child.G - node.G));
        if (node.H < minChildSumHAndOperatorCost)
        {
            node.HBonus += minChildSumHAndOperatorCost - node.H;
            node.H = minChildSumHAndOperatorCost;
            ++_reversePathMaxBoosts;
        }

        // Forward Path-Max (Mero, 1984)
        foreach (var child in children)
        {
            _closedList.Add(child, child);

            // Bequeath remainder of h from parent
            int remainingParentH = parentH - (child.G - parentCost);
            if (child.H < remainingParentH)
            {
                child.H = (ushort)remainingParentH;
                _pathMaxBoosts++;
            }
            // "Path-Max Plus"
            if (node.MinimumVertexCover > 0 &&
                (ConflictChoice == ConflictChoice.CARDINAL_MDD || ConflictChoice == ConflictChoice.CARDINAL_LOOKAHEAD ||
                 ConflictChoice == ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_SMALLEST_GROUP ||
                 ConflictChoice == ConflictChoice.CARDINAL_MDD_THEN_MERGE_EARLY_MOST_CONFLICTING_AND_SMALLEST_GROUP
                 ))
            {
                child.H = (ushort)Math.Max(child.H, node.MinimumVertexCover - 1);  // -1 because we've just resolved a cardinal conflict.
                                                                                    // Even if the cost increased by more than 1 for the child, this is still our estimate, based on all the other conflicts
                _pathMaxPlusBoosts++;
            }
            if (child.F <= _maxSolutionCost)
            {
                _highLevelGenerated++;
                addToGlobalConflictCount(child);  // TODO: Make MACBS_WholeTreeThreshold use nodes that do this automatically after choosing a conflict
                OpenList.Add(child);
            }
            else
                _surplusNodesAvoided++;
        }
    }

    public virtual void Expand(CbsNode node)
    {
        if (_bypassStrategy == BypassStrategy.BEST_FIT_LOOKAHEAD && node.ParentAlreadyLookedAheadOf)
            Debug.WriteLine("Not looking ahead from this node, one of its ancestors of the same cost was already looked ahead of");

        if (ConflictChoice == ConflictChoice.FIRST || ConflictChoice == ConflictChoice.MOST_CONFLICTING_SMALLEST_AGENTS) // Then just choose a conflict once and stick with it.
        {
            ExpandIgnoringCardinalsButSupportingBP2(node);
        }
        else // We try to split according to cardinal conflicts
        {
            Trace.Assert(_bypassStrategy != BypassStrategy.BEST_FIT_LOOKAHEAD, "For simplicity, BP2 is not supported when choosing cardinal conflicts");
            IcbsExpand(node);
        }
    }

    /// <summary>
    /// </summary>
    /// <returns>A ValueTuple with:
    /// child: null if planning the child's path failed or it was a closed list hit, otherwise - the new child
    /// closedListHitChildCost: Used to tell if conflict is cardinal, semi-cardinal or
    /// non-cardinal when the child is a closed list hit.
    /// </returns>
    protected (CbsNode child, int closedListHitChildCost) MergeExpand(CbsNode node)
    {
        CbsConflict conflict = node.GetConflict();
        int closedListHitChildCost = -1;
            
        CbsNode child = new(node, node.AgentsGroupAssignment[conflict.agentAIndex], node.AgentsGroupAssignment[conflict.agentBIndex]);
        if (_closedList.ContainsKey(child) == false) // We may have already merged these agents in another node
        {
            Debug.WriteLine($"Merging agents {conflict.agentAIndex} and {conflict.agentBIndex}");
            int aCost = node.GetGroupCost(node.AgentsGroupAssignment[conflict.agentAIndex]);
            int bCost = node.GetGroupCost(node.AgentsGroupAssignment[conflict.agentBIndex]);
            int minNewCost;
            if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                minNewCost = aCost + bCost;
            else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                        Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                minNewCost = Math.Max(aCost, bCost);
            else
                throw new NotImplementedException($"Unsupported cost function {Constants.costFunction}");
            if (_useOldCost == false)
                minNewCost = -1;
            bool success = child.Replan(conflict.agentAIndex,  // or agentBIndex. Doesn't matter - they're in the same group.
                                        MinSolutionTimeStep, minPathCost: minNewCost);

            if (success == false) // A timeout probably occured
                return (child: null, closedListHitChildCost);

            Debug.WriteLine($"Child hash: {child.GetHashCode()}");
            Debug.WriteLine($"Child cost: {child.G}");
            Debug.WriteLine($"Child min ops to solve: {child.MinOpsToSolve}");
            Debug.WriteLine("");

            _maxSizeGroup = Math.Max(_maxSizeGroup, child.ReplanSize);

            return (child, closedListHitChildCost);
        }
        else
        {
            _closedListHits++;
            return (child: null, closedListHitChildCost: _closedList[child].G);
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns>A ValueTuple with:
    /// child: null if planning the child's path failed or it was a closed list hit,
    /// the parent node if expansion was deferred, or otherwise - the new child.
    /// closedListHitChildCost: Used to tell if conflict is cardinal, semi-cardinal or
    /// non-cardinal when the child is a closed list hit.
    /// </returns>
    /// </returns>
    protected (CbsNode child, int closedListHitChildCost) ConstraintExpand(CbsNode node, bool doLeftChild)
    {
        CbsConflict conflict = node.GetConflict();
        int conflictingAgentIndex = doLeftChild? conflict.agentAIndex : conflict.agentBIndex;
        CbsNode.ExpansionState expansionsState = doLeftChild ? node.AgentAExpansion : node.AgentBExpansion;
        CbsNode.ExpansionState otherChildExpansionsState = doLeftChild ? node.AgentBExpansion : node.AgentAExpansion;
        string agentSide = doLeftChild? "left" : "right";
        int planSize = node.SingleAgentPlans[conflictingAgentIndex].GetSize();  // Used to check if the conflict occurs while the agent is at its goal
        int groupSize = node.GetGroupSize(conflictingAgentIndex);
        CbsConflict.WillCostIncrease willCostIncrease = doLeftChild ? node.Conflict.willCostIncreaseForAgentA : node.Conflict.willCostIncreaseForAgentB;

        // Check if expansion should be deferred
        if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS && // Otherwise adding a constraint to an agent at a time step after
                                                                                // it reaches its goal doesn't necessarily increase the cost,
                                                                                // so we're not allowed to defer expansion.
                                                                                // I think in a makespan variant this optimization is inapplicable:
                                                                                // You can't have a conflict at the last step of the agent with the longest plan,
                                                                                // because who would conflict with it? An agent with a longer plan? (no such agent)
                                                                                // An agent with a plan of the same length? (but goals don't collide)
            ((Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG &&
            expansionsState == CbsNode.ExpansionState.NOT_EXPANDED && conflict.isVertexConflict == true &&  // An edge conflict at the goal isn't guaranteed
                                                                                                    // to increase the cost - the agent might be able to reach the goal from another direction
                conflict.timeStep >= node.SingleAgentCosts[conflictingAgentIndex] && // Can't just check whether the node is at its goal - 
                                                                                        // the plan may involve it passing through its goal and returning to it later because of preexisting constraints.
                                                                                        // This assumes unit move costs
                node.H < conflict.timeStep + 1 - node.SingleAgentCosts[conflictingAgentIndex] && // Otherwise we won't be increasing its h and there would be no reason to delay expansion
                                                                                                    // The agent's new cost will be at least conflict.timeStep + 1, so however much this is more than its current cost 
                                                                                                    // is an admissible heuristic
                groupSize == 1) || // Otherwise an agent in the group can be forced to take a longer
                                // route without increasing the group's cost because
                                // another agent would be able to take a shorter route.
            (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE &&
            expansionsState == CbsNode.ExpansionState.NOT_EXPANDED && conflict.isVertexConflict == true &&
            ((conflict.timeStep > planSize - 1 && node.H < 2) ||
                (conflict.timeStep == planSize - 1 && node.H < 1)) &&  // Otherwise we won't be increasing its h and there would be no reason to delay expansion
            groupSize == 1))) // Otherwise an agent in the group can be forced to take a longer
                                // route without increasing the group's cost because
                                // another agent would be able to take a shorter route.
        // Conflict happens when or after the agent reaches its goal, and the agent is in a single-agent group.
        // With multi-agent groups, banning the goal doesn't guarantee a higher cost solution,
        // since if an agent is forced to take a longer route it may enable another agent in the group
        // to take a shorter route, getting an alternative solution of the same cost
        // The child would cost a lot because:
        // A) All WAIT moves in the goal before leaving it now add to the g (if we're in the original problem variant).
        // B) We force the low level to compute a path longer than the optimal,
        //    and with a bad suprise towards the end in the form of a constraint,
        //    so the low-level's SIC heuristic performs poorly.
        // C) We're banning the GOAL from all directions (since this is a vertex conflict),
        //    so any alternative plan will at least cost 1 more.
        //    We're ignoring edge conflicts because they can only happen at the goal when reaching it,
        //    and aren't guaranteed to increase the cost because the goal can still be possibly reached from another edge.
        {
            // Defer expansion. The conflict happens while the agent is at its goal - the cost will
            // surely increase.
            if (otherChildExpansionsState == CbsNode.ExpansionState.DEFERRED)
                    throw new Exception("Unexpected: Expansion of both children deffered, " +
                        "but this is a vertex conflict so that means the targets for the " +
                        "two agents are equal, which is illegal");

            Debug.WriteLine($"Skipping {agentSide} child for now");
            if (doLeftChild)
                node.AgentAExpansion = CbsNode.ExpansionState.DEFERRED;
            else
                node.AgentBExpansion = CbsNode.ExpansionState.DEFERRED;
            // Add the minimal delta in the child's cost:
            // since we're banning the goal at conflict.timeStep, it must at least do conflict.timeStep+1 steps
            if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG)
            {
                node.H = Math.Max(node.H, (ushort)(conflict.timeStep + 1 - node.SingleAgentCosts[conflictingAgentIndex]));
                // Technically, we've already made sure above we're going to increase the node's h,
                // This is just to make the line look correct without reading the complex if statement above.
            }
            else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
            {
                if (conflict.timeStep > planSize - 1) // Agent will need to step out and step in to the goal, at least
                    node.H = Math.Max(node.H, (ushort) 1);
                else // Conflict is just when agent enters the goal, it'll have to at least wait one timestep.
                    node.H = Math.Max(node.H, (ushort) 1);
                // Technically, we've already made sure above we're going to increase the node's h,
                // This is just to make the line look correct without reading the complex if statement above.
            }
            _partialExpansions++;  // The other child was surely generated, so we can count a partial expansion here.
            return (node, closedListHitChildCost: -1);
        }
        else if (expansionsState != CbsNode.ExpansionState.EXPANDED)
        // Agent expansion already skipped in the past or not forcing it from its goal - finally generate the child:
        {
            Debug.WriteLine($"Generating {agentSide} child");

            if (doLeftChild)
                node.AgentAExpansion = CbsNode.ExpansionState.EXPANDED;
            else
                node.AgentBExpansion = CbsNode.ExpansionState.EXPANDED;

            CbsConstraint newConstraint = new(conflict, _instance, doLeftChild);
            CbsNode child = new(node, newConstraint, conflictingAgentIndex);

            if (DoMalte && doLeftChild == false)
            {
                // Add the case where both agents shouldn't be at the conflict point to the _left_ child
                // by forcing the first agent in the _right_ child to _always_ be at the conflict point.
                // Notice that vertex conflicts create must constraint with NO_DIRECTION and edge conflicts
                // don't, as necessary.
                CbsConstraint newMustConstraint = new(conflict, _instance, true);
                child.SetMustConstraint(newMustConstraint);
            }

            if (_closedList.ContainsKey(child) == false)
            {
                int oldCost = node.GetGroupCost(node.AgentsGroupAssignment[conflictingAgentIndex]);
                int minNewCost = oldCost;
                if (willCostIncrease == CbsConflict.WillCostIncrease.YES)
                    minNewCost = oldCost + 1;

                int maxNewCost = int.MaxValue;
                if (willCostIncrease == CbsConflict.WillCostIncrease.NO)
                    maxNewCost = oldCost;

                if (_useOldCost == false)
                {
                    minNewCost = -1;
                    maxNewCost = int.MaxValue;
                }

                bool success = child.Replan(conflictingAgentIndex, MinSolutionTimeStep,  // The node takes the max between minSolutionTimeStep and the max time over all constraints.
                                            minPathCost: minNewCost, maxPathCost: maxNewCost);

                if (success == false)
                    return (null, closedListHitChildCost: -1); // A timeout probably occured

                Debug.WriteLine($"Child hash: {child.GetHashCode()}");
                Debug.WriteLine($"Child cost: {child.G}");
                Debug.WriteLine($"Child min ops to solve: {child.MinOpsToSolve}");
                Debug.WriteLine($"Child num of agents that conflict: {child.TotalInternalAgentsThatConflict}");
                Debug.WriteLine($"Child num of internal conflicts: {child.TotalConflictsBetweenInternalAgents}");
                Debug.WriteLine("");

                if (child.G < node.G && groupSize == 1) // Catch the error early
                {
                    child.DebugPrint();
                    Debug.WriteLine("Child plan: (cost {0})", child.SingleAgentCosts[conflictingAgentIndex]);
                    child.SingleAgentPlans[conflictingAgentIndex].DebugPrint();
                    Debug.WriteLine("Parent plan: (cost {0})", node.SingleAgentCosts[conflictingAgentIndex]);
                    node.SingleAgentPlans[conflictingAgentIndex].DebugPrint();
                    Trace.Assert(false, $"Single agent node with lower cost than parent! {child.G} < {node.G}");
                }

                return (child, closedListHitChildCost: -1);
            }
            else
            {
                _closedListHits++;
                Debug.WriteLine("Child already in closed list!");
                // No need to check if we need to reopen - reopening is only necessary if F is lower,
                // but for the same constraints and group assignment we're going to have the same
                // g, and the h in the closed list can only be higher (if it was already expanded
                // and the open list's expensive heuristic was computed for it).
                return (child: null, closedListHitChildCost: _closedList[child].G);
            }
        }
        else
        {
            Debug.WriteLine("Child already generated before");
            return (child: null, closedListHitChildCost: -1);
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns>Whether the candidate's solution was adopted</returns>
    protected bool AdoptConditionally(CbsNode node, CbsNode adoptionCandidate, int nodeOrigH)
    {
        Debug.WriteLine($"Considering adoption of node hash: {adoptionCandidate.GetHashCode()}.");

        if (adoptionCandidate.G == node.G && // No need to branch :)
            adoptionCandidate.TieBreak(node, ignorePartialExpansion: true, ignoreDepth: true) == -1
            )
        {
            _conflictsBypassed += node.TotalConflictsWithExternalAgents + node.TotalConflictsBetweenInternalAgents
                                                    - adoptionCandidate.TotalConflictsWithExternalAgents - adoptionCandidate.TotalConflictsBetweenInternalAgents;
            node.AdoptSolutionOf(adoptionCandidate);
            node.H = nodeOrigH; // Cancel partial expansion h boost. This wasn't a cardinal conflict, so h could only have been from a partial expansion.
                                // FIXME: when semi-cardinal and non-cardinal conflicts start contributing to h, this won't be correct

            Debug.WriteLine("Child has same cost as parent and a better solution - child solution adopted by parent! (other generated children and partial expansions cancelled)");
            Debug.WriteLine("Node new details:");
            node.DebugPrint();
            return true;
        }
        Debug.WriteLine("Did not adopt.");
        Debug.WriteLine("");
        return false;
    }

    protected virtual void addToGlobalConflictCount(CbsConflict conflict) { }

    protected virtual void addToGlobalConflictCount(CbsNode node) { }

    public virtual Plan GetPlan()
    {
        if (_solution == null)
            _solution = _goalNode.CalculateJointPlan();
        return _solution;
    }

    public int GetSolutionDepth() { return _solutionDepth; }
        
    public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        
    public SinglePlan[] GetSinglePlans() => _goalNode.SingleAgentPlans;

    public virtual int[] GetSingleCosts() => _goalNode.SingleAgentCosts;

    public int GetHighLevelExpanded() => _highLevelExpanded;
    public int GetHighLevelGenerated()  => _highLevelGenerated;
    public int GetLowLevelExpanded()  => _solver.GetAccumulatedExpanded();
    public int GetLowLevelGenerated() => _solver.GetAccumulatedGenerated();
    public int GetExpanded() => _highLevelExpanded;
    public int GetGenerated() => _highLevelGenerated;
    public int GetAccumulatedExpanded() => _accHLExpanded;
    public int GetAccumulatedGenerated() => _accHLGenerated;
    public int GetMaxGroupSize() => _maxSizeGroup;
}

/// <summary>
/// Merges agents if they conflict more times than the given threshold in all the CT.
/// </summary>
public class MACBS_WholeTreeThreshold : CBS
{
    /// <summary>
    /// Counts conflicts between pairs of individual agents. To get counts for meta-agents, sum the values for their concrete agents.
    /// </summary>
    public int[][] globalConflictsCounter;

    public MACBS_WholeTreeThreshold(ICbsSolver singleAgentSolver, ICbsSolver generalSolver,
                                int mergeThreshold = -1,
                                BypassStrategy bypassStrategy = BypassStrategy.NONE,
                                bool doMalte = false,
                                ConflictChoice conflictChoice = ConflictChoice.FIRST,
                                ILazyHeuristic<CbsNode> heuristic = null,
                                bool disableTieBreakingByMinOpsEstimate = false,
                                int lookaheadMaxExpansions = 1,
                                bool mergeCausesRestart = false)
        : base(singleAgentSolver, generalSolver, mergeThreshold,
                bypassStrategy, doMalte, conflictChoice, heuristic,
                disableTieBreakingByMinOpsEstimate, lookaheadMaxExpansions,
                mergeCausesRestart)
    {
        //throw new NotImplementedException("Not supported until we decide how to count conflicts. Used to rely on the specific conflict chosen in each node.");
    }

    /// <summary>
    /// Implement ISolver.Setup.
    /// Assumes agent nums start from 0 and are consecutive.
    /// </summary>
    /// <param name="problemInstance"></param>
    /// <param name="runner"></param>
    public override void Setup(ProblemInstance problemInstance, Run runner)
    {
        MakeConflictMatrix(problemInstance);
        base.Setup(problemInstance, runner);
    }

    private void MakeConflictMatrix(ProblemInstance problemInstance)
    {
        globalConflictsCounter = new int[problemInstance.agents.Length][];
        for (int i = 0; i < globalConflictsCounter.Length; i++)
        {
            globalConflictsCounter[i] = new int[i];
            for (int j = 0; j < i; j++)
            {
                globalConflictsCounter[i][j] = 0;
            }
        }
    }

    protected override bool ShouldMerge(CbsNode node)
    {
        return node.ShouldMerge(MergeThreshold, globalConflictsCounter, node.Conflict.agentAIndex, node.Conflict.agentBIndex);  //TODO: Add an option to choose the old merge criterion
    }

    /// <summary>
    /// Old logic. Only add the chosen conflict to the counts
    /// </summary>
    /// <param name="conflict"></param>
    protected override void addToGlobalConflictCount(CbsConflict conflict)
    {
        if (conflict != null)
            globalConflictsCounter[Math.Max(conflict.agentAIndex, conflict.agentBIndex)][Math.Min(conflict.agentAIndex, conflict.agentBIndex)]++;
    }

    protected override void addToGlobalConflictCount(CbsNode node)
    {
        if (_mergeCausesRestart == false)
        {
            // Old logic:
            if (node.Conflict != null)
                globalConflictsCounter[Math.Max(node.Conflict.agentAIndex, node.Conflict.agentBIndex)][Math.Min(node.Conflict.agentAIndex, node.Conflict.agentBIndex)]++;
            // This only looks at conflicts between individual agents - doesn't take into account merges.
            // Merges are local when merge & restart is off so the global counts might be incorrect.
            // FIXME: Currently only looks at the chosen conflict.
        }
        else
        {
            //TODO: Add an option to use the old logic above here too
            for (int i = 0; i < GetProblemInstance().GetNumOfAgents(); i++)
            {
                if (node.NewPlans[i] == false)
                    continue;
                foreach (var kvp in node.ConflictCountsPerAgent[i])
                {
                    if (i > kvp.Key)
                        continue;
                    int groupRepA = node.AgentsGroupAssignment[i];
                    int groupRepB = node.AgentsGroupAssignment[kvp.Key];
                    if (groupRepA > groupRepB)
                        globalConflictsCounter[groupRepA][groupRepB] += kvp.Value;
                    else
                        globalConflictsCounter[groupRepB][groupRepA] += kvp.Value;
                }
            }
        }
    }

    public override string GetName()
    {
        string baseName = base.GetName();
        return baseName.Replace("Local", "Global");
    }

    public override void Reset()
    {
        base.Reset();
        //MakeConflictMatrix(instance);
    }
}
