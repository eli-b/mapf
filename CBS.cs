using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;
using System.Linq;

using ExtensionMethods;

namespace CPF_experiment
{
    /// <summary>
    /// Merges agents if they conflict more times than the given threshold in the CT nodes from the root to the current CT nodes only.
    /// </summary>
    public class CBS : ICbsSolver, IHeuristicSolver<CbsNode>
    {
        /// <summary>
        /// The key of the constraints list used for each CBS node
        /// </summary>
        public static readonly string CONSTRAINTS = "constraints";
        /// <summary>
        /// The key of the must constraints list used for each CBS node
        /// </summary>
        public static readonly string MUST_CONSTRAINTS = "must constraints";
        /// <summary>
        /// The key of the internal CAT for CBS, used to favor A* nodes that have fewer conflicts with other routes during tie-breaking.
        /// Also used to indicate that CBS is running.
        /// </summary>
        public static readonly string CAT = "CBS CAT";

        /// <summary>
        /// For each agent, map sets of constraints to MDDs
        /// </summary>
        public Dictionary<CbsCacheEntry, MDD>[] mddCache;

        /// <summary>
        /// For each agent, map sets of constraints to a dictionary that
        /// maps each level (timestep) of its mdd to a narrowness degree.
        /// Non-narrow levels are omitted.
        /// </summary>
        public Dictionary<CbsCacheEntry, Dictionary<int, MDD.LevelNarrowness>>[] mddNarrownessValuesCache;

        protected ProblemInstance instance;
        public OpenList<CbsNode> openList;
        /// <summary>
        /// Might as well be a HashSet. We don't need to retrieve from it.
        /// TODO: Consider a closedList for single agent paths under an unordered set of constraints.
        ///       It would get more hits.
        /// </summary>
        public Dictionary<CbsNode, CbsNode> closedList;
        protected ILazyHeuristic<CbsNode> heuristic;
        protected int highLevelExpanded;
        protected int highLevelGenerated;
        protected int closedListHits;
        protected int partialExpansions;
        protected int bypasses;
        protected int nodesExpandedWithGoalCost;
        protected int bypassLookAheadNodesCreated;
        protected int cardinalLookAheadNodesCreated;
        protected int conflictsBypassed;
        protected int cardinalConflictSplits;
        protected int semiCardinalConflictSplits;
        protected int nonCardinalConflictSplits;
        public int mddsBuilt;
        public int mddsAdapted;
        protected int restarts;
        protected int pathMaxBoosts;
        protected int reversePathMaxBoosts;
        protected int pathMaxPlusBoosts;
        protected int surplusNodesAvoided;
        public int mddCacheHits;
        public double timePlanningPaths;
        public double timeBuildingMdds;
        // TODO: Count shuffles
        protected int accHLExpanded;
        protected int accHLGenerated;
        protected int accClosedListHits;
        protected int accPartialExpansions;
        protected int accBypasses;
        protected int accNodesExpandedWithGoalCost;
        protected int accBypassLookAheadNodesCreated;
        protected int accCardinalLookAheadNodesCreated;
        protected int accConflictsBypassed;
        protected int accCardinalConflictSplits;
        protected int accSemiCardinalConflictSplits;
        protected int accNonCardinalConflictSplits;
        protected int accMddsBuilt;
        protected int accMddsAdapted;
        protected int accRestarts;
        protected int accPathMaxBoosts;
        protected int accReversePathMaxBoosts;
        protected int accPathMaxPlusBoosts;
        protected int accSurplusNodesAvoided;
        protected int accMddCacheHits;
        protected double accTimePlanningPaths;
        protected double accTimeBuildingMdds;

        public int solutionCost;
        /// <summary>
        /// The difference between the solution's cost and the f of the root node.
        /// Notice root.g != 0 in CBS.
        /// </summary>
        protected int solutionDepth;
        public Run runner;
        protected CbsNode goalNode;
        protected Plan solution;
        /// <summary>
        /// Nodes with a higher F aren't generated. As a result, goal nodes with a higher cost
        /// won't be found.
        /// </summary>
        protected int maxSolutionCost;
        /// <summary>
        /// Goal Nodes with with a lower cost aren't considered a goal. Used directly by CbsNode.
        /// </summary>
        public int minSolutionCost;
        /// <summary>
        /// Search is stopped when the minimum F in the open list reaches the target,
        /// regardless of whether a goal node was found. Note maxSolutionCost stops the search when
        /// the same F value is exhausted from the open list later.
        /// </summary>
        public int targetF {
            get
            {
                return this.m_targetF;
            }
            set
            {
                this.maxSolutionCost = Math.Min(this.maxSolutionCost, value);  // No need to generate nodes with a higher F
                this.m_targetF = value;
            }
        }
        protected int m_targetF;
        /// <summary>
        /// Search is stopped when the low level generated nodes count exceeds the cap
        /// </summary>
        public int lowLevelGeneratedCap { set; get; }
        /// <summary>
        /// Search is stopped when the millisecond count exceeds the cap
        /// </summary>
        public int milliCap { set; get; }
        protected ICbsSolver solver;
        protected ICbsSolver singleAgentSolver;
        public int mergeThreshold;
        protected int minSolutionTimeStep;
        protected int maxSizeGroup;
        protected int accMaxSizeGroup;
        /// <summary>
        /// Used to know when to clear problem parameters.
        /// </summary>
        public bool topMost;
        public bool doShuffle;
        public BypassStrategy bypassStrategy;
        public bool doMalte;
        public ConflictChoice conflictChoice;
        public bool disableTieBreakingByMinOpsEstimate;
        public int lookaheadMaxExpansions;
        public enum ConflictChoice : byte
        {
            FIRST = 0,
            MOST_CONFLICTING_SMALLEST_AGENTS,
            CARDINAL_MDD,
            CARDINAL_LOOKAHEAD,
        }
        public enum BypassStrategy : byte
        {
            NONE = 0,
            FIRST_FIT_LOOKAHEAD,
            BEST_FIT_LOOKAHEAD
        }
        private bool mergeCausesRestart;
        private bool useOldCost;
        public bool replanSameCostWithMdd;
        public bool cacheMdds;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="singleAgentSolver"></param>
        /// <param name="generalSolver"></param>
        /// <param name="mergeThreshold"></param>
        /// <param name="doShuffle"></param>
        /// <param name="bypassStrategy"></param>
        /// <param name="doMalte"></param>
        /// <param name="conflictChoice"></param>
        /// <param name="heuristic"></param>
        /// <param name="disableTieBreakingByMinOpsEstimate"></param>
        /// <param name="lookaheadMaxExpansions"></param>
        /// <param name="mergeCausesRestart"></param>
        public CBS(ICbsSolver singleAgentSolver, ICbsSolver generalSolver,
                                  int mergeThreshold = -1,
                                  bool doShuffle = false,
                                  BypassStrategy bypassStrategy = BypassStrategy.NONE,
                                  bool doMalte = false,
                                  ConflictChoice conflictChoice = ConflictChoice.FIRST,
                                  ILazyHeuristic<CbsNode> heuristic = null,
                                  bool disableTieBreakingByMinOpsEstimate = true,
                                  int lookaheadMaxExpansions = 1,
                                  bool mergeCausesRestart = false,
                                  bool replanSameCostWithMdd = false,
                                  bool cacheMdds = false,
                                  bool useOldCost = false
            )
        {
            this.closedList = new Dictionary<CbsNode, CbsNode>();
            if (heuristic == null)
                this.openList = new OpenList<CbsNode>(this);
            else
                this.openList = new DynamicLazyOpenList<CbsNode>(this, heuristic);
            this.mergeThreshold = mergeThreshold;
            this.solver = generalSolver;
            this.singleAgentSolver = singleAgentSolver;
            this.doShuffle = doShuffle;
            this.bypassStrategy = bypassStrategy;
            this.doMalte = doMalte;
            this.conflictChoice = conflictChoice;
            this.heuristic = heuristic;
            if (Constants.costFunction != Constants.CostFunction.SUM_OF_COSTS)
            {
                Debug.Assert(conflictChoice != ConflictChoice.CARDINAL_MDD &&
                             conflictChoice != ConflictChoice.CARDINAL_LOOKAHEAD,  // TODO: Might be OK. Need to look at it.
                    "Under makespan, increasing the cost for a single agent might not increase the cost for the solution." +
                    "Before this strategy is enabled we need to add a consideration of whether the agent whose cost will " +
                    "increase has the highest cost in the solution first");
            }
            this.disableTieBreakingByMinOpsEstimate = disableTieBreakingByMinOpsEstimate;
            this.lookaheadMaxExpansions = lookaheadMaxExpansions;
            this.mergeCausesRestart = mergeCausesRestart;
            this.replanSameCostWithMdd = replanSameCostWithMdd;
            this.cacheMdds = cacheMdds;
            this.useOldCost = useOldCost;
        }
        
        /// <summary>
        /// 
        /// </summary>
        /// <param name="problemInstance"></param>
        /// <param name="minSolutionTimeStep"></param>
        /// <param name="runner"></param>
        /// <param name="minSolutionCost"></param>
        /// <param name="maxSolutionCost"></param>
        /// <param name="mdd">Currently ignored. FIXME: Need to convert to array of MDDs to use.</param>
        public virtual void Setup(ProblemInstance problemInstance, int minSolutionTimeStep, Run runner,
            int minSolutionCost = -1, int maxSolutionCost = int.MaxValue, MDD mdd = null)
        {
            this.instance = problemInstance;
            this.runner = runner;
            if (this.openList is DynamicLazyOpenList<CbsNode>)
                ((DynamicLazyOpenList<CbsNode>)this.openList).runner = runner;

            this.ClearPrivateStatistics();
            this.solutionCost = 0;
            this.solutionDepth = -1;
            this.lowLevelGeneratedCap = int.MaxValue;
            this.targetF = int.MaxValue;
            this.milliCap = int.MaxValue;
            this.goalNode = null;
            this.solution = null;

            // Independence Detection parameters
            if (problemInstance.parameters.ContainsKey(IndependenceDetection.MAXIMUM_COST_KEY))
                this.maxSolutionCost = (int)problemInstance.parameters[IndependenceDetection.MAXIMUM_COST_KEY];
            else
                this.maxSolutionCost = int.MaxValue;

            // CBS parameters
            this.minSolutionTimeStep = minSolutionTimeStep;
            this.minSolutionCost = minSolutionCost;
            this.maxSolutionCost = Math.Max(this.maxSolutionCost, maxSolutionCost);
            if (this.replanSameCostWithMdd)
                Debug.Assert(this.mergeThreshold == -1, "Using MDDs to replan same-cost paths is currently only supported for single agents");

            if (this.cacheMdds)
            {
                this.mddCache = new Dictionary<CbsCacheEntry, MDD>[instance.agents.Length];
                this.mddNarrownessValuesCache = new Dictionary<CbsCacheEntry, Dictionary<int, MDD.LevelNarrowness>>[instance.agents.Length];
                for (int i = 0; i < instance.agents.Length; i++)
                {
                    this.mddCache[i] = new Dictionary<CbsCacheEntry, MDD>();
                    this.mddNarrownessValuesCache[i] = new Dictionary<CbsCacheEntry, Dictionary<int, MDD.LevelNarrowness>>();
                }
            }

            this.topMost = this.SetGlobals();

            CbsNode root = new CbsNode(instance.agents.Length, this.solver, this.singleAgentSolver, this); // Problem instance and various strategy data is all passed under 'this'.
            // Solve the root node
            bool solved = root.Solve(minSolutionTimeStep);

            if (solved)
            {
                if (root.f <= this.maxSolutionCost)
                {
                    this.openList.Add(root);
                    this.highLevelGenerated++;
                    this.closedList.Add(root, root);
                }
                else
                {
                    this.surplusNodesAvoided++;
                }
            }
        }

        public virtual void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.Setup(problemInstance, 0, runner);
        }

        public IHeuristicCalculator<CbsNode> GetHeuristic()
        {
            return this.heuristic;
        }

        public ICbsSolver GetSolver()
        {
            return this.solver;
        }

        public Dictionary<int, int> GetExternalConflictCounts()
        {
            throw new NotImplementedException(); // For now. Also need to take care of generalised goal nodes!
        }

        public Dictionary<int, List<int>> GetConflictTimes()
        {
            throw new NotImplementedException(); // For now. Also need to take care of generalised goal nodes!
        }

        public ProblemInstance GetProblemInstance()
        {
            return this.instance;
        }

        public void Clear()
        {
            this.openList.Clear();
            this.closedList.Clear();
            this.solver.Clear();
            // Statistics are reset on Setup.
        }

        public virtual string GetName() 
        {
            string lowLevelSolvers;
            if (mergeThreshold == -1 || Object.ReferenceEquals(this.singleAgentSolver, this.solver))
                lowLevelSolvers = $"({this.singleAgentSolver})";
            else
                lowLevelSolvers = $"(single:{singleAgentSolver} multi:{solver})";
            string variants = "";
            if (this.doShuffle)
                variants += " with shuffle";
            if (this.bypassStrategy == BypassStrategy.FIRST_FIT_LOOKAHEAD)
            {
                if (this.lookaheadMaxExpansions != int.MaxValue)
                {
                    if (this.lookaheadMaxExpansions != 1)
                        variants += $" with first fit adoption max expansions: {this.lookaheadMaxExpansions}";
                    else
                        variants += " + BP";
                }
                else
                    variants += " with first fit adoption max expansions: $\\infty$"; // LaTeX infinity symbol
            }
            if (this.bypassStrategy == BypassStrategy.BEST_FIT_LOOKAHEAD)
            {
                if (this.lookaheadMaxExpansions == int.MaxValue)
                    variants += " with infinite lookahead best fit adoption";
                else
                    variants += $" with {this.lookaheadMaxExpansions} lookahead best fit adoption";
            }
            if (this.doMalte)
                variants += " with Malte";

            if (this.conflictChoice == ConflictChoice.FIRST)
                variants += " choosing the first conflict in CBS nodes";
            else if (this.conflictChoice == ConflictChoice.CARDINAL_MDD)
                variants += " + PC";
            else if (this.conflictChoice == ConflictChoice.CARDINAL_LOOKAHEAD)
                variants += " choosing cardinal conflicts using lookahead";

            if (this.disableTieBreakingByMinOpsEstimate == true)
                variants += " without smart tie breaking";

            if (this.mergeCausesRestart == true && mergeThreshold != -1)
                variants += " with merge&restart";

            if (this.replanSameCostWithMdd)
                variants += " with replanning same cost paths with MDDs";

            if (this.cacheMdds)
                variants += " with caching MDDs";

            if (this.useOldCost)
                variants += " with using old path costs";

            if (this.openList.GetType() != typeof(OpenList<CbsNode>))
            {
                variants += $" using {this.openList.GetName()}";
            }

            if (mergeThreshold == -1)
                return $"CBS/{lowLevelSolvers}{variants}";
            return $"MA-CBS-Local-{mergeThreshold}/{lowLevelSolvers}{variants}";
        }

        public override string ToString()
        {
            return GetName();
        }

        public int GetSolutionCost() { return this.solutionCost; }

        protected void ClearPrivateStatistics()
        {
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 0;
            this.closedListHits = 0;
            this.partialExpansions = 0;
            this.bypasses = 0;
            this.nodesExpandedWithGoalCost = 0;
            this.bypassLookAheadNodesCreated = 0;
            this.cardinalLookAheadNodesCreated = 0;
            this.conflictsBypassed = 0;
            this.cardinalConflictSplits = 0;
            this.semiCardinalConflictSplits = 0;
            this.nonCardinalConflictSplits = 0;
            this.mddsBuilt = 0;
            this.mddsAdapted = 0;
            this.restarts = 0;
            this.pathMaxBoosts = 0;
            this.reversePathMaxBoosts = 0;
            this.pathMaxPlusBoosts = 0;
            this.maxSizeGroup = 1;
            this.surplusNodesAvoided = 0;
            this.mddCacheHits = 0;
            this.timePlanningPaths = 0;
            this.timeBuildingMdds = 0;
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Closed List Hits (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Partial Expansions (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Adoptions (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Nodes Expanded With Goal Cost (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Bypass Look Ahead Nodes Created (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Cardinal Look Ahead Nodes Created (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Conflicts Bypassed With Adoption (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Cardinal Conflict Splits (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Semi-Cardinal Conflict Splits (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Non-Cardinal Conflict Splits (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " MDDs Built (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " MDDs Adapted (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Restarts (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Path-Max Boosts (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Reverse Path-Max Boosts (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Path-Max Plus Boosts (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Max Group Size (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Surplus Nodes Avoided (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " MDD Cache Hits (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Time Planning Paths (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Time Building MDDs (HL)");
            output.Write(Run.RESULTS_DELIMITER);

            this.solver.OutputStatisticsHeader(output);
            if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                this.singleAgentSolver.OutputStatisticsHeader(output);

            this.openList.OutputStatisticsHeader(output);
        }

        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes (High-Level): {0}", this.GetHighLevelExpanded());
            Console.WriteLine("Total Generated Nodes (High-Level): {0}", this.GetHighLevelGenerated());
            Console.WriteLine("Closed List Hits (High-Level): {0}", this.closedListHits);
            Console.WriteLine("Partial Expansions (High-Level): {0}", this.partialExpansions);
            Console.WriteLine("Adoptions (High-Level): {0}", this.bypasses);
            Console.WriteLine("Nodes expanded with goal cost (High-Level): {0}", this.nodesExpandedWithGoalCost);
            Console.WriteLine("Bypass lookahead nodes created (High-Level): {0}", this.bypassLookAheadNodesCreated);
            Console.WriteLine("Cardinal lookahead nodes created (High-Level): {0}", this.cardinalLookAheadNodesCreated);
            Console.WriteLine("Conflicts Bypassed With Adoption (High-Level): {0}", this.conflictsBypassed);
            Console.WriteLine("Cardinal Conflicts Splits (High-Level): {0}", this.cardinalConflictSplits);
            Console.WriteLine("Semi-Cardinal Conflicts Splits (High-Level): {0}", this.semiCardinalConflictSplits);
            Console.WriteLine("Non-Cardinal Conflicts Splits (High-Level): {0}", this.nonCardinalConflictSplits);
            Console.WriteLine("MDDs Built (High-Level): {0}", this.mddsBuilt);
            Console.WriteLine("MDDs adapted (High-Level): {0}", this.mddsAdapted);
            Console.WriteLine("Restarts (High-Level): {0}", this.restarts);
            Console.WriteLine("Path-Max Boosts (High-Level): {0}", this.pathMaxBoosts);
            Console.WriteLine("Reverse Path-Max Boosts (High-Level): {0}", this.reversePathMaxBoosts);
            Console.WriteLine("Path-Max Plus Boosts (High-Level): {0}", this.pathMaxPlusBoosts);
            Console.WriteLine("Max Group Size (High-Level): {0}", this.maxSizeGroup);
            Console.WriteLine("Surplus Nodes Avoided (High-Level): {0}", this.surplusNodesAvoided);
            Console.WriteLine("MDD cache hits (High-Level): {0}", this.mddCacheHits);
            Console.WriteLine("Time planning paths (High-Level): {0}", this.timePlanningPaths);
            Console.WriteLine("Time building mdds (High-Level): {0}", this.timeBuildingMdds);

            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.closedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.partialExpansions + Run.RESULTS_DELIMITER);
            output.Write(this.bypasses + Run.RESULTS_DELIMITER);
            output.Write(this.nodesExpandedWithGoalCost + Run.RESULTS_DELIMITER);
            output.Write(this.bypassLookAheadNodesCreated + Run.RESULTS_DELIMITER);
            output.Write(this.cardinalLookAheadNodesCreated + Run.RESULTS_DELIMITER);
            output.Write(this.conflictsBypassed + Run.RESULTS_DELIMITER);
            output.Write(this.cardinalConflictSplits + Run.RESULTS_DELIMITER);
            output.Write(this.semiCardinalConflictSplits + Run.RESULTS_DELIMITER);
            output.Write(this.nonCardinalConflictSplits + Run.RESULTS_DELIMITER);
            output.Write(this.mddsBuilt + Run.RESULTS_DELIMITER);
            output.Write(this.mddsAdapted + Run.RESULTS_DELIMITER);
            output.Write(this.restarts + Run.RESULTS_DELIMITER);
            output.Write(this.pathMaxBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.reversePathMaxBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.pathMaxPlusBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.maxSizeGroup + Run.RESULTS_DELIMITER);
            output.Write(this.surplusNodesAvoided + Run.RESULTS_DELIMITER);
            output.Write(this.mddCacheHits + Run.RESULTS_DELIMITER);
            output.Write(this.timePlanningPaths + Run.RESULTS_DELIMITER);
            output.Write(this.timeBuildingMdds + Run.RESULTS_DELIMITER);

            this.solver.OutputAccumulatedStatistics(output);
            if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                this.singleAgentSolver.OutputAccumulatedStatistics(output);

            this.openList.OutputStatistics(output);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                int numSolverStats = this.solver.NumStatsColumns;
                if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                    numSolverStats += this.singleAgentSolver.NumStatsColumns;
                return 23 + numSolverStats + this.openList.NumStatsColumns;
            }
        }

        public virtual void ClearStatistics()
        {
            if (this.topMost)
            {
                this.solver.ClearAccumulatedStatistics(); // Is this correct? Or is it better not to do it?
                if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                    this.singleAgentSolver.ClearAccumulatedStatistics();
            }
            this.ClearPrivateStatistics();
            this.openList.ClearStatistics();
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.accHLExpanded = 0;
            this.accHLGenerated = 0;
            this.accClosedListHits = 0;
            this.accPartialExpansions = 0;
            this.accBypasses = 0;
            this.accNodesExpandedWithGoalCost = 0;
            this.accBypassLookAheadNodesCreated = 0;
            this.accCardinalLookAheadNodesCreated = 0;
            this.accConflictsBypassed = 0;
            this.accCardinalConflictSplits = 0;
            this.accSemiCardinalConflictSplits = 0;
            this.accNonCardinalConflictSplits = 0;
            this.accMddsBuilt = 0;
            this.accMddsAdapted = 0;
            this.accRestarts = 0;
            this.accPathMaxBoosts = 0;
            this.accReversePathMaxBoosts = 0;
            this.accPathMaxPlusBoosts = 0;
            this.accMaxSizeGroup = 1;
            this.accSurplusNodesAvoided = 0;
            this.accMddCacheHits = 0;
            this.accTimePlanningPaths = 0;
            this.accTimeBuildingMdds = 0;

            this.solver.ClearAccumulatedStatistics();
            if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                this.singleAgentSolver.ClearAccumulatedStatistics();

            this.openList.ClearAccumulatedStatistics();
        }

        public virtual void AccumulateStatistics()
        {
            this.accHLExpanded += this.highLevelExpanded;
            this.accHLGenerated += this.highLevelGenerated;
            this.accClosedListHits += this.closedListHits;
            this.accPartialExpansions += this.partialExpansions;
            this.accBypasses += this.bypasses;
            this.accNodesExpandedWithGoalCost += this.nodesExpandedWithGoalCost;
            this.accBypassLookAheadNodesCreated += this.bypassLookAheadNodesCreated;
            this.accCardinalLookAheadNodesCreated += this.cardinalLookAheadNodesCreated;
            this.accConflictsBypassed += this.conflictsBypassed;
            this.accCardinalConflictSplits += this.cardinalConflictSplits;
            this.accSemiCardinalConflictSplits += this.semiCardinalConflictSplits;
            this.accNonCardinalConflictSplits += this.nonCardinalConflictSplits;
            this.accMddsBuilt += this.mddsBuilt;
            this.accMddsAdapted += this.mddsAdapted;
            this.accRestarts += this.restarts;
            this.accPathMaxBoosts += this.pathMaxBoosts;
            this.accReversePathMaxBoosts += this.reversePathMaxBoosts;
            this.accPathMaxPlusBoosts += this.pathMaxPlusBoosts;
            this.accMaxSizeGroup = Math.Max(this.accMaxSizeGroup, this.maxSizeGroup);
            this.accSurplusNodesAvoided += this.surplusNodesAvoided;
            this.accMddCacheHits += this.mddCacheHits;
            this.accTimePlanningPaths += this.timePlanningPaths;
            this.accTimeBuildingMdds += this.timeBuildingMdds;

            // this.solver statistics are accumulated every time it's used.

            this.openList.AccumulateStatistics();
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Accumulated Expanded Nodes (High-Level): {1}", this, this.accHLExpanded);
            Console.WriteLine("{0} Accumulated Generated Nodes (High-Level): {1}", this, this.accHLGenerated);
            Console.WriteLine("{0} Accumulated Closed List Hits (High-Level): {1}", this, this.accClosedListHits);
            Console.WriteLine("{0} Accumulated Partial Expansions (High-Level): {1}", this, this.accPartialExpansions);
            Console.WriteLine("{0} Accumulated Adoptions (High-Level): {1}", this, this.accBypasses);
            Console.WriteLine("{0} Accumulated Nodes Expanded With Goal Cost (High-Level): {1}", this, this.accNodesExpandedWithGoalCost);
            Console.WriteLine("{0} Accumulated Look Ahead Nodes Created (High-Level): {1}", this, this.accNodesExpandedWithGoalCost);
            Console.WriteLine("{0} Accumulated Conflicts Bypassed With Adoption (High-Level): {1}", this, this.accConflictsBypassed);
            Console.WriteLine("{0} Accumulated Cardinal Conflicts Splits (High-Level): {1}", this, this.accCardinalConflictSplits);
            Console.WriteLine("{0} Accumulated Semi-Cardinal Conflicts Splits (High-Level): {1}", this, this.accSemiCardinalConflictSplits);
            Console.WriteLine("{0} Accumulated Non-Cardinal Conflicts Splits (High-Level): {1}", this, this.accNonCardinalConflictSplits);
            Console.WriteLine("{0} Accumulated MDDs Built (High-Level): {1}", this, this.accMddsBuilt);
            Console.WriteLine("{0} Accumulated MDDs adapted (High-Level): {1}", this, this.accMddsAdapted);
            Console.WriteLine("{0} Accumulated Restarts (High-Level): {1}", this, this.accRestarts);
            Console.WriteLine("{0} Accumulated Path-Max Boosts (High-Level): {1}", this, this.accPathMaxBoosts);
            Console.WriteLine("{0} Accumulated Reverse Path-Max Boosts (High-Level): {1}", this, this.accReversePathMaxBoosts);
            Console.WriteLine("{0} Accumulated Path-Max Plus Boosts (High-Level): {1}", this, this.accPathMaxPlusBoosts);
            Console.WriteLine("{0} Max Group Size (High-Level): {1}", this, this.accMaxSizeGroup);
            Console.WriteLine("{0} Accumulated Surplus Nodes Avoided (High-Level): {1}", this, this.accSurplusNodesAvoided);
            Console.WriteLine("{0} Accumulated MDD cache hits (High-Level): {1}", this, this.accMddCacheHits);
            Console.WriteLine("{0} Accumulated time planning paths (High-Level): {1}", this, this.accTimePlanningPaths);
            Console.WriteLine("{0} Accumulated time building MDDs (High-Level): {1}", this, this.accTimeBuildingMdds);

            output.Write(this.accHLExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accHLGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.accClosedListHits + Run.RESULTS_DELIMITER);
            output.Write(this.accPartialExpansions + Run.RESULTS_DELIMITER);
            output.Write(this.accBypasses + Run.RESULTS_DELIMITER);
            output.Write(this.accNodesExpandedWithGoalCost + Run.RESULTS_DELIMITER);
            output.Write(this.accBypassLookAheadNodesCreated + Run.RESULTS_DELIMITER);
            output.Write(this.accCardinalLookAheadNodesCreated + Run.RESULTS_DELIMITER);
            output.Write(this.accConflictsBypassed + Run.RESULTS_DELIMITER);
            output.Write(this.accCardinalConflictSplits + Run.RESULTS_DELIMITER);
            output.Write(this.accSemiCardinalConflictSplits + Run.RESULTS_DELIMITER);
            output.Write(this.accNonCardinalConflictSplits + Run.RESULTS_DELIMITER);
            output.Write(this.accMddsBuilt + Run.RESULTS_DELIMITER);
            output.Write(this.accMddsAdapted + Run.RESULTS_DELIMITER);
            output.Write(this.accRestarts + Run.RESULTS_DELIMITER);
            output.Write(this.accPathMaxBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.accReversePathMaxBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.accPathMaxPlusBoosts + Run.RESULTS_DELIMITER);
            output.Write(this.accMaxSizeGroup + Run.RESULTS_DELIMITER);
            output.Write(this.accSurplusNodesAvoided + Run.RESULTS_DELIMITER);
            output.Write(this.accMddCacheHits + Run.RESULTS_DELIMITER);
            output.Write(this.accTimePlanningPaths + Run.RESULTS_DELIMITER);
            output.Write(this.accTimeBuildingMdds + Run.RESULTS_DELIMITER);

            this.solver.OutputAccumulatedStatistics(output);
            if (Object.ReferenceEquals(this.singleAgentSolver, this.solver) == false)
                this.singleAgentSolver.OutputAccumulatedStatistics(output);

            this.openList.OutputAccumulatedStatistics(output);
        }

        public bool debug = false;
        private bool equivalenceWasOn;

        /// <summary>
        /// 
        /// </summary>
        /// <returns>Whether this is the top-most CBS</returns>
        protected bool SetGlobals()
        {
            this.equivalenceWasOn = AgentState.EquivalenceOverDifferentTimes == true;
            AgentState.EquivalenceOverDifferentTimes = false;

            if (this.instance.parameters.ContainsKey(CBS.CAT) == false) // Top-most CBS solver
            {
                this.instance.parameters[CBS.CAT] = new Dictionary_U<TimedMove, int>(); // Dictionary_U values are actually lists of ints.
                this.instance.parameters[CBS.CONSTRAINTS] = new HashSet_U<CbsConstraint>();
                if (this.doMalte && this.instance.parameters.ContainsKey(CBS.MUST_CONSTRAINTS) == false)
                    this.instance.parameters[CBS.MUST_CONSTRAINTS] = new HashSet_U<CbsConstraint>();
                return true;
            }
            else
                return false;
        }

        protected void CleanGlobals()
        {
            if (this.equivalenceWasOn)
                AgentState.EquivalenceOverDifferentTimes = true;
            if (this.topMost) // Clear problem parameters. Done for clarity only, since the Union structures are expected to be empty at this point.
            {
                this.instance.parameters.Remove(CBS.CAT);
                this.instance.parameters.Remove(CBS.CONSTRAINTS);
                // Don't remove must constraints:
                // A) It usually wasn't CBS that added them (they're only used for Malte's variant).
                // B) Must constraints only appear in temporary problems. There's no danger of leaking them to other solvers.
                // C) We don't have the information to re-create them later.
            }
        }

        public bool Solve()
        {
            //this.SetGlobals(); // Again, because we might be resuming a search that was stopped.

            int initialEstimate = 0;
            if (openList.Count > 0)
                initialEstimate = openList.Peek().f;

            int maxExpandedNodeF = -1;
            int currentCost = -1;

            while (openList.Count > 0)
            {
                // Check if max time has been exceeded
                if (this.runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    this.solutionCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.solutionDepth = openList.Peek().f - initialEstimate; // A minimum estimate
                    this.Clear(); // Total search time exceeded - we're not going to resume this search.
                    this.CleanGlobals();
                    return false;
                }

                CbsNode currentNode = openList.Remove();

                if (currentNode.f > this.maxSolutionCost)  // A late heuristic application may have increased the node's cost
                {
                    continue;
                    // Don't expand the node.
                    // This will exhaust the open list, assuming Fs of nodes chosen for expansions
                    // are monotonically increasing.
                    // TODO: Pass maxSolutionCost to the DynamicLazyOpenList so it can inform the 
                    //       heuristic not to try to improve its estimate to make the node's f more
                    //       than 1 + maxSolutionCost
                }


                currentNode.ChooseConflict();

                // Notice that even though we may discover cardinal conflicts in hindsight later,
                // there would be no point in pushing their node back at that point,
                // as we would've already made the split by then.

                this.addToGlobalConflictCount(currentNode.GetConflict()); // TODO: Make CBS_GlobalConflicts use nodes that do this automatically after choosing a conflict

                currentNode.DebugPrint();

                // Shuffle stage:
                if (this.doShuffle)
                {
                    this.Shuffle(currentNode);
                }

                // Update nodesExpandedWithGoalCost statistic
                if (currentNode.g > currentCost) // Needs to be here because the goal may have a cost unseen before
                {
                    currentCost = currentNode.g;
                    this.nodesExpandedWithGoalCost = 0;
                }
                else if (currentNode.g == currentCost) // check needed because macbs node cost isn't exactly monotonous
                {
                    this.nodesExpandedWithGoalCost++;
                }

                // Check if node is the goal
                if (currentNode.GoalTest())
                {
                    Debug.Assert(currentNode.g >= maxExpandedNodeF,
                                 $"CBS goal node found with lower cost than the max cost node ever expanded: {currentNode.g} < {maxExpandedNodeF}");
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
                    this.solutionCost = currentNode.g;
                    this.solutionDepth = this.solutionCost - initialEstimate;
                    this.goalNode = currentNode; // Saves the single agent plans and costs
                    // The joint plan is calculated on demand.
                    this.Clear(); // Goal found - we're not going to resume this search
                    this.CleanGlobals();
                    return true;
                }

                if (maxExpandedNodeF < currentNode.f)
                {
                    maxExpandedNodeF = currentNode.f;
                    Debug.Print("New max F: {0}", maxExpandedNodeF);
                }

                // Check conditions that stop the search before a goal is found
                if (currentNode.f >= this.targetF || // Node is good enough
                    this.singleAgentSolver.GetAccumulatedGenerated() + this.solver.GetAccumulatedGenerated() > 
                        this.lowLevelGeneratedCap || // Stop because this is taking too long.
                                                     // We're looking at _generated_ low level nodes since that's an indication to the amount of work done,
                                                     // while expanded nodes is an indication of the amount of good work done.
                    (this.milliCap != int.MaxValue && // (This check is much cheaper than the method call)
                     this.runner.ElapsedMilliseconds() > this.milliCap)) // Search is taking too long.
                {
                    Debug.WriteLine("-----------------");
                    this.solutionCost = maxExpandedNodeF; // This is the min possible cost so far.
                    this.openList.Add(currentNode); // To be able to continue the search later
                    this.CleanGlobals();
                    return false;
                }

                // Expand
                bool wasUnexpandedNode = (currentNode.agentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED &&
                                         currentNode.agentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED);
                Expand(currentNode);
                if (wasUnexpandedNode)
                    highLevelExpanded++;
                // Consider moving the following into Expand()
                if (currentNode.agentAExpansion == CbsNode.ExpansionState.EXPANDED &&
                    currentNode.agentBExpansion == CbsNode.ExpansionState.EXPANDED) // Fully expanded
                    currentNode.Clear();
            }

            this.solutionCost = Constants.NO_SOLUTION_COST;
            this.Clear(); // unsolvable problem - we're not going to resume it
            this.CleanGlobals();
            return false;
        }

        protected virtual bool ShouldMerge(CbsNode node)
        {
            return node.ShouldMerge(mergeThreshold);
        }

        /// <summary>
        /// TODO: Count the shuffles.
        /// TODO: Consider choosing a different conflict to work on if that current one couldn't be solved with a shuffle.
        ///       It isn't so easy, because just choosing another conflict won't work - a shuffle that changed nothing would revert to the old conflict automatically.
        /// </summary>
        /// <param name="currentNode"></param>
        protected void Shuffle(CbsNode currentNode)
        {
            if (currentNode.agentAExpansion != CbsNode.ExpansionState.NOT_EXPANDED || currentNode.agentBExpansion != CbsNode.ExpansionState.NOT_EXPANDED)
                return; // Partially expanded node was already shuffled before

            bool conflictChanged = false;
            int lastReshuffledAgent = -1; // To quiet the compiler
            int beforeLastReshuffledAgent = -1; // To quiet the compiler

            while (currentNode.GoalTest() == false)
            {
                CbsConflict conflict;
                int groupRepresentative;
                int internalConflictCountBefore;
                int externalConflictCountBefore;
                bool success;
                int internalConflictCountAfter;
                int externalConflictCountAfter;
                bool agentAPlanChanged = false;
                CbsConflict lastConflict = currentNode.GetConflict();

                conflict = currentNode.GetConflict();
                if (
                    ( // This is the root, and we're just starting to work on it
                     conflictChanged == false &&
                     currentNode.prev == null
                    ) ||
                    ( // Check that the first agent in the conflict wasn't just replanned or merged - shuffling in that case would necessarily have the same result since the low level is deterministic
                     conflictChanged == false &&
                     currentNode.prev != null &&
                     (
                      (currentNode.constraint == null && // This node is the result of a merge (since it isn't the root)
                       currentNode.prev.GetGroup(currentNode.prev.GetConflict().agentAIndex).Contains(conflict.agentAIndex) == false &&
                       currentNode.prev.GetGroup(currentNode.prev.GetConflict().agentBIndex).Contains(conflict.agentAIndex) == false // Agent A in this node's conflict wasn't just merged
                      ) || 
                      (currentNode.constraint != null && // This node is the result of a replan
                       conflict.agentAIndex != currentNode.agentNumToIndex[currentNode.constraint.agentNum] // Agent A wasn't just replanned
                      )
                     )
                    ) ||
                    ( // Check that the first agent in the conflict wasn't just reshuffled
                     conflictChanged == true &&
                     lastReshuffledAgent != conflict.agentAIndex
                    )
                   )
                {
                    // Are infinite improvement loops possible (a1 improves by screwing a2 and vice versa)? I don't think so, because we're looking for strictly better solutions.
                    Debug.WriteLine("");
                    Debug.WriteLine("Shuffling the first agent...");

                    groupRepresentative = currentNode.agentsGroupAssignment[conflict.agentAIndex];
                    internalConflictCountBefore = currentNode.countsOfInternalAgentsThatConflict[groupRepresentative];
                    externalConflictCountBefore = currentNode.totalExternalAgentsThatConflict;
                    SinglePlan agentAPlanBefore = currentNode.allSingleAgentPlans[conflict.agentAIndex];
                    success = currentNode.Replan(conflict.agentAIndex, this.minSolutionTimeStep);
                    // TODO: Pass minPathCost correctly, also cap the path cost to the same value
                    if (success == false) // Usually means a timeout occured
                        break;
                    beforeLastReshuffledAgent = lastReshuffledAgent;
                    lastReshuffledAgent = conflict.agentAIndex;
                    internalConflictCountAfter = currentNode.countsOfInternalAgentsThatConflict[groupRepresentative];
                    externalConflictCountAfter = currentNode.totalExternalAgentsThatConflict;

                    if ((internalConflictCountAfter > internalConflictCountBefore) ||
                        (internalConflictCountAfter == internalConflictCountBefore) && (externalConflictCountAfter >= externalConflictCountBefore))
                    {
                        Debug.WriteLine($"Shuffling the first agent didn't help: " +
                                        $"internal before={internalConflictCountBefore} " +
                                        $"internal after={internalConflictCountAfter} " +
                                        $"external before={externalConflictCountBefore} " +
                                        $"external after={externalConflictCountAfter}");
                        // Notice the shuffle can't increase the number of conflicts as the low level tries to minimize it,
                        // and the current configuration of paths is possible.
                    }
                    else
                    {
                        Debug.WriteLine($"Shuffling the left agent helped! " +
                                        $"internal before={internalConflictCountBefore} " +
                                        $"internal after={internalConflictCountAfter} " +
                                        $"external before={externalConflictCountBefore} " +
                                        $"external after={externalConflictCountAfter}");
                        currentNode.DebugPrint();
                    }
                    if (currentNode.GetConflict() == null ||
                        currentNode.GetConflict().Equals(lastConflict) == false) // Shuffling can help even without changing the conflict, by resolving unselected conflicts
                    {
                        Debug.WriteLine("Conflict changed - restarting shuffle");
                        //currentNode.Print();
                        conflictChanged = true;
                        continue;
                    }

                    agentAPlanChanged = agentAPlanBefore == currentNode.allSingleAgentPlans[conflict.agentAIndex]; // Can also happen if there was no improvement
                }
                else
                {
                    Debug.WriteLine("Skipping shuffling of first agent - it was just replanned, merged or reshuffled");
                }



                if (
                    agentAPlanChanged == true || // Then even if agentB was replanned/merged/shuffled recently, agent B is going to see a new CAT
                    ( // This is the root, and we're just starting to work on it
                     conflictChanged == false &&
                     currentNode.prev == null
                    ) ||
                    ( // Check that the first agent in the conflict wasn't just replanned or merged - shuffling in that case would necessarily have the same result since the low level is deterministic
                     conflictChanged == false &&
                     currentNode.prev != null &&
                     (
                      (currentNode.constraint == null && // This node is the result of a merge (since it isn't the root)
                       currentNode.prev.GetGroup(currentNode.prev.GetConflict().agentAIndex).Contains(conflict.agentBIndex) == false &&
                       currentNode.prev.GetGroup(currentNode.prev.GetConflict().agentBIndex).Contains(conflict.agentBIndex) == false // Agent A in this node's conflict wasn't just merged
                      ) ||
                      (currentNode.constraint != null && // This node is the result of a replan
                       conflict.agentBIndex != currentNode.agentNumToIndex[currentNode.constraint.agentNum] // Agent B wasn't just replanned
                      )
                     )
                    ) ||
                    ( // Check that the first agent in the conflict wasn't just reshuffled
                     conflictChanged == true &&
                     beforeLastReshuffledAgent != conflict.agentBIndex
                    )
                   )
                {
                    Debug.WriteLine("");
                    Debug.WriteLine("Shuffling the second agent...");

                    conflict = currentNode.GetConflict();
                    groupRepresentative = currentNode.agentsGroupAssignment[conflict.agentBIndex];
                    internalConflictCountBefore = currentNode.countsOfInternalAgentsThatConflict[groupRepresentative];
                    externalConflictCountBefore = currentNode.totalExternalAgentsThatConflict;
                    success = currentNode.Replan(conflict.agentBIndex, this.minSolutionTimeStep);
                    // TODO: Pass minPathCost correctly, also cap the path cost to the same value
                    if (success == false) // Usually means a timeout occured
                        break;
                    beforeLastReshuffledAgent = lastReshuffledAgent;
                    lastReshuffledAgent = conflict.agentBIndex;
                    internalConflictCountAfter = currentNode.countsOfInternalAgentsThatConflict[groupRepresentative];
                    externalConflictCountAfter = currentNode.totalExternalAgentsThatConflict;

                    if ((internalConflictCountAfter > internalConflictCountBefore) ||
                        (internalConflictCountAfter == internalConflictCountBefore) && (externalConflictCountAfter >= externalConflictCountBefore))
                    {
                        Debug.WriteLine($"Shuffling the second agent didn't help: " +
                                        $"internal before={internalConflictCountBefore} " +
                                        $"internal after={internalConflictCountAfter} " +
                                        $"external before={externalConflictCountBefore} " +
                                        $"external after={externalConflictCountAfter}");
                    }
                    else
                    {
                        Debug.WriteLine($"Shuffling the second agent helped! " +
                                        $"internal before={internalConflictCountBefore} " +
                                        $"internal after={internalConflictCountAfter} " +
                                        $"external before={externalConflictCountBefore} " +
                                        $"external after={externalConflictCountAfter}");
                        currentNode.DebugPrint();
                    }

                }
                else
                {
                    Debug.WriteLine("Skipping shuffling of second agent - it was just replanned, " +
                                    "merged or reshuffled and the first agent's shuffle didn't " +
                                    "change the first agent's plan"); // So the second agent will see the exact same CAT
                }

                if (currentNode.GetConflict() == null ||
                    currentNode.GetConflict().Equals(lastConflict) == false)
                {
                    Debug.WriteLine("Conflict changed - restarting shuffle");
                    //currentNode.Print();
                    conflictChanged = true;
                    continue;
                }
                else
                {
                    Debug.WriteLine("Still the exact same conflict - stop shuffling");
                    break;
                }
            }
        }

        public virtual void Reset()
        {
            this.restarts++;
            this.openList.Clear();
            this.closedList.Clear();
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
        protected (bool adopted, IList<CbsNode> children, bool reinsertParent) ExpandImpl(
            CbsNode node, bool adopt, CbsNode adoptBy = null)
        {
            CbsConflict conflict = node.GetConflict();
            var children = new List<CbsNode>();
            CbsNode child;
            int closedListHitChildCost;
            if (adoptBy == null)
                adoptBy = node;
            int adoptByH = adoptBy.h;

            bool leftSameCost = false; // To quiet the compiler
            bool rightSameCost = false;

            if (this.mergeThreshold != -1 && ShouldMerge(node))
            {
                if (this.mergeCausesRestart == false)
                {
                    (child, closedListHitChildCost) = this.MergeExpand(node);
                    if (child == null)
                        return (adopted: false, children, reinsertParent: false); // A timeout occured,
                                                                                  // or the child was already in the closed list,
                                                                                  // or there were just too many constraints
                                                                                  // (happens with ID, which adds whole paths as constraints)
                }
                else
                {
                    // TODO: What if planning a path for the merged agents finds a path with the same
                    // cost as the sum of their current paths and no other conflicts exist
                    child = new CbsNode(this.instance.agents.Length, this.solver,
                                        this.singleAgentSolver, this, node.agentsGroupAssignment);  // This will be the new root node
                    child.MergeGroups(node.agentsGroupAssignment[conflict.agentAIndex],
                        node.agentsGroupAssignment[conflict.agentBIndex], fixCounts: false);
                    this.maxSizeGroup = Math.Max(this.maxSizeGroup, child.GetGroupSize(conflict.agentAIndex));
                    bool solved = child.Solve(this.minSolutionTimeStep);
                    if (solved == false)
                        return (adopted: false, children, reinsertParent: false);
                    //if ((child.allSingleAgentCosts[child.agentsGroupAssignment[conflict.agentAIndex]] ==
                    //    node.allSingleAgentCosts[node.agentsGroupAssignment[conflict.agentAIndex]] +
                    //    node.allSingleAgentCosts[node.agentsGroupAssignment[conflict.agentBIndex]]) &&
                    //    //All of node's conflicts are between the agents of the merged groups)
                    if (this.debug)
                        Debug.WriteLine("Restarting the search with merged agents.");
                    this.Reset();
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
                    leftSameCost = child.g == node.g;
                    if (leftSameCost)
                    {
                        Debug.Assert(node.conflict.willCostIncreaseForAgentA != CbsConflict.WillCostIncrease.YES);
                    }
                    else
                    {
                        Debug.Assert(node.conflict.willCostIncreaseForAgentA != CbsConflict.WillCostIncrease.NO);
                    }
                }
            }
            else  // A timeout occured, or the child was already in the closed list.
            {
                if (closedListHitChildCost != -1)
                    leftSameCost = closedListHitChildCost == node.g;
            }

            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
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
                    rightSameCost = child.g == node.g;
                    if (rightSameCost)
                    {
                        Debug.Assert(node.conflict.willCostIncreaseForAgentB != CbsConflict.WillCostIncrease.YES);
                    }
                    else
                    {
                        Debug.Assert(node.conflict.willCostIncreaseForAgentB != CbsConflict.WillCostIncrease.NO);
                    }
                }
            }
            else  // A timeout occured, or the child was already in the closed list.
            {
                if (closedListHitChildCost != -1)
                    rightSameCost = closedListHitChildCost == node.g;
            }

            if (node.agentAExpansion == CbsNode.ExpansionState.DEFERRED || node.agentBExpansion == CbsNode.ExpansionState.DEFERRED)
                this.semiCardinalConflictSplits++; // Count only on the first expansion of the node. On the second expansion no count will be incremented.
            else
            {
                if (leftSameCost && rightSameCost)
                    this.nonCardinalConflictSplits++;
                else if (leftSameCost || rightSameCost)
                    this.semiCardinalConflictSplits++;
                else
                    this.cardinalConflictSplits++;
            }

            return (adopted: false, children, reinsertParent);
        }
        
        private void ExpandIgnoringCardinalsButSupportingBP2(CbsNode node)
        {
            int parentCost = node.g;
            int parentH = node.h;
            IList<CbsNode> children = new List<CbsNode>(2);
            bool reinsertParent = false;
            bool adopted = false;
             
            int origCardinalConflictSplits = this.cardinalConflictSplits;
            int origSemiCardinalConflictSplits = this.semiCardinalConflictSplits;
            int origNonCardinalConflictSplits = this.nonCardinalConflictSplits;

            if (this.bypassStrategy == BypassStrategy.NONE)
            {
                (adopted, children, reinsertParent) = this.ExpandImpl(node, adopt: false, adoptBy: null);
            }
            else if (this.bypassStrategy == BypassStrategy.FIRST_FIT_LOOKAHEAD || node.parentAlreadyLookedAheadOf) // Do bypass but don't look ahead
            {
                bool adoptionPerformedBefore = false;
                while (true) // Until a node with a higher cost is found or a goal is found
                {
                    var lookAheadOpenList = new OpenList<CbsNode>(this);
                    var lookAheadSameCostNodes = new HashSet<CbsNode>();
                    var lookAheadLargerCostNodes = new HashSet<CbsNode>();
                    var lookAheadSameCostNodesToReinsertWithHigherCost = new HashSet<CbsNode>();
                    lookAheadOpenList.Add(node);

                    if (lookAheadOpenList.Count != 0)
                        Debug.Print("Starting lookahead:");
                    IList<CbsNode> lookAheadChildren;
                    bool lookAheadReinsertParent;
                    int expansions = 0;
                    while (lookAheadOpenList.Count != 0)
                    {
                        if (expansions + 1 > this.lookaheadMaxExpansions) // + 1 because we're checking before the coming expansion. lookaheadMaxExpansions = 1 is the minimum working value.
                        {
                            Debug.WriteLine("Lookahead count exceeded. Stopping lookahead.");
                            break;
                        }

                        if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                            return;

                        CbsNode lookAheadNode = lookAheadOpenList.Remove();
                        lookAheadNode.ChooseConflict();

                        Debug.WriteLine($"Looking ahead from node hash: {lookAheadNode.GetHashCode()}.");

                        (adopted, lookAheadChildren, lookAheadReinsertParent) = this.ExpandImpl(lookAheadNode, adopt: true, adoptBy: node);
                        expansions++;

                        if (adopted == true)
                        {
                            this.bypassLookAheadNodesCreated += lookAheadChildren.Count + 1; // Either 1, if the first child was adopted, or 2 otherwise. +1 for the node itself before it adopted another solution.
                            this.cardinalConflictSplits = origCardinalConflictSplits;  // No split was done
                            this.semiCardinalConflictSplits = origSemiCardinalConflictSplits;
                            this.nonCardinalConflictSplits = origNonCardinalConflictSplits;
                            break; // Insert children into open list
                        }

                        if (lookAheadReinsertParent)
                            lookAheadSameCostNodesToReinsertWithHigherCost.Add(lookAheadNode);

                        foreach (var child in lookAheadChildren)
                        {
                            this.bypassLookAheadNodesCreated++;
                            this.closedList.Add(child, child); // Temporarily! Just so look ahead expansion can use this data
                            if (child.g == node.g)
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
                        this.bypasses++; // Only count one adoption for the entire look ahead subtree branch traversed.
                    }

                    if (node.GoalTest()) // Lookahead found an admissable solution! (The original node to expand wasn't a goal)
                    {
                        Debug.WriteLine("Goal found with same cost - stopping lookahead.");
                        this.openList.Add(node);
                        return;
                    }

                    if (adopted == false) // Insert children into open list
                    {
                        if (node.h < 1 && lookAheadOpenList.Count == 0) // Looked ahead exhaustively
                            node.h = 1; // We know the goal isn't under it with the same cost

                        // We already expanded this whole subtree of same cost nodes (at least partially). Insert the frontier to the open list.
                        reinsertParent = false; // It may be reinserted as an element of the lookAheadSameCostNodesToReinsertWithHigherCost
                        children = lookAheadLargerCostNodes.ToList(); // Larger cost nodes aren't expanded so they're always on the frontier
                        int unexpandedSameCostNodes = lookAheadOpenList.Count;
                        while (lookAheadOpenList.Count != 0)
                        {
                            CbsNode child = lookAheadOpenList.Remove();
                            children.Add(child); // Unexapnded so it's also on the frontier
                            this.closedList.Remove(child); // Just so it'll be inserted into the open list at the end of the method
                        }
                        foreach (var reInsertNode in lookAheadSameCostNodesToReinsertWithHigherCost)
                        {
                            if (reInsertNode == node)
                            {
                                reinsertParent = true;
                                continue;
                            }
                            children.Add(reInsertNode);
                            this.closedList.Remove(reInsertNode); // Just so it'll be inserted into the open list at the end of the method
                        }
                        foreach (CbsNode lookAheadNode in lookAheadLargerCostNodes)
                            this.closedList.Remove(lookAheadNode); // Just so they'll be inserted into the open list at the end of the method
                        this.bypassLookAheadNodesCreated -= lookAheadSameCostNodes.Count + lookAheadLargerCostNodes.Count; // None of these nodes weren't wasted effort, they were properly generated
                        this.highLevelGenerated += lookAheadSameCostNodes.Count
                                                   - lookAheadSameCostNodesToReinsertWithHigherCost.Count
                                                   - unexpandedSameCostNodes; // You could say they were generated and not looked ahead at, and they won't be counted later.
                        this.highLevelExpanded += lookAheadSameCostNodes.Count - unexpandedSameCostNodes;

                        break;
                    }
                    else // Adopted. Do a new round of expansions.
                    {
                        foreach (CbsNode lookAheadNode in lookAheadLargerCostNodes)
                            this.closedList.Remove(lookAheadNode); // Just so they'll be inserted into the open list at the end of the method
                        foreach (CbsNode lookAheadNode in lookAheadSameCostNodes)
                            this.closedList.Remove(lookAheadNode); // Just so they'll be inserted into the open list at the end of the method
                                                                   // The only difference in closed list cleanup after adoption is that expanded same cost nodes are also removed from the closed list.
                    }
                }
            }
            else // this.bypassStrategy == BypassStrategy.BEST_FIT_LOOKAHEAD and this set of costs not already done
            {
                // FIXME: lookaheadMaxExpansions isn't respected correctly here. We actually limit the number of same-cost nodes generated, which is similar but not the same.
                var lookAheadOpenList = new OpenList<CbsNode>(this);
                var lookAheadSameCostNodes = new HashSet<CbsNode>();
                var lookAheadLargerCostNodes = new HashSet<CbsNode>();
                var lookAheadSameCostNodesToReinsertWithHigherCost = new HashSet<CbsNode>();
                lookAheadOpenList.Add(node);
                node.parentAlreadyLookedAheadOf = true;

                if (lookAheadOpenList.Count != 0)
                    Debug.Print("Starting lookahead:");
                IList<CbsNode> lookAheadChildren;
                bool lookAheadReinsertParent;
                while (lookAheadOpenList.Count != 0)
                {
                    if (lookAheadSameCostNodes.Count + 1 >= this.lookaheadMaxExpansions) // + 1 for the root
                    {
                        Debug.WriteLine("Lookahead count exceeded. Stopping lookahead.");
                        break;
                    }

                    CbsNode lookAheadNode = lookAheadOpenList.Remove();
                    lookAheadNode.ChooseConflict();

                    if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                        return;

                    Debug.WriteLine($"Looking ahead from node hash: {lookAheadNode.GetHashCode()}.");

                    (adopted, lookAheadChildren, lookAheadReinsertParent) = this.ExpandImpl(lookAheadNode, adopt: false);

                    if (lookAheadReinsertParent)
                        lookAheadSameCostNodesToReinsertWithHigherCost.Add(lookAheadNode);

                    foreach (var child in lookAheadChildren)
                    {
                        this.bypassLookAheadNodesCreated++;
                        this.closedList.Add(child, child); // Temporarily! Just so look ahead expansion can use this data
                        if (child.g == node.g)
                        {
                            if (child.GoalTest()) // Lookahead found an admissable solution!
                            {
                                Debug.WriteLine("Goal found with same cost - stopping lookahead.");
                                this.openList.Add(child); // Technically should have just breaked and let node adopt child. This is just a short-cut.
                                this.bypasses++; // Just a technicality needed to make the adoption count not lower than if we didn't use immediate adoption. You could say we adopt the goal's solution.
                                return;
                            }

                            if (lookAheadSameCostNodes.Contains(child) == false)
                            {
                                lookAheadOpenList.Add(child);
                                lookAheadSameCostNodes.Add(child);
                            }
                        }
                        else
                            lookAheadLargerCostNodes.Add(child); // No need to check if it's already there :)
                    }
                }

                if (node.h < 1 && lookAheadOpenList.Count == 0) // Looked ahead exhaustively
                    node.h = 1; // We know the goal isn't under it with the same cost

                bool bypassPerformed = false;
                // Find the best look ahead node and check if it's worth adopting
                if (lookAheadSameCostNodes.Count != 0)
                {
                    IList<CbsNode> lookAheadSameCostNodesSerialzed = lookAheadSameCostNodes.ToList<CbsNode>();
                    CbsNode adoptionCandidate = lookAheadSameCostNodesSerialzed[0];
                    for (int i = 1; i < lookAheadSameCostNodesSerialzed.Count; i++)
                    {
                        if (lookAheadSameCostNodesSerialzed[i].CompareToIgnoreH(adoptionCandidate) == -1)
                            adoptionCandidate = lookAheadSameCostNodesSerialzed[i];
                    }
                    if (AdoptConditionally(node, adoptionCandidate, parentH))
                    {
                        bypassPerformed = true;
                        this.bypasses++;

                        this.cardinalConflictSplits = origCardinalConflictSplits;
                        this.semiCardinalConflictSplits = origSemiCardinalConflictSplits;
                        this.nonCardinalConflictSplits = origNonCardinalConflictSplits;

                        // Remove cancelled look-ahead nodes from closed list
                        foreach (CbsNode lookAheadNode in lookAheadSameCostNodes)
                            this.closedList.Remove(lookAheadNode);
                        foreach (CbsNode lookAheadNode in lookAheadLargerCostNodes)
                            this.closedList.Remove(lookAheadNode);

                        if (this.openList.Count != 0 && this.openList.Peek().f < node.f)
                        // This is a new unexpanded node, and we just raised its h, so we can push it back
                        // FIXME: More duplication of the push back logic
                        {
                            reinsertParent = true;
                            children = new List<CbsNode>(); // Children will be generated when this node comes out of the open list

                            if (this.debug)
                                Debug.Print("Reinserting node into the open list with h=1, since the goal wasn't found with the same cost under it.");
                        }
                        else
                        {
                            // We updated the node, need to re-expand it. Surprisingly, re-expansion can produce even better nodes for adoption, so we need to allow immediate expansion too.
                            // TODO: Just re-run the infinite lookahead? That would be the immediate adoption variant above
                            this.Expand(node);
                            return;
                        }
                    }
                }

                if (bypassPerformed == false)
                {
                    // Adoption not performed, and we already expanded this whole subtree of same cost nodes (at least partially)
                    reinsertParent = false; // It may reinserted as an element of the children list
                    children = lookAheadLargerCostNodes.ToList();
                    int unexpandedSameCostNodes = lookAheadOpenList.Count;
                    while (lookAheadOpenList.Count != 0)
                    {
                        CbsNode child = lookAheadOpenList.Remove();
                        children.Add(child);
                        this.closedList.Remove(child); // Just so it'll be inserted into the open list at the end of the method
                    }
                    foreach (var reInsertNode in lookAheadSameCostNodesToReinsertWithHigherCost)
                    {
                        if (reInsertNode == node)
                        {
                            reinsertParent = true;
                            continue;
                        }
                        children.Add(reInsertNode);
                        this.closedList.Remove(reInsertNode); // Just so it'll be inserted into the open list at the end of the method
                        if (reInsertNode == node) // Re-inserting the original node that was to be expanded, with a higher h
                            this.highLevelGenerated--; // It'll be counted as a new generated child later, and we don't need to count it twice
                    }
                    foreach (CbsNode lookAheadNode in lookAheadLargerCostNodes)
                        this.closedList.Remove(lookAheadNode); // Just so they'll be inserted into the open list at the end of the method
                    this.bypassLookAheadNodesCreated -= lookAheadSameCostNodes.Count + lookAheadLargerCostNodes.Count; // These nodes weren't wasted effort
                    this.highLevelGenerated += lookAheadSameCostNodes.Count - lookAheadSameCostNodesToReinsertWithHigherCost.Count - unexpandedSameCostNodes; // You could say they were generated and not looked ahead at, and they won't be counted later.
                    this.highLevelExpanded += lookAheadSameCostNodes.Count - unexpandedSameCostNodes;
                }
            }

            // Both children considered. None adopted. Add them to the open list, and re-insert the partially expanded parent too if necessary.
            if (reinsertParent)
                this.openList.Add(node); // Re-insert node into open list with higher cost, don't re-increment global conflict counts

            foreach (var child in children)
            {
                closedList.Add(child, child);

                // Bequeath remainder of h from parent
                int remainingParentH = parentH - (child.g - parentCost);
                if (child.h < remainingParentH)
                    child.h = (ushort) remainingParentH;

                if (this.bypassStrategy == BypassStrategy.BEST_FIT_LOOKAHEAD)
                {
                    if (child.g == parentCost) // Total cost didn't increase (yet)
                        child.parentAlreadyLookedAheadOf = true;
                    else
                        child.parentAlreadyLookedAheadOf = false;
                }

                if (child.f <= this.maxSolutionCost)
                // Assuming h is an admissable heuristic, no need to generate nodes that won't get us
                // to the goal within the budget
                {
                    this.highLevelGenerated++;
                    openList.Add(child);
                }
                else
                {
                    this.surplusNodesAvoided++;
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
            int parentCost = node.g;
            int parentH = node.h;
            IList<CbsNode> children = new List<CbsNode>(2);
            bool reinsertParent;
            bool adopted = false;

            int origCardinalConflictSplits = this.cardinalConflictSplits;
            int origSemiCardinalConflictSplits = this.semiCardinalConflictSplits;
            int origNonCardinalConflictSplits = this.nonCardinalConflictSplits;

            Debug.Assert(this.bypassStrategy == BypassStrategy.NONE || this.lookaheadMaxExpansions == 1, "Only BP1 is supported with cardinal conflict choice");  // Assumed in order to simplify code
            Debug.Assert(this.bypassStrategy != BypassStrategy.BEST_FIT_LOOKAHEAD, "Only first-fit adoption is supported with cardinal conflict choice"); // Assumed in order to simplify code

            bool adoptionPerformedBefore = false;

            // Adoption and cardinal-lookahead cycle
            while (true)
            {
                (adopted, children, reinsertParent) = this.ExpandImpl(node, adopt: bypassStrategy != BypassStrategy.NONE);

                if (this.mergeCausesRestart && (this.closedList.Count == 0)) // HACK: Means a restart was triggered
                    break;

                if (adopted) // Either we found a goal, or we need to re-expand using a remaining conflict
                {
                    if (adoptionPerformedBefore == false)
                    {
                        adoptionPerformedBefore = true;
                        this.bypasses++; // Only count one adoption for the entire look ahead subtree branch traversed.
                    }
                    this.bypassLookAheadNodesCreated += children.Count + 1; // Either 1, if the first child was adopted, or 2 otherwise, +1 because the adopted child isn't returned.
                    this.cardinalConflictSplits = origCardinalConflictSplits;
                    this.semiCardinalConflictSplits = origSemiCardinalConflictSplits;
                    this.nonCardinalConflictSplits = origNonCardinalConflictSplits;

                    if (node.GoalTest()) // Adoption found an admissable solution! (The original node to expand wasn't a goal)
                    {
                        Debug.WriteLine("Bypass found a goal! Inserting it into OPEN.");
                        this.openList.Add(node);
                        return;
                    }

                    Debug.Assert(node.conflict != null, "A conflict should have been found");
                    // We might re-encounter conflicts we've already found aren't cardinal, but
                    // since at least one path changed, the number of conflicts after adopting a child
                    // might become smaller after adopting a child's solution this time
                    continue;
                }

                if (
                    children.All(child => child.g > node.g) && // All generated nodes have a higher cost
                    ((node.agentAExpansion == CbsNode.ExpansionState.EXPANDED && node.agentBExpansion == CbsNode.ExpansionState.EXPANDED) || // Both children generated by now (possibly one in the past and one deferred to now)
                    (node.agentAExpansion == CbsNode.ExpansionState.EXPANDED && node.agentBExpansion == CbsNode.ExpansionState.DEFERRED) || // One child still deferred, will surely have a higher cost (that's why it was deferred)
                    (node.agentAExpansion == CbsNode.ExpansionState.DEFERRED && node.agentBExpansion == CbsNode.ExpansionState.EXPANDED)) // One child still deferred, will surely have a higher cost (that's why it was deferred)
                        ) // A cardinal conflict, or deferred expansion of a cardinal or semi-cardinal conflict.
                {
                    if (this.debug)
                        Debug.WriteLine("This was a cardinal conflict, or deferred expansion of a cardinal or semi-cardinal conflict. Inserting the generated children into OPEN.");

                    break; // Look no further
                }
                else
                {
                    // This wasn't a cardinal conflict, continue cycling through conflicts
                }

                if (children.Any(child => child.GoalTest() && child.g == node.g)) // Admissable goal found (and adoption isn't enabled, otherwise the goal would have been adopted above)
                {
                    if (this.debug)
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
                    Debug.WriteLine($"This was a non-cardinal conflict. Trying potentially-cardinal conflict: {node.conflict}");
                }

                // Prepare to restart the loop
                // Cancel partial expansion effects:
                node.agentAExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
                node.agentBExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
                node.h = parentH;
                // Take care of counts:
                this.cardinalLookAheadNodesCreated += children.Count;
            }

            if (reinsertParent)
            {
                this.openList.Add(node); // Re-insert node into open list with higher cost, don't re-increment global conflict counts
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
            int minChildSumHAndOperatorCost = int.MaxValue;
            foreach (var child in children)
            {
                if (minChildSumHAndOperatorCost > child.h + (child.g - node.g))
                {
                    minChildSumHAndOperatorCost = child.h + (child.g - node.g);
                }
            }
            if (node.h < minChildSumHAndOperatorCost)
            {
                node.hBonus += minChildSumHAndOperatorCost - node.h;
                node.h = minChildSumHAndOperatorCost;
                ++reversePathMaxBoosts;
            }

            // Forward Path-Max (Mero, 1984)
            foreach (var child in children)
            {
                closedList.Add(child, child);

                // Bequeath remainder of h from parent
                int remainingParentH = parentH - (child.g - parentCost);
                if (child.h < remainingParentH)
                {
                    child.h = (ushort)remainingParentH;
                    this.pathMaxBoosts++;
                }
                // "Path-Max Plus"
                if (node.minimumVertexCover > 0 &&
                    (this.conflictChoice == ConflictChoice.CARDINAL_MDD || this.conflictChoice == ConflictChoice.CARDINAL_LOOKAHEAD))
                {
                    child.h = (ushort)Math.Max(child.h, node.minimumVertexCover - 1);  // -1 because we've just resolved a cardinal conflict.
                                                                                       // Even if the cost increased by more than 1 for the child, this is still our estimate, based on all the other conflicts
                    this.pathMaxPlusBoosts++;
                }
                if (child.f <= this.maxSolutionCost)
                {
                    this.highLevelGenerated++;
                    openList.Add(child);
                }
            }
        }

        public virtual void Expand(CbsNode node)
        {
            if (this.bypassStrategy == BypassStrategy.BEST_FIT_LOOKAHEAD && node.parentAlreadyLookedAheadOf)
                Debug.Print("Not looking ahead from this node, one of its ancestors of the same cost was already looked ahead of");

            if (this.conflictChoice == ConflictChoice.FIRST || this.conflictChoice == ConflictChoice.MOST_CONFLICTING_SMALLEST_AGENTS) // Then just choose a conflict once and stick with it.
            {
                this.ExpandIgnoringCardinalsButSupportingBP2(node);
            }
            else // We try to split according to cardinal conflicts
            {
                Debug.Assert(bypassStrategy != BypassStrategy.BEST_FIT_LOOKAHEAD, "For simplicity, BP2 is not supported when choosing cardinal conflicts");
                this.IcbsExpand(node);
            }
        }

        /// <summary>
        /// </summary>
        /// <param name="node"></param>
        /// <returns>A ValueTuple with:
        /// child: null if planning the child's path failed or it was a closed list hit, otherwise - the new child
        /// closedListHitChildCost: Used to tell if conflict is cardinal, semi-cardinal or
        /// non-cardinal when the child is a closed list hit.
        /// </returns>
        protected (CbsNode child, int closedListHitChildCost) MergeExpand(CbsNode node)
        {
            CbsConflict conflict = node.GetConflict();
            int closedListHitChildCost = -1;
            
            CbsNode child = new CbsNode(node, node.agentsGroupAssignment[conflict.agentAIndex], node.agentsGroupAssignment[conflict.agentBIndex]);
            if (closedList.ContainsKey(child) == false) // We may have already merged these agents in another node
            {
                Debug.WriteLine("Merging agents {0} and {1}", conflict.agentAIndex, conflict.agentBIndex);
                int aCost = node.GetGroupCost(node.agentsGroupAssignment[conflict.agentAIndex]);
                int bCost = node.GetGroupCost(node.agentsGroupAssignment[conflict.agentBIndex]);
                int minNewCost;
                if (Constants.costFunction == Constants.CostFunction.SUM_OF_COSTS)
                    minNewCost = aCost + bCost;
                else if (Constants.costFunction == Constants.CostFunction.MAKESPAN ||
                         Constants.costFunction == Constants.CostFunction.MAKESPAN_THEN_SUM_OF_COSTS)
                    minNewCost = Math.Max(aCost, bCost);
                else
                    throw new NotImplementedException("Unsupported cost function");
                if (this.useOldCost == false)
                    minNewCost = -1;
                bool success = child.Replan(conflict.agentAIndex,  // or agentBIndex. Doesn't matter - they're in the same group.
                                            this.minSolutionTimeStep, minPathCost: minNewCost);

                if (success == false) // A timeout probably occured
                    return (child: null, closedListHitChildCost);

                Debug.WriteLine($"Child hash: {child.GetHashCode()}");
                Debug.WriteLine($"Child cost: {child.g}");
                Debug.WriteLine($"Child min ops to solve: {child.minOpsToSolve}");
                Debug.WriteLine("");

                this.maxSizeGroup = Math.Max(this.maxSizeGroup, child.replanSize);

                return (child, closedListHitChildCost);
            }
            else
            {
                closedListHits++;
                return (child: null, closedListHitChildCost: closedList[child].g);
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="node"></param>
        /// <param name="doLeftChild"></param>
        /// <returns>
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
            CbsNode.ExpansionState expansionsState = doLeftChild ? node.agentAExpansion : node.agentBExpansion;
            CbsNode.ExpansionState otherChildExpansionsState = doLeftChild ? node.agentBExpansion : node.agentAExpansion;
            string agentSide = doLeftChild? "left" : "right";
            int planSize = node.allSingleAgentPlans[conflictingAgentIndex].GetSize();
            int groupSize = node.GetGroupSize(conflictingAgentIndex);
            CbsConflict.WillCostIncrease willCostIncrease = doLeftChild ? node.conflict.willCostIncreaseForAgentA : node.conflict.willCostIncreaseForAgentB;

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
                 conflict.timeStep >= node.allSingleAgentCosts[conflictingAgentIndex] && // Can't just check whether the node is at its goal - 
                                                                                         // the plan may involve it passing through its goal and returning to it later because of preexisting constraints.
                                                                                         // This assumes unit move costs
                 node.h < conflict.timeStep + 1 - node.allSingleAgentCosts[conflictingAgentIndex] && // Otherwise we won't be increasing its h and there would be no reason to delay expansion
                                                                                                     // The agent's new cost will be at least conflict.timeStep + 1, so however much this is more than its current cost 
                                                                                                     // is an admissable heuristic
                 groupSize == 1) || // Otherwise an agent in the group can be forced to take a longer
                                    // route without increasing the group's cost because
                                    // another agent would be able to take a shorter route.
               (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE &&
                expansionsState == CbsNode.ExpansionState.NOT_EXPANDED && conflict.isVertexConflict == true &&
                ((conflict.timeStep > planSize - 1 && node.h < 2) ||
                 (conflict.timeStep == planSize - 1 && node.h < 1)) &&  // Otherwise we won't be increasing its h and there would be no reason to delay expansion
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
                    node.agentAExpansion = CbsNode.ExpansionState.DEFERRED;
                else
                    node.agentBExpansion = CbsNode.ExpansionState.DEFERRED;
                // Add the minimal delta in the child's cost:
                // since we're banning the goal at conflict.timeStep, it must at least do conflict.timeStep+1 steps
                if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.ORIG)
                {
                    node.h = Math.Max(node.h, (ushort)(conflict.timeStep + 1 - node.allSingleAgentCosts[conflictingAgentIndex]));
                    // Technically, we've already made sure above we're going to increase the node's h,
                    // This is just to make the line look correct without reading the complex if statement above.
                }
                else if (Constants.sumOfCostsVariant == Constants.SumOfCostsVariant.WAITING_AT_GOAL_ALWAYS_FREE)
                {
                    if (conflict.timeStep > planSize - 1) // Agent will need to step out and step in to the goal, at least
                        node.h = Math.Max(node.h, (ushort) 1);
                    else // Conflict is just when agent enters the goal, it'll have to at least wait one timestep.
                        node.h = Math.Max(node.h, (ushort) 1);
                    // Technically, we've already made sure above we're going to increase the node's h,
                    // This is just to make the line look correct without reading the complex if statement above.
                }
                this.partialExpansions++;  // The other child was surely generated, so we can count a partial expansion here.
                return (node, closedListHitChildCost: -1);
            }
            else if (expansionsState != CbsNode.ExpansionState.EXPANDED)
            // Agent expansion already skipped in the past or not forcing it from its goal - finally generate the child:
            {
                Debug.WriteLine($"Generating {agentSide} child");

                if (doLeftChild)
                    node.agentAExpansion = CbsNode.ExpansionState.EXPANDED;
                else
                    node.agentBExpansion = CbsNode.ExpansionState.EXPANDED;

                var newConstraint = new CbsConstraint(conflict, instance, doLeftChild);
                CbsNode child = new CbsNode(node, newConstraint, conflictingAgentIndex);

                if (this.doMalte && doLeftChild == false)
                {
                    // Add the case where both agents shouldn't be at the conflict point to the _left_ child
                    // by forcing the first agent in the _right_ child to _always_ be at the conflict point.
                    // Notice that vertex conflicts create must constraint with NO_DIRECTION and edge conflicts
                    // don't, as necessary.
                    var newMustConstraint = new CbsConstraint(conflict, instance, true);
                    child.SetMustConstraint(newMustConstraint);
                }

                if (closedList.ContainsKey(child) == false)
                {
                    int oldCost = node.GetGroupCost(node.agentsGroupAssignment[conflictingAgentIndex]);
                    int minNewCost = oldCost;
                    if (willCostIncrease == CbsConflict.WillCostIncrease.YES)
                        minNewCost = oldCost + 1;

                    int maxNewCost = int.MaxValue;
                    if (willCostIncrease == CbsConflict.WillCostIncrease.NO)
                        maxNewCost = oldCost;

                    if (this.useOldCost == false)
                    {
                        minNewCost = -1;
                        maxNewCost = int.MaxValue;
                    }

                    bool success = child.Replan(conflictingAgentIndex, this.minSolutionTimeStep,  // The node takes the max between minSolutionTimeStep and the max time over all constraints.
                                                minPathCost: minNewCost, maxPathCost: maxNewCost);

                    if (success == false)
                        return (null, closedListHitChildCost: -1); // A timeout probably occured

                    Debug.WriteLine($"Child hash: {child.GetHashCode()}");
                    Debug.WriteLine($"Child cost: {child.g}");
                    Debug.WriteLine($"Child min ops to solve: {child.minOpsToSolve}");
                    Debug.WriteLine($"Child num of agents that conflict: {child.totalInternalAgentsThatConflict}");
                    Debug.WriteLine($"Child num of internal conflicts: {child.totalConflictsBetweenInternalAgents}");
                    Debug.WriteLine("");

                    if (child.g < node.g && groupSize == 1) // Catch the error early
                    {
                        child.DebugPrint();
                        Debug.WriteLine("Child plan: (cost {0})", child.allSingleAgentCosts[conflictingAgentIndex]);
                        child.allSingleAgentPlans[conflictingAgentIndex].DebugPrint();
                        Debug.WriteLine("Parent plan: (cost {0})", node.allSingleAgentCosts[conflictingAgentIndex]);
                        node.allSingleAgentPlans[conflictingAgentIndex].DebugPrint();
                        Debug.Assert(false, $"Single agent node with lower cost than parent! {child.g} < {node.g}");
                    }

                    return (child, closedListHitChildCost: -1);
                }
                else
                {
                    this.closedListHits++;
                    Debug.WriteLine("Child already in closed list!");
                    // No need to check if we need to reopen - reopening is only necessary if F is lower,
                    // but for the same constraints and group assignment we're going to have the same
                    // g, and the h in the closed list can only be higher (if it was already expanded
                    // and the open list's expensive heuristic was computed for it).
                    return (child: null, closedListHitChildCost: this.closedList[child].g);
                }
            }
            else
            {
                Debug.WriteLine("Child already generated before");
                return (child: null, closedListHitChildCost: -1);
            }
        }

        protected bool AdoptConditionally(CbsNode node, CbsNode adoptionCandidate, int nodeOrigH)
        {
            Debug.WriteLine($"Considering adoption of node hash: {adoptionCandidate.GetHashCode()}.");

            if (adoptionCandidate.g == node.g && // No need to branch :)
                adoptionCandidate.CompareToIgnoreH(node, ignorePartialExpansion: true, ignoreDepth: true) == -1
               )
            {
                this.conflictsBypassed += node.totalConflictsWithExternalAgents + node.totalConflictsBetweenInternalAgents
                                                      - adoptionCandidate.totalConflictsWithExternalAgents - adoptionCandidate.totalConflictsBetweenInternalAgents;
                node.AdoptSolutionOf(adoptionCandidate);
                node.h = nodeOrigH; // Cancel partial expansion h boost

                Debug.WriteLine("Child has same cost as parent and a better solution - child solution adopted by parent! (other generated children and partial expansions cancelled)");
                Debug.WriteLine("Node new details:");
                node.DebugPrint();
                return true;
            }
            Debug.WriteLine("Did not adopt.");
            return false;
        }

        protected virtual void addToGlobalConflictCount(CbsConflict conflict) { }

        public virtual Plan GetPlan()
        {
            if (this.solution == null)
                this.solution = this.goalNode.CalculateJointPlan();
            return this.solution;
        }

        public int GetSolutionDepth() { return this.solutionDepth; }
        
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        
        public SinglePlan[] GetSinglePlans()
        {
            return goalNode.allSingleAgentPlans;
        }

        public virtual int[] GetSingleCosts()
        {
            return goalNode.allSingleAgentCosts;
        }

        public int GetHighLevelExpanded() { return highLevelExpanded; }
        public int GetHighLevelGenerated() { return highLevelGenerated; }
        public int GetLowLevelExpanded() { return this.solver.GetAccumulatedExpanded(); }
        public int GetLowLevelGenerated() { return this.solver.GetAccumulatedGenerated(); }
        public int GetExpanded() { return highLevelExpanded; }
        public int GetGenerated() { return highLevelGenerated; }
        public int GetAccumulatedExpanded() { return accHLExpanded; }
        public int GetAccumulatedGenerated() { return accHLGenerated; }
        public int GetMaxGroupSize() { return this.maxSizeGroup; }
    }

    /// <summary>
    /// Merges agents if they conflict more times than the given threshold in all the CT.
    /// </summary>
    public class CBS_GlobalConflicts : CBS
    {
        public int[][] globalConflictsCounter;

        public CBS_GlobalConflicts(ICbsSolver singleAgentSolver, ICbsSolver generalSolver,
                                   int mergeThreshold = -1,
                                   bool doShuffle = false,
                                   BypassStrategy bypassStrategy = BypassStrategy.NONE,
                                   bool doMalte = false,
                                   ConflictChoice conflictChoice = ConflictChoice.FIRST,
                                   ILazyHeuristic<CbsNode> heuristic = null,
                                   bool disableTieBreakingByMinOpsEstimate = false,
                                   int lookaheadMaxExpansions = 1,
                                   bool mergeCausesRestart = false)
            : base(singleAgentSolver, generalSolver, mergeThreshold, doShuffle,
                   bypassStrategy, doMalte, conflictChoice, heuristic,
                   disableTieBreakingByMinOpsEstimate, lookaheadMaxExpansions,
                   mergeCausesRestart)
        {
            //throw new NotImplementedException("Not supported until we decide how to count conflicts. Used to rely on the specific conflict chosen in each node.");
        }

        /// <summary>
        /// Assumes agent nums start from 0 and are consecutive.
        /// </summary>
        /// <param name="problemInstance"></param>
        /// <param name="runner"></param>
        public override void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.MakeConflictMatrix(problemInstance);
            base.Setup(problemInstance, runner);
        }

        private void MakeConflictMatrix(ProblemInstance problemInstance)
        {
            this.globalConflictsCounter = new int[problemInstance.agents.Length][];
            for (int i = 0; i < globalConflictsCounter.Length; i++)
            {
                this.globalConflictsCounter[i] = new int[i];
                for (int j = 0; j < i; j++)
                {
                    this.globalConflictsCounter[i][j] = 0;
                }
            }
        }

        protected override bool ShouldMerge(CbsNode node)
        {
            return node.ShouldMerge(mergeThreshold, globalConflictsCounter);
        }

        protected override void addToGlobalConflictCount(CbsConflict conflict)
        {
            if (conflict != null)
                globalConflictsCounter[Math.Max(conflict.agentAIndex, conflict.agentBIndex)][Math.Min(conflict.agentAIndex, conflict.agentBIndex)]++;
        }

        public override string GetName()
        {
            string baseName = base.GetName();
            return baseName.Replace("Local", "Global");
        }

        public override void Reset()
        {
            base.Reset();
            //this.MakeConflictMatrix(this.instance);
        }
    }
}
