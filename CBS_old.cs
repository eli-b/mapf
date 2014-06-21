using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    /// <summary>
    /// Like CBS, without partial expansion.
    /// </summary>
    class CBS_old : ICbsSolver
    {
          // The key of the constraints list used for each CBS node
        public static string CONSTRAINTS = "constraints";
        // The key of the internal CAT for CBS
        public static string INTERNAL_CAT = "internalCAT";
        // The key of recent constraints
        public static string NEW_CONSTRAINTS = "newConstraints";
        // The key of recent internal CAT for CBS
        public static string NEW_INTERNAL_CAT = "newInternalCAT";

        protected ProblemInstance instance;
        public OpenList openList;
        public Dictionary<CbsNode, CbsNode> closedList;
        protected int highLevelExpanded;
        protected int highLevelGenerated;
        protected int closedListHits;
        protected int accHLExpanded;
        protected int accHLGenerated;
        protected int accClosedListHits;
        public int totalCost;
        protected Run runner;
        protected CbsNode goalNode;
        protected Plan solution;
        protected int maxCost;
        protected ICbsSolver solver;
        protected ICbsSolver lowLevelSolver;
        protected int mergeThreshold;
        protected int minDepth;
        protected int maxThreshold;
        protected int maxSizeGroup;
        protected bool topMost;

        public CBS_old() { }

        public CBS_old(ICbsSolver solver, int maxThreshold = -1, int currentThreshold = -1)
        {
            this.closedList = new Dictionary<CbsNode, CbsNode>();
            this.openList = new OpenList(this);
            this.mergeThreshold = currentThreshold;
            this.solver = solver;
            this.lowLevelSolver = solver;
            this.maxThreshold = maxThreshold;
            if (currentThreshold < maxThreshold)
            {
                this.solver = new CBS_old(solver, maxThreshold, currentThreshold + 1);
            }
        }

        public virtual void Setup(ProblemInstance problemInstance, Run runner)
        {
            AgentState.EquivalenceOverDifferentTimes = false;
            this.Clear();

            this.instance = problemInstance;
            this.runner = runner;
            CbsNode root = new CbsNode(instance.m_vAgents.Length, problemInstance, this.solver, this.lowLevelSolver, runner);
            this.openList.Add(root);
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 1;
            this.maxSizeGroup = 1;
            this.totalCost = 0;
            if (problemInstance.parameters.ContainsKey(Trevor.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[Trevor.MAXIMUM_COST_KEY]);
            else
                this.maxCost = int.MaxValue;
            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT) == false)
            {
                problemInstance.parameters[CBS_LocalConflicts.INTERNAL_CAT] = new HashSet_U<TimedMove>();
                problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS] = new HashSet_U<CbsConstraint>();
                this.topMost = true;
            }
            else
                this.topMost = false;
            
            if (this.minDepth == -1)
                this.minDepth = 0;
        }

        public void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            this.minDepth = minDepth;
            Setup(problemInstance, runner);
        }

        public void SetHeuristic(HeuristicCalculator heuristic)
        {
            this.solver.SetHeuristic(heuristic);
        }

        public HeuristicCalculator GetHeuristic()
        {
            return this.solver.GetHeuristic();
        }

        public void Clear()
        {
            this.openList.Clear();
            this.closedList.Clear();
            this.solver.Clear();
            this.minDepth = -1;
        }

        public virtual String GetName() 
        {
            if (mergeThreshold == -1)
                return "Basic CBS_old";
            return "CBS_old Local(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver;
        }

        public int GetSolutionCost() { return this.totalCost; }

        protected void ClearPrivateStatistics()
        {
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 0;
            this.closedListHits = 0;
        }
        
        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Closed List Hits (HL)");
            output.Write(Run.RESULTS_DELIMITER);

            this.solver.OutputStatisticsHeader(output);

            this.openList.OutputStatisticsHeader(output);
        }

        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes (High-Level): {0}", this.GetHighLevelExpanded());
            Console.WriteLine("Total Generated Nodes (High-Level): {0}", this.GetHighLevelGenerated());
            Console.WriteLine("Closed List Hits (High-Level): {0}", this.closedListHits);

            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.closedListHits + Run.RESULTS_DELIMITER);

            this.solver.OutputAccumulatedStatistics(output);

            this.openList.OutputStatistics(output);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                return 3 + this.solver.NumStatsColumns + this.openList.NumStatsColumns;
            }
        }

        public virtual void ClearStatistics()
        {
            if (this.topMost)
                this.solver.ClearAccumulatedStatistics();
            this.ClearPrivateStatistics();
            this.openList.ClearStatistics();
        }

        public virtual void ClearAccumulatedStatistics()
        {
            this.accHLExpanded = 0;
            this.accHLGenerated = 0;
            this.accClosedListHits = 0;

            this.solver.ClearAccumulatedStatistics();

            this.openList.ClearAccumulatedStatistics();
        }

        public virtual void AccumulateStatistics()
        {
            this.accHLExpanded += this.highLevelExpanded;
            this.accHLGenerated += this.highLevelGenerated;
            this.accClosedListHits += this.closedListHits;

            // this.solver statistics are accumulated every time it's used.

            this.openList.AccumulateStatistics();
        }

        public virtual void OutputAccumulatedStatistics(TextWriter output)
        {
            Console.WriteLine("{0} Accumulated Expanded Nodes (High-Level): {1}", this, this.accHLExpanded);
            Console.WriteLine("{0} Accumulated Generated Nodes (High-Level): {1}", this, this.accHLGenerated);
            Console.WriteLine("{0} Accumulated Closed List Hits (High-Level): {1}", this, this.accClosedListHits);

            output.Write(this.accHLExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.accHLGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.accClosedListHits + Run.RESULTS_DELIMITER);

            this.solver.OutputAccumulatedStatistics(output);

            this.openList.OutputAccumulatedStatistics(output);
        }

        public bool Solve()
        {
            //Debug.WriteLine("Solving Sub-problem On Level - " + mergeThreshold);

            CbsConflict conflict;
            CbsNode currentNode = (CbsNode)openList.Remove();
            highLevelExpanded++;
            if (currentNode.Solve(minDepth) == false)
            {
                AgentState.EquivalenceOverDifferentTimes = true;
                return false;
            }
            if (currentNode.totalCost <= this.maxCost)
            {
                this.openList.Add(currentNode);
                this.closedList.Add(currentNode, currentNode);
                this.addToGlobalConflictCount(currentNode.GetConflict());
            }
            while (openList.Count > 0 && runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.Clear();
                    AgentState.EquivalenceOverDifferentTimes = true;
                    return false;
                }
                currentNode = (CbsNode)openList.Remove();
                conflict = currentNode.GetConflict();
                // Check if node is the goal
                if (conflict == null)
                {
                    this.totalCost = currentNode.totalCost;
                    this.goalNode = currentNode;
                    this.solution = currentNode.CalculateJointPlan();
                    this.Clear();
                    AgentState.EquivalenceOverDifferentTimes = true;
                    return true;
                }
                // Expand
                highLevelExpanded++;
                if (Expand(currentNode, conflict))
                    currentNode.Clear();
            }
            totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            AgentState.EquivalenceOverDifferentTimes = true;
            return false;
        }

        protected virtual bool MergeConflicting(CbsNode node)
        {
            return node.ShouldMerge(mergeThreshold);
        }

        public virtual bool Expand(CbsNode node, CbsConflict conflict)
        {
            if (this.maxThreshold != -1)
            {
                closedList.Remove(node); // This may be the last chance to do it, if a merge occurs in the next line
                if (MergeConflicting(node))
                {
                    closedList.Add(node, node); // With new hash code
                    bool solved = node.Replan(conflict.agentA, this.minDepth);
                    this.maxSizeGroup = Math.Max(this.maxSizeGroup, node.replanSize);
                    if (solved == false)
                        return true;
                    if (node.totalCost <= maxCost)
                        openList.Add(node);
                    this.addToGlobalConflictCount(node.GetConflict());
                    return false;
                }
                else
                    closedList.Add(node, node); // With old hash code
            }

            CbsConstraint con;
            CbsNode toAdd;

            con = new CbsConstraint(conflict, instance, true);
            toAdd = new CbsNode(node, con, conflict.agentA);

            if (closedList.ContainsKey(toAdd) == false)
            {
                if (toAdd.Replan(conflict.agentA, Math.Max(minDepth, conflict.timeStep)))
                {
                    if (toAdd.totalCost <= this.maxCost)
                    {
                        openList.Add(toAdd);
                        closedList.Add(toAdd, toAdd);
                        this.highLevelGenerated++;
                        addToGlobalConflictCount(toAdd.GetConflict());
                    }
                }
            }
            else
                this.closedListHits++;

            con = new CbsConstraint(conflict, instance, false);
            toAdd = new CbsNode(node, con, conflict.agentB);

            if (closedList.ContainsKey(toAdd) == false)
            {
                if (toAdd.Replan(conflict.agentB, Math.Max(minDepth, conflict.timeStep)))
                {
                    if (toAdd.totalCost <= this.maxCost)
                    {
                        openList.Add(toAdd);
                        closedList.Add(toAdd, toAdd);
                        this.highLevelGenerated++;
                        addToGlobalConflictCount(toAdd.GetConflict());
                    }
                }

            }
            else
                this.closedListHits++;

            return true;
        }

        protected virtual void addToGlobalConflictCount(CbsConflict conflict) { }

        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        private void PrintSolution(WorldState end)
        {
        }

        public int GetSolutionDepth() { return -1; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        public SinglePlan[] GetSinglePlans()
        {
            return goalNode.allSingleAgentPlans;
        }
        public int GetHighLevelExpanded() { return highLevelExpanded; }
        public int GetHighLevelGenerated() { return highLevelGenerated; }
        public int GetLowLevelExpanded() { return this.solver.GetAccumulatedExpanded(); }
        public int GetLowLevelGenerated() { return this.solver.GetAccumulatedGenerated(); }
        public int GetExpanded() { return highLevelExpanded; }
        public int GetGenerated() { return highLevelGenerated; }
        public int GetAccumulatedExpanded() { return accHLExpanded; }
        public int GetAccumulatedGenerated() { return accHLGenerated; }
        public int GetMaxGroupSize()
        {
            return this.maxSizeGroup;
        }

        public virtual int[] GetSingleCosts()
        {
            return null;
        }
    }
    class CBS_GlobalConflicts_OLD : CBS_old
    {
        int[][] globalConflictsCounter;

        public CBS_GlobalConflicts_OLD(ICbsSolver solver, int maxThreshold, int currentThreshold)
        {
            this.closedList = new Dictionary<CbsNode, CbsNode>();
            this.openList = new OpenList(this);
            this.mergeThreshold = currentThreshold;
            this.solver = solver;
            this.lowLevelSolver = solver;
            this.maxThreshold = maxThreshold;
            if (currentThreshold < maxThreshold)
            {
                this.solver = new CBS_GlobalConflicts_OLD(solver, maxThreshold, currentThreshold + 1);
            }
        }

        public CBS_GlobalConflicts_OLD(ICbsSolver solver) : base(solver) { }

        public override void Setup(ProblemInstance problemInstance, Run runner)
        {
            globalConflictsCounter = new int[problemInstance.m_vAgents.Length][];
            for (int i = 0; i < globalConflictsCounter.Length; i++)
            {
                globalConflictsCounter[i] = new int[i];
                for (int j = 0; j < i; j++)
                {
                    globalConflictsCounter[i][j] = 0;
                }
            }
            base.Setup(problemInstance, runner);
        }

        protected override bool MergeConflicting(CbsNode node)
        {
            return node.ShouldMerge(mergeThreshold, globalConflictsCounter);
        }

        protected override void addToGlobalConflictCount(CbsConflict conflict)
        {
            if (conflict != null)
                globalConflictsCounter[Math.Max(conflict.agentA, conflict.agentB)][Math.Min(conflict.agentA, conflict.agentB)]++;
        }

        public override string GetName()
        {
            if (mergeThreshold == -1)
                return "Basic CBS_OLD";
            return "CBS_OLD Global(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver;
        }
    }
}
