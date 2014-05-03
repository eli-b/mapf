using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
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
        public BinaryHeap openList;
        public Dictionary<CbsNode, CbsNode> closedList;
        public int highLevelExpanded;
        public int highLevelGenerated;
        public int totalCost;
        protected Run runner;
        protected CbsNode goalNode;
        protected Plan solution;
        protected int maxCost;
        protected int lowLevelExpanded;
        protected int lowLevelGenerated;
        protected ICbsSolver solver;
        protected ICbsSolver lowLevelSolver;
        protected int mergeThreshold;
        protected int minDepth;
        protected int maxThreshold;
        protected int maxSizeGroup;
        protected HeuristicCalculator heuristic;

        public CBS_old() { }

        public CBS_old(ICbsSolver solver, int maxThreshold = -1, int currentThreshold = -1)
        {
            this.closedList = new Dictionary<CbsNode, CbsNode>();
            this.openList = new BinaryHeap();
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
            this.instance = problemInstance;
            this.runner = runner;
            CbsNode root = new CbsNode(instance.m_vAgents.Length, problemInstance, this.solver, this.lowLevelSolver, runner);
            this.openList.Add(root);
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 1;
            this.lowLevelExpanded = 0;
            this.lowLevelGenerated = 0;
            this.maxSizeGroup = 1;
            this.totalCost = 0;
            if (problemInstance.parameters.ContainsKey(Trevor.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[Trevor.MAXIMUM_COST_KEY]);
            else
                this.maxCost = int.MaxValue;
            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.NEW_INTERNAL_CAT) == false)
            {
                problemInstance.parameters[CBS_LocalConflicts.INTERNAL_CAT] = new HashSet_U<TimedMove>();
                problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS] = new HashSet_U<CbsConstraint>();
                problemInstance.parameters[CBS_LocalConflicts.NEW_CONSTRAINTS] = new HashSet<CbsConstraint>();
                problemInstance.parameters[CBS_LocalConflicts.NEW_INTERNAL_CAT] = new HashSet<TimedMove>();
            }
            else
            {
                problemInstance.parameters[CBS_LocalConflicts.NEW_INTERNAL_CAT] = new HashSet<TimedMove>();
                problemInstance.parameters[CBS_LocalConflicts.NEW_CONSTRAINTS] = new HashSet<CbsConstraint>();
            }
            minDepth = 0;
        }

        public void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            Setup(problemInstance, runner);
            this.minDepth = minDepth;
        }

        public void SetHeuristic(HeuristicCalculator heuristic)
        {
            this.heuristic = heuristic;
            this.solver.SetHeuristic(heuristic);
        }

        public HeuristicCalculator GetHeuristic()
        {
            return this.heuristic;
        }

        public void Clear()
        {
            this.openList.Clear();
            this.closedList.Clear();
            this.instance = null;
            this.solver.Clear();
        }

        public virtual String GetName() 
        {
            if (mergeThreshold == -1)
                return "Basic CBS_old";
            return "CBS_old Local(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver;
        }

        public int GetSolutionCost() { return this.totalCost; }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Expanded (LL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (LL)");
            output.Write(Run.RESULTS_DELIMITER);
        }

        public void OutputStatistics(TextWriter output)
        {
            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.lowLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.lowLevelGenerated + Run.RESULTS_DELIMITER);
        }

        public int NumStatsColumns
        {
            get
            {
                return 4;
            }
        }

        public bool Solve()
        {
            //Debug.WriteLine("Solving Sub-problem On Level - " + mergeThreshold);
            //Console.ReadLine();

            CbsConflict conflict;
            CbsNode currentNode = (CbsNode)openList.Remove();
            highLevelExpanded++;
            if (currentNode.Solve(minDepth, ref highLevelExpanded,ref highLevelGenerated, ref lowLevelExpanded,ref lowLevelGenerated) == false)
            {
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
                    return true;
                }
                // Expand
                highLevelExpanded++;
                if (Expand(currentNode, conflict))
                    if (currentNode.collapse != 0)
                        currentNode.Clear();
            }
            totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            return false;
        }

        protected virtual bool MergeConflicting(CbsNode node)
        {
            return node.MergeIf(mergeThreshold);
        }

        public virtual bool Expand(CbsNode node, CbsConflict conflict)
        {
            if (this.maxThreshold != -1)
            {
                closedList.Remove(node); // This may be the last chance to do it, if a merge occurs in the next line
                if (MergeConflicting(node))
                {
                    closedList.Add(node, node); // With new hash code
                    if (node.Replan(conflict.agentA, this.minDepth, ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated) == false)
                    {
                        this.maxSizeGroup = Math.Max(this.maxSizeGroup, node.replanSize);
                        return true;
                    }
                    this.maxSizeGroup = Math.Max(this.maxSizeGroup, node.replanSize);
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
                if (toAdd.Replan(conflict.agentA, Math.Max(minDepth, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
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

            con = new CbsConstraint(conflict, instance, false);
            toAdd = new CbsNode(node, con, conflict.agentB);

            if (closedList.ContainsKey(toAdd) == false)
            {
                if (toAdd.Replan(conflict.agentB, Math.Max(minDepth, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
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
        public int GetLowLevelExpanded() { return lowLevelExpanded; }
        public int GetLowLevelGenerated() { return lowLevelGenerated; }
        public int GetMaxGroupSize()
        {
            return this.maxSizeGroup;
        }
    }
    class CBS_GlobalConflicts_OLD : CBS_old
    {
        int[][] globalConflictsCounter;

        public CBS_GlobalConflicts_OLD(ICbsSolver solver, int maxThreshold, int currentThreshold)
        {
            this.closedList = new Dictionary<CbsNode, CbsNode>();
            this.openList = new BinaryHeap();
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
            return node.MergeIf(mergeThreshold, globalConflictsCounter);
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
