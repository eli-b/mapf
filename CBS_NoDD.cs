using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;


namespace CPF_experiment
{
    /// <summary>
    /// Cbs with no closed list. According to the paper, it's usually a bit faster since there are very few cycles in the search.
    /// </summary>
    class CBS_NoDD : ISolver
    {
        protected ProblemInstance instance;
        public BinaryHeap openList;
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
        protected int minCost;
        protected int maxThreshold;
        protected int maxSizeGroup;
        protected HeuristicCalculator heuristic;
        int[][] globalConflictsCounter;

        public CBS_NoDD(ICbsSolver solver, int maxThreshold = -1, int currentThreshold = -1)
        {
            this.openList = new BinaryHeap();
            this.mergeThreshold = currentThreshold;
            this.solver = solver;
            this.lowLevelSolver = solver;
            this.maxThreshold = maxThreshold;
            if (currentThreshold < maxThreshold)
            {
                this.solver = new CBS_GlobalConflicts(solver, maxThreshold, currentThreshold + 1);
            }
        }

        public void Clear()
        {
            this.openList.Clear();
            this.instance = null;
            this.solver.Clear();
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
            if (currentNode.Solve(minCost, ref highLevelExpanded,ref highLevelGenerated, ref lowLevelExpanded,ref lowLevelGenerated) == false)
            {
                return false;
            }
            if (currentNode.totalCost <= this.maxCost)
            {
                this.openList.Add(currentNode);
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
                    if (currentNode.collapse == 0)
                        currentNode.Clear();
            }
            totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            return false;
        }

        public virtual bool Expand(CbsNode node,CbsConflict conflict)
        {

            if (this.maxThreshold != -1 && MergeConflicting(node))
            {
                if (node.Replan(conflict.agentA, this.minCost, ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated) == false)
                {
                    if (node.replanSize > this.maxSizeGroup)
                        maxSizeGroup = node.replanSize;
                    return true;
                }
                if (node.replanSize > this.maxSizeGroup)
                    maxSizeGroup = node.replanSize;
                if (node.totalCost <= maxCost)
                    openList.Add(node);
                this.addToGlobalConflictCount(node.GetConflict());
                return false;
            }


             CbsConstraint con;
             CbsNode toAdd;
            if (node.collapse != CbsNode.ExpansionState.A_NOT_EXPANDED &&
                Math.Max(minCost, conflict.timeStep) > node.PathLength(conflict.agentA))
            {
                node.collapse = CbsNode.ExpansionState.A_NOT_EXPANDED;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.PathLength(conflict.agentA));
                openList.Add(node);
            }
            else
            {
                con = new CbsConstraint(conflict, instance, true);
                toAdd = new CbsNode(node, con, conflict.agentA);

                if (toAdd.Replan(conflict.agentA, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                {
                    if (toAdd.totalCost <= this.maxCost)
                    {
                        openList.Add(toAdd);
                        this.highLevelGenerated++;
                        addToGlobalConflictCount(toAdd.GetConflict());
                    }
                }
                
                if (node.collapse == CbsNode.ExpansionState.A_NOT_EXPANDED)
                    node.collapse = CbsNode.ExpansionState.NOT_EXPANDED;
            }
            if (node.collapse != CbsNode.ExpansionState.B_NOT_EXPANDED &&
                Math.Max(minCost, conflict.timeStep) > node.PathLength(conflict.agentB))
            {
                if (node.collapse != CbsNode.ExpansionState.NOT_EXPANDED)
                    throw new Exception("Expand/CBS");
                node.collapse = CbsNode.ExpansionState.B_NOT_EXPANDED;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.PathLength(conflict.agentB));
                openList.Add(node);
            }
            else
            {
                con = new CbsConstraint(conflict, instance, false);
                toAdd = new CbsNode(node, con, conflict.agentB);

                if (toAdd.Replan(conflict.agentB, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                {
                    if (toAdd.totalCost <= this.maxCost)
                    {
                        openList.Add(toAdd);
                        this.highLevelGenerated++;
                        addToGlobalConflictCount(toAdd.GetConflict());
                    }
                }

                if (node.collapse == CbsNode.ExpansionState.B_NOT_EXPANDED)
                    node.collapse = CbsNode.ExpansionState.NOT_EXPANDED;
            }
            return true;
        }

        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        private void PrintSolution(WorldState end)
        {
        }

        public int GetSolutionDepth() { return -1; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        public SinglePlan[] getSinglePlans()
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

        public void Setup(ProblemInstance problemInstance, Run runner)
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
            this.instance = problemInstance;
            this.runner = runner;
            CbsNode root = new CbsNode(instance.m_vAgents.Length, problemInstance, this.solver, this.lowLevelSolver, runner);
            this.openList.Add(root);
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 1;
            lowLevelExpanded = 0;
            lowLevelGenerated = 0;
            maxSizeGroup = 1;
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
            minCost = 0;
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

        protected bool MergeConflicting(CbsNode node)
        {
            return node.MergeIf(mergeThreshold, globalConflictsCounter);
        }

        protected void addToGlobalConflictCount(CbsConflict conflict)
        {
            if (conflict != null)
                globalConflictsCounter[Math.Max(conflict.agentA, conflict.agentB)][Math.Min(conflict.agentA, conflict.agentB)]++;
        }

        public string GetName()
        {
            if (mergeThreshold == -1)
                return "Basic CBS NoDD";
            return "CBS Global NoDD(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver;
        }
    }

    class CBS_NoDDb3 : ISolver
    {

        protected ProblemInstance instance;
        public BinaryHeap openList;
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
        protected HeuristicCalculator heuristic;
        protected int mergeThreshold;
        protected int minCost;
        protected int maxThreshold;
        protected int maxSizeGroup;
        int[][] globalConflictsCounter;

        public CBS_NoDDb3(ICbsSolver solver, int maxThreshold = -1, int currentThreshold = -1)
        {
            this.openList = new BinaryHeap();
            this.mergeThreshold = currentThreshold;
            this.solver = solver;
            this.lowLevelSolver = solver;
            this.maxThreshold = maxThreshold;
            if (currentThreshold < maxThreshold)
            {
                this.solver = new CBS_GlobalConflicts(solver, maxThreshold, currentThreshold + 1);
            }
        }

        public void Clear()
        {
            this.openList.Clear();
            this.instance = null;
            this.solver.Clear();
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

        public bool Solve()
        {
            //Debug.WriteLine("Solving Sub-problem On Level - " + mergeThreshold);
            //Console.ReadLine();

            CbsConflict conflict;
            CbsNode currentNode = (CbsNode)openList.Remove();

            highLevelExpanded++;
            if (currentNode.Solve(minCost, ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated) == false)
            {
                return false;
            }
            if (currentNode.totalCost <= this.maxCost)
            {
                this.openList.Add(currentNode);
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
                    if (currentNode.collapse == 0)
                        currentNode.Clear();
            }
            totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            return false;
        }

        public virtual bool Expand(CbsNode node, CbsConflict conflict)
        {
            if (this.maxThreshold != -1 && MergeConflicting(node))
            {
                if (node.Replan(conflict.agentA, this.minCost, ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated) == false)
                {
                    if (node.replanSize > this.maxSizeGroup)
                        maxSizeGroup = node.replanSize;
                    return true;
                }
                if (node.replanSize > this.maxSizeGroup)
                    maxSizeGroup = node.replanSize;
                if (node.totalCost <= maxCost)
                    openList.Add(node);
                this.addToGlobalConflictCount(node.GetConflict());
                return false;
            }


            CbsNode toAdd;
            CbsConstraint con2 = new CbsConstraint(conflict, instance, false);
            CbsConstraint con1 = new CbsConstraint(conflict, instance, true);
            if (node.collapse != CbsNode.ExpansionState.A_NOT_EXPANDED &&
                Math.Max(minCost, conflict.timeStep) > node.PathLength(conflict.agentA))
            {
                node.collapse = CbsNode.ExpansionState.A_NOT_EXPANDED;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.PathLength(conflict.agentA));
                openList.Add(node);
            }
            else 
            {
                if (node.IsAllowedConstraint(con1))
                {
                    toAdd = new CbsNode(node, con1, conflict.agentA);
                    toAdd.SetUnConstraint(con2);

                    if (toAdd.Replan(conflict.agentA, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.GetConflict());
                        }
                    }
                }
                if (node.collapse == CbsNode.ExpansionState.A_NOT_EXPANDED)
                    node.collapse = CbsNode.ExpansionState.NOT_EXPANDED;
            }
            if (node.collapse != CbsNode.ExpansionState.B_NOT_EXPANDED &&
                Math.Max(minCost, conflict.timeStep) > node.PathLength(conflict.agentB))
            {
                if (node.collapse != CbsNode.ExpansionState.NOT_EXPANDED)
                    throw new Exception("Expand/CBS");
                node.collapse = CbsNode.ExpansionState.B_NOT_EXPANDED;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.PathLength(conflict.agentB));
                openList.Add(node);
            }
            else 
            {
                if (node.IsAllowedConstraint(con2))
                {
                    toAdd = new CbsNode(node, con2, conflict.agentB);
                    toAdd.SetUnConstraint(con1);

                    if (toAdd.Replan(conflict.agentB, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.GetConflict());
                        }
                    }
                }
                if (node.collapse == CbsNode.ExpansionState.B_NOT_EXPANDED)
                    node.collapse = CbsNode.ExpansionState.NOT_EXPANDED;
            }
            if (node.collapse == CbsNode.ExpansionState.NOT_EXPANDED)
            {
                toAdd = new CbsNode(node, con1, conflict.agentA);
                if (toAdd.Replan(conflict.agentA, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                {
                    toAdd = new CbsNode(toAdd, con2, conflict.agentB);
                    if (toAdd.Replan(conflict.agentB, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.GetConflict());
                        }
                    }
                }
            }
            return true;
        }

        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        private void PrintSolution(WorldState end)
        {
        }

        public int GetSolutionDepth() { return -1; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        public SinglePlan[] getSinglePlans()
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

        public void Setup(ProblemInstance problemInstance, Run runner)
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
            this.instance = problemInstance;
            this.runner = runner;
            CbsNode root = new CbsNode(instance.m_vAgents.Length, problemInstance, this.solver, this.lowLevelSolver, runner);
            this.openList.Add(root);
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 1;
            lowLevelExpanded = 0;
            lowLevelGenerated = 0;
            maxSizeGroup = 1;
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
            minCost = 0;
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

        protected bool MergeConflicting(CbsNode node)
        {
            return node.MergeIf(mergeThreshold, globalConflictsCounter);
        }

        protected void addToGlobalConflictCount(CbsConflict conflict)
        {
            if (conflict != null)
                globalConflictsCounter[Math.Max(conflict.agentA, conflict.agentB)][Math.Min(conflict.agentA, conflict.agentB)]++;
        }

        public string GetName()
        {
            if (mergeThreshold == -1)
                return "Basic CBS NoDDb3";
            return "CBS Global NoDDb3(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver;
        }
    }
}
