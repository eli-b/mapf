using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;


namespace CPF_experiment
{
    class CBS_IDA : ISolver
    {
        protected ProblemInstance instance;
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
        protected int mergeThreshold, nextF, fBound;
        protected int minCost;
        protected int maxThreshold;
        protected int maxSizeGroup;
        protected HeuristicCalculator heuristic;
        int[][] globalConflictsCounter;
        CbsNode root;

        public CBS_IDA(ICbsSolver solver, int maxThreshold = -1, int currentThreshold = -1)
        {
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
            this.instance = null;
            this.solver.Clear();
        }

        public int GetSolutionCost() { return this.totalCost; }

        public void OutputStatistics(TextWriter output)
        {
            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write(lowLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(Process.GetCurrentProcess().VirtualMemorySize64 + Run.RESULTS_DELIMITER + Run.RESULTS_DELIMITER);
        }

        public bool Solve()
        {
            //Debug.WriteLine("Solving Sub-problem On Level - " + mergeThreshold);
            //Console.ReadLine();

            if (root.Solve(minCost, ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated) == false)
            {
                return false;
            }

            fBound = root.totalCost;

            //fBound must be <= maxCost
            while (fBound < maxCost)
            {
                nextF = int.MaxValue;
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.Clear();
                    return false;
                }
                if (Expand(root, root.GetConflict()))
                    return true;
                fBound = nextF;
            }
            totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            return false;
        }

        public virtual bool Expand(CbsNode node, CbsConflict conflict)
        {
            if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
            {
                return false;
            }
            highLevelExpanded++;
            if (conflict == null)
            {
                this.totalCost = node.totalCost;
                this.goalNode = node;
                this.solution = node.CalculateJointPlan();
                this.Clear();
                return true;
            }
            CbsNode toAdd;
            CbsConstraint con2 = new CbsConstraint(conflict, instance, false);
            CbsConstraint con1 = new CbsConstraint(conflict, instance, true);
            byte stepLength=0;
            if (conflict.vartex)
                stepLength = 1;
            bool ok1 = false, ok2 = false;

            if (node.totalCost + conflict.timeStep + stepLength - node.PathLength(conflict.agentA) <= fBound)
            {
                ok1 = true;
                if (node.IsAllowedConstraint(con1))
                {
                    toAdd = new CbsNode(node, con1, conflict.agentA);
                    toAdd.SetUnConstraint(con2);

                    if (toAdd.Replan3b(conflict.agentA, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                    {
                        this.highLevelGenerated++;
                        if (toAdd.totalCost <= fBound)
                        {
                            if (Expand(toAdd, toAdd.GetConflict()))
                                return true;
                        }
                        else if (toAdd.totalCost < nextF)
                            nextF = toAdd.totalCost;
                    }
                }
            }

            if (node.totalCost + conflict.timeStep + stepLength - node.PathLength(conflict.agentB) <= fBound)
            {
                ok2 = true;
                if (node.IsAllowedConstraint(con2))
                {
                    toAdd = new CbsNode(node, con2, conflict.agentB);
                    toAdd.SetUnConstraint(con1);

                    if (toAdd.Replan3b(conflict.agentB, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                    {
                        this.highLevelGenerated++;
                        if (toAdd.totalCost <= fBound)
                        {
                            if (Expand(toAdd, toAdd.GetConflict()))
                                return true;
                        }
                        else if (toAdd.totalCost < nextF)
                            nextF = toAdd.totalCost;
                    }
                }
            }

            if (ok1 && ok2)
            {
                toAdd = new CbsNode(node, con1, conflict.agentA);
                if (toAdd.Replan3b(conflict.agentA, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                {
                    if (toAdd.totalCost <= fBound)
                    {
                        toAdd = new CbsNode(toAdd, con2, conflict.agentB);
                        if (toAdd.Replan(conflict.agentB, Math.Max(minCost, conflict.timeStep), ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                        {
                            this.highLevelGenerated++;
                            if (toAdd.totalCost <= fBound)
                            {
                                if (Expand(toAdd, toAdd.GetConflict()))
                                    return true;
                            }
                            else if (toAdd.totalCost < nextF)
                                nextF = toAdd.totalCost;
                        }
                    }
                }
            }
            return false; ;
        }

        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        private void PrintSolution(WorldState end)
        {
        }

        public int GetSolutionDepth() { return -1; }
        public int GetNodesPassedPruningCounter() { return lowLevelExpanded; }
        public int getNodesFailedOn2Counter() { return -1; }
        public int getNodesFailedOn3Counter() { return -1; }
        public int getNodesFailedOn4Counter() { return -1; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        public WorldState GetGoal() { throw new NotSupportedException("CBS doesn't have a traditional goal state as it solves the problem independently for each agent"); }
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
            root = new CbsNode(instance.m_vAgents.Length, problemInstance, this.solver, this.lowLevelSolver, runner);
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
                problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTSP] = new List<CbsConstraint>();
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
                return "DFID-CBS3b";
            return "Bad Input As Solver CBS_IDA*";
        }
    }
}
