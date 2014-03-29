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
        protected int loweLevelExpanded;
        protected int loweLevelGenerated;
        protected ICbsSolver solver;
        protected ICbsSolver lowLevelSolver;
        protected int mergeThreshold, nextF, fBound;
        protected int minCost;
        protected int maxThreshold;
        protected int maxSizeGroup;
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
            CbsNode.allConstraintsForNode.Clear();
            
        }

        public int GetSolutionCost() { return this.totalCost; }

        public void OutputStatistics(TextWriter output)
        {
            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write("N/A" + Run.RESULTS_DELIMITER);
            output.Write(loweLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(Process.GetCurrentProcess().VirtualMemorySize64 + Run.RESULTS_DELIMITER + Run.RESULTS_DELIMITER);
        }

        public bool Solve(Run runner)
        {
            //Debug.WriteLine("Solving Sub-problem On Level - " + mergeThreshold);
            //Console.ReadLine();

            CBS_LocalConflicts.isCbs = true;
            CbsConflict conflict;
            this.runner = runner;

            if (root.solve(instance, runner, minCost, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated) == false)
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
                    CBS_LocalConflicts.isCbs = false;
                    this.Clear();
                    return false;
                }
                if (expand(root, root.getConflict()))
                    return true;
                fBound = nextF;
            }
            totalCost = Constants.NO_SOLUTION_COST;
            CBS_LocalConflicts.isCbs = false;
            this.Clear();
            return false;
        }

        public virtual bool expand(CbsNode node, CbsConflict conflict)
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
                CBS_LocalConflicts.isCbs = false;
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

            if (node.totalCost + conflict.timeStep + stepLength - node.pathLength(conflict.agentA) <= fBound)
            {
                ok1 = true;
                if (node.isAllowedConstraint(con1))
                {
                    toAdd = new CbsNode(node, con1, conflict.agentA, instance);
                    toAdd.setUnConstraint(con2);

                    if (toAdd.rePlan3b(instance, runner, conflict.agentA, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                    {
                        this.highLevelGenerated++;
                        if (toAdd.totalCost <= fBound)
                        {
                            if (expand(toAdd, toAdd.getConflict()))
                                return true;
                        }
                        else if (toAdd.totalCost < nextF)
                            nextF = toAdd.totalCost;
                    }
                }
            }

            if (node.totalCost + conflict.timeStep + stepLength - node.pathLength(conflict.agentB) <= fBound)
            {
                ok2 = true;
                if (node.isAllowedConstraint(con2))
                {
                    toAdd = new CbsNode(node, con2, conflict.agentB, instance);
                    toAdd.setUnConstraint(con1);

                    if (toAdd.rePlan3b(instance, runner, conflict.agentB, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                    {
                        this.highLevelGenerated++;
                        if (toAdd.totalCost <= fBound)
                        {
                            if (expand(toAdd, toAdd.getConflict()))
                                return true;
                        }
                        else if (toAdd.totalCost < nextF)
                            nextF = toAdd.totalCost;
                    }
                }
            }

            if (ok1 && ok2)
            {
                toAdd = new CbsNode(node, con1, conflict.agentA, instance);
                if (toAdd.rePlan3b(instance, runner, conflict.agentA, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                {
                    if (toAdd.totalCost <= fBound)
                    {
                        toAdd = new CbsNode(toAdd, con2, conflict.agentB, instance);
                        if (toAdd.rePlan(instance, runner, conflict.agentB, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                        {
                            this.highLevelGenerated++;
                            if (toAdd.totalCost <= fBound)
                            {
                                if (expand(toAdd, toAdd.getConflict()))
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

        public int getSolutionDepth() { return -1; }
        public int getNodesPassedPruningCounter() { return loweLevelExpanded; }
        public int getNodesFailedOn2Counter() { return -1; }
        public int getNodesFailedOn3Counter() { return -1; }
        public int getNodesFailedOn4Counter() { return -1; }
        public long getMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        public WorldState GetGoal() { throw new NotSupportedException("CBS doesn't have a traditional goal state as it solves the problem independently for each agent"); }
        public SinglePlan[] getSinglePlans()
        {
            return goalNode.allSingleAgentPlans;
        }
        public int getHighLevelExpanded() { return highLevelExpanded; }
        public int getHighLevelGenerated() { return highLevelGenerated; }
        public int getLowLevelExpanded() { return loweLevelExpanded; }
        public int getLowLevelGenerated() { return loweLevelGenerated; }
        public int getMaxGroupSize()
        {
            return this.maxSizeGroup;
        }

        public void Setup(ProblemInstance problemInstance)
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
            root = new CbsNode(instance.m_vAgents.Length);
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 1;
            loweLevelExpanded = 0;
            loweLevelGenerated = 0;
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
            CbsNode.allConstraintsForNode = new HashSet<CbsConstraint>();
            minCost = 0;
            CBS_LocalConflicts.isGlobal = true;
        }

        protected bool checkMerge(CbsNode node)
        {
            return node.checkMergeCondition(mergeThreshold, globalConflictsCounter);
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
