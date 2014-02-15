using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    class CBS_old : IDnCSolver
    {
          // The key of the constraints list used for each D&C node
        public static string CONSTRAINTS = "constraints";
        // The key of the internal CAT for DnC
        public static string INTERNAL_CAT = "internalCAT";
        // The key of recent constraints
        public static string NEW_CONSTRAINTS = "newConstraints";
        // The key of recent internal CAT for DnC
        public static string NEW_INTERNAL_CAT = "newInternalCAT";
        public static bool isGlobal;

        protected ProblemInstance instance;
        public BinaryHeap openList;
        public HashTable_C closedList;
        public int highLevelExpanded;
        public int highLevelGenerated;
        public int totalCost;
        protected Run runner;
        protected DnCNode goalNode;
        protected Plan solution;
        public static bool isDnC;
        protected int maxCost;
        protected int loweLevelExpanded;
        protected int loweLevelGenerated;
        protected IDnCSolver solver;
        protected IDnCSolver lowLevelSolver;
        protected int mergeThreshold;
        protected int minCost;
        protected int maxThreshold;
        protected int maxSizeGroup;

        public CBS_old() { }

        public CBS_old(IDnCSolver solver, int maxThreshold = -1, int currentThreshold = -1)
        {
            this.closedList = new HashTable_C();
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

        public virtual void Setup(ProblemInstance problemInstance)
        {
            this.instance = problemInstance;
            DnCNode root = new DnCNode(instance.m_vAgents.Length);
            this.openList.Add(root);
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
            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.NEW_INTERNAL_CAT)==false)
            {
                problemInstance.parameters[CBS_LocalConflicts.INTERNAL_CAT] = new HashSet_U<TimedMove>();
                problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS] = new HashSet_U<DnCConstraint>();
                problemInstance.parameters[CBS_LocalConflicts.NEW_CONSTRAINTS] = new HashSet<DnCConstraint>();
                problemInstance.parameters[CBS_LocalConflicts.NEW_INTERNAL_CAT] = new HashSet<TimedMove>();
            }
            else
            {
                problemInstance.parameters[CBS_LocalConflicts.NEW_INTERNAL_CAT] = new HashSet<TimedMove>();
                problemInstance.parameters[CBS_LocalConflicts.NEW_CONSTRAINTS] = new HashSet<DnCConstraint>();
            }
            DnCNode.allConstraintsForNode = new HashSet<DnCConstraint>();
            minCost = 0;
            isGlobal=false;
        }

        public void Setup(ProblemInstance problemInstance, int minDepth)
        {
            Setup(problemInstance);
            this.minCost = minDepth;
            isGlobal=false;
        }

        public void Clear()
        {
            this.openList.Clear();
            this.closedList.Clear();
            this.instance = null;
            this.solver.Clear();
            DnCNode.allConstraintsForNode.Clear();
        }

        public virtual String GetName() 
        {
            if (mergeThreshold == -1)
                return "Basic CBS_old";
            return "CBS_old Local(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver.GetName();
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

            isDnC = true;
            DnCConflict conflict;
            this.runner = runner;
            DnCNode currentNode = (DnCNode)openList.Remove();
            highLevelExpanded++;
            if (currentNode.solve(instance,runner,minCost , lowLevelSolver ,ref highLevelExpanded,ref highLevelGenerated, ref loweLevelExpanded,ref loweLevelGenerated) == false)
            {
                return false;
            }
            if (currentNode.totalCost <= this.maxCost)
            {
                this.openList.Add(currentNode);
                this.closedList.Add(currentNode);
                this.addToGlobalConflictCount(currentNode.getConflict());
            }
            while (openList.Count > 0 && runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    isDnC = false;
                    this.Clear();
                    return false;
                }
                currentNode = (DnCNode)openList.Remove();
                conflict = currentNode.getConflict();
                // Check if node is the goal
                if (conflict == null)
                {
                    this.totalCost = currentNode.totalCost;
                    this.goalNode = currentNode;
                    this.solution = currentNode.CalculateJointPlan();
                    isDnC = false;
                    this.Clear();
                    return true;
                }
                // Expand
                highLevelExpanded++;
                if (expand(currentNode, conflict))
                    if (currentNode.colapse != 0)
                        currentNode.clear();
            }
            totalCost = Constants.NO_SOLUTION_COST;
            isDnC = false;
            this.Clear();
            return false;
        }

        protected virtual bool checkMerge(DnCNode node)
        {
            return node.checkMergeCondition(mergeThreshold);
        }

        public virtual bool expand(DnCNode node, DnCConflict conflict)
        {
            closedList.Remove(node);

            if (this.maxThreshold != -1 && checkMerge(node))
            {
                if (closedList.Contains(node))
                    return true;
                if (node.rePlan(instance, runner, conflict.agentA, this.minCost, solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated) == false)
                {
                    if (node.replanSize > this.maxSizeGroup)
                        maxSizeGroup = node.replanSize;
                    return true;
                }
                if (node.replanSize > this.maxSizeGroup)
                    maxSizeGroup = node.replanSize;
                if (node.totalCost <= maxCost)
                    openList.Add(node);
                closedList.Add(node);
                this.addToGlobalConflictCount(node.getConflict());
                return false;
            }

            closedList.Add(node);

            DnCConstraint con;
            DnCNode toAdd;

            con = new DnCConstraint(conflict, instance, true);
            toAdd = new DnCNode(node, con, conflict.agentA, instance);

            if (closedList.Contains(toAdd) == false)
            {

                if (toAdd.rePlan(instance, runner, conflict.agentA, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                {
                    if (toAdd.totalCost <= this.maxCost)
                    {
                        openList.Add(toAdd);
                        closedList.Add(toAdd);
                        this.highLevelGenerated++;
                        addToGlobalConflictCount(toAdd.getConflict());
                    }
                }
            }



            con = new DnCConstraint(conflict, instance, false);
            toAdd = new DnCNode(node, con, conflict.agentB, instance);

            if (closedList.Contains(toAdd) == false)
            {
                if (toAdd.rePlan(instance, runner, conflict.agentB, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                {
                    if (toAdd.totalCost <= this.maxCost)
                    {
                        openList.Add(toAdd);
                        closedList.Add(toAdd);
                        this.highLevelGenerated++;
                        addToGlobalConflictCount(toAdd.getConflict());
                    }
                }

            }
            return true;
        }

        protected virtual void addToGlobalConflictCount(DnCConflict conflict) { }

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
        public WorldState GetGoal() { throw new NotSupportedException("Divide & Constraint dosent have a traditional goal state as it solves the problem independetly for each agent"); }
        public SinglePlan[] getSinglePlans()
        {
            return goalNode.allSingleAgentPlans;
        }
        public int getHighLeveExpanded() { return highLevelExpanded; }
        public int getHighLeveGenerated() { return highLevelGenerated; }
        public int getLowLevelExpanded() { return loweLevelExpanded; }
        public int getLowLevelGenerated() { return loweLevelGenerated; }
        public int getMaxGroupSize()
        {
            return this.maxSizeGroup;
        }
    }
    class CBS_GlobalConflicts_OLD : CBS_old
    {
        int[][] globalConflictsCounter;

        public CBS_GlobalConflicts_OLD(IDnCSolver solver, int maxThreshold, int currentThreshold)
        {
            this.closedList = new HashTable_C();
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

        public CBS_GlobalConflicts_OLD(IDnCSolver solver) : base(solver) { }

        public override void Setup(ProblemInstance problemInstance)
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
            base.Setup(problemInstance);
            isGlobal = true;
        }

        protected override bool checkMerge(DnCNode node)
        {
            return node.checkMergeCondition(mergeThreshold, globalConflictsCounter);
        }

        protected override void addToGlobalConflictCount(DnCConflict conflict)
        {
            if (conflict != null)
                globalConflictsCounter[Math.Max(conflict.agentA, conflict.agentB)][Math.Min(conflict.agentA, conflict.agentB)]++;
        }

        public override string GetName()
        {
            if (mergeThreshold == -1)
                return "Basic CBS_OLD";
            return "CBS_OLD Global(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver.GetName();
        }
    }
}
