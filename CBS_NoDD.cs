using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Diagnostics;


namespace CPF_experement
{
    class CBS_NoDD : ISolver
    {

         protected ProblemInstance instance;
        public BinaryHeap openList;
        public int highLevelExpanded;
        public int highLevelGenerated;
        public int totalCost;
        protected Run runner;
        protected DnCNode goalNode;
        protected Plan solution;
        protected int maxCost;
        protected int loweLevelExpanded;
        protected int loweLevelGenerated;
        protected IDnCSolver solver;
        protected IDnCSolver lowLevelSolver;
        protected int mergeThreshold;
        protected int minCost;
        protected int maxThreshold;
        protected int maxSizeGroup;
        int[][] globalConflictsCounter;

        public CBS_NoDD(IDnCSolver solver, int maxThreshold = -1, int currentThreshold = -1)
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
            DnCNode.allConstraintsForNode.Clear();
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

            CBS_LocalConflicts.isDnC = true;
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
                this.addToGlobalConflictCount(currentNode.getConflict());
            }
            while (openList.Count > 0 && runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    CBS_LocalConflicts.isDnC = false;
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
                    CBS_LocalConflicts.isDnC = false;
                    this.Clear();
                    return true;
                }
                // Expand
                highLevelExpanded++;
                if (expand(currentNode, conflict))
                    if (currentNode.colapse == 0)
                        currentNode.clear();
            }
            totalCost = Constants.NO_SOLUTION_COST;
            CBS_LocalConflicts.isDnC = false;
            this.Clear();
            return false;
        }

        public virtual bool expand(DnCNode node, DnCConflict conflict)
        {

            if (this.maxThreshold!=-1 && checkMerge(node))
            {
                if (node.rePlan(instance, runner, conflict.agentA, this.minCost, solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated) == false)
                {
                    if (node.replanSize > this.maxSizeGroup)
                        maxSizeGroup = node.replanSize;
                    return true;
                }
                if (node.replanSize > this.maxSizeGroup)
                    maxSizeGroup = node.replanSize;
                if(node.totalCost<=maxCost)
                    openList.Add(node);
                this.addToGlobalConflictCount(node.getConflict());
                return false;
            }


             DnCConstraint con;
             DnCNode toAdd;
             //colapse = 1 - 1st wasnot expanded 
             //colapse = 2 - 2st wasnot expanded 
             //colapse = 0 - both not expanded 
            if (node.colapse != 1 && Math.Max(minCost, conflict.timeStep) > node.pathLength(conflict.agentA))
            {
                node.colapse = 1;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.pathLength(conflict.agentA));
                openList.Add(node);
            }
            else
            {
                 con = new DnCConstraint(conflict, instance, true);
                 toAdd = new DnCNode(node, con, conflict.agentA, instance);


                    if (toAdd.rePlan(instance, runner, conflict.agentA, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.getConflict());
                        }
                    }
                
                if (node.colapse == 1)
                    node.colapse = 0;
            }
            if (node.colapse != 2 && Math.Max(minCost, conflict.timeStep) > node.pathLength(conflict.agentB))
            {
                if (node.colapse != 0)
                    throw new Exception("expand/CBS");
                node.colapse = 2;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.pathLength(conflict.agentB));
                openList.Add(node);
            }
            else
            {
                con = new DnCConstraint(conflict, instance, false);
                toAdd = new DnCNode(node, con, conflict.agentB, instance);

                    if (toAdd.rePlan(instance, runner, conflict.agentB, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.getConflict());
                        }
                    }
                
                if (node.colapse == 2)
                    node.colapse = 0;
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

        public int getSolutionDepth() { return -1; }
        public int getNodesPassedPruningCounter() { return loweLevelExpanded; }
        public int getNodesFailedOn2Counter() { return -1; }
        public int getNodesFailedOn3Counter() { return -1; }
        public int getNodesFailedOn4Counter() { return -1; }
        public long getMemuryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
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
            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.NEW_INTERNAL_CAT) == false)
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
            CBS_LocalConflicts.isGlobal = true;
        }

        protected bool checkMerge(DnCNode node)
        {
            return node.checkMergeCondition(mergeThreshold, globalConflictsCounter);
        }

        protected void addToGlobalConflictCount(DnCConflict conflict)
        {
            if (conflict != null)
                globalConflictsCounter[Math.Max(conflict.agentA, conflict.agentB)][Math.Min(conflict.agentA, conflict.agentB)]++;
        }

        public string GetName()
        {
            if (mergeThreshold == -1)
                return "Basic CBS NoDD";
            return "CBS Global NoDD(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver.GetName();
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
        protected DnCNode goalNode;
        protected Plan solution;
        protected int maxCost;
        protected int loweLevelExpanded;
        protected int loweLevelGenerated;
        protected IDnCSolver solver;
        protected IDnCSolver lowLevelSolver;
        protected int mergeThreshold;
        protected int minCost;
        protected int maxThreshold;
        protected int maxSizeGroup;
        int[][] globalConflictsCounter;

        public CBS_NoDDb3(IDnCSolver solver, int maxThreshold = -1, int currentThreshold = -1)
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
            DnCNode.allConstraintsForNode.Clear();
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

            CBS_LocalConflicts.isDnC = true;
            DnCConflict conflict;
            this.runner = runner;
            DnCNode currentNode = (DnCNode)openList.Remove();

            highLevelExpanded++;
            if (currentNode.solve(instance, runner, minCost, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated) == false)
            {
                return false;
            }
            if (currentNode.totalCost <= this.maxCost)
            {
                this.openList.Add(currentNode);
                this.addToGlobalConflictCount(currentNode.getConflict());
            }
            while (openList.Count > 0 && runner.ElapsedMilliseconds() < Constants.MAX_TIME)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    CBS_LocalConflicts.isDnC = false;
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
                    CBS_LocalConflicts.isDnC = false;
                    this.Clear();
                    return true;
                }
                // Expand
                highLevelExpanded++;
                if (expand(currentNode, conflict))
                    if (currentNode.colapse == 0)
                        currentNode.clear();
            }
            totalCost = Constants.NO_SOLUTION_COST;
            CBS_LocalConflicts.isDnC = false;
            this.Clear();
            return false;
        }

        public virtual bool expand(DnCNode node, DnCConflict conflict)
        {

            if (this.maxThreshold != -1 && checkMerge(node))
            {
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
                this.addToGlobalConflictCount(node.getConflict());
                return false;
            }


            DnCNode toAdd;
            DnCConstraint con2 = new DnCConstraint(conflict, instance, false);
            DnCConstraint con1 = new DnCConstraint(conflict, instance, true);
            //colapse = 1 - 1st wasnot expanded 
            //colapse = 2 - 2st wasnot expanded 
            //colapse = 0 - both not expanded 
            if (node.colapse != 1 && Math.Max(minCost, conflict.timeStep) > node.pathLength(conflict.agentA))
            {
                node.colapse = 1;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.pathLength(conflict.agentA));
                openList.Add(node);
            }
            else 
            {
                if (node.isAllowedConstraint(con1))
                {
                    toAdd = new DnCNode(node, con1, conflict.agentA, instance);
                    toAdd.setUnConstraint(con2);

                    if (toAdd.rePlan(instance, runner, conflict.agentA, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.getConflict());
                        }
                    }
                }
                if (node.colapse == 1)
                    node.colapse = 0;
            }
            if (node.colapse != 2 && Math.Max(minCost, conflict.timeStep) > node.pathLength(conflict.agentB))
            {
                if (node.colapse != 0)
                    throw new Exception("expand/CBS");
                node.colapse = 2;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.pathLength(conflict.agentB));
                openList.Add(node);
            }
            else 
            {
                if (node.isAllowedConstraint(con2))
                {
                    toAdd = new DnCNode(node, con2, conflict.agentB, instance);
                    toAdd.setUnConstraint(con1);

                    if (toAdd.rePlan(instance, runner, conflict.agentB, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.getConflict());
                        }
                    }
                }
                if (node.colapse == 2)
                    node.colapse = 0;
            }
            if (node.colapse == 0)
            {
                toAdd = new DnCNode(node, con1, conflict.agentA, instance);
                if (toAdd.rePlan(instance, runner, conflict.agentA, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                {
                    toAdd = new DnCNode(toAdd, con2, conflict.agentB, instance);
                    if (toAdd.rePlan(instance, runner, conflict.agentB, Math.Max(minCost, conflict.timeStep), solver, lowLevelSolver, ref highLevelExpanded, ref highLevelGenerated, ref loweLevelExpanded, ref loweLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.getConflict());
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

        public int getSolutionDepth() { return -1; }
        public int getNodesPassedPruningCounter() { return loweLevelExpanded; }
        public int getNodesFailedOn2Counter() { return -1; }
        public int getNodesFailedOn3Counter() { return -1; }
        public int getNodesFailedOn4Counter() { return -1; }
        public long getMemuryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
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
            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.NEW_INTERNAL_CAT) == false)
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
            CBS_LocalConflicts.isGlobal = true;
        }

        protected bool checkMerge(DnCNode node)
        {
            return node.checkMergeCondition(mergeThreshold, globalConflictsCounter);
        }

        protected void addToGlobalConflictCount(DnCConflict conflict)
        {
            if (conflict != null)
                globalConflictsCounter[Math.Max(conflict.agentA, conflict.agentB)][Math.Min(conflict.agentA, conflict.agentB)]++;
        }

        public string GetName()
        {
            if (mergeThreshold == -1)
                return "Basic CBS NoDDb3";
            return "CBS Global NoDDb3(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver.GetName();
        }
    }
}
