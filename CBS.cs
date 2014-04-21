using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    class CBS_LocalConflicts : ICbsSolver
    {

        // The key of the constraints list used for each CBS node
        public static string CONSTRAINTS = "constraints";
        // The key of the must constraints list used for each CBS node
        public static string CONSTRAINTSP = "constraintsp";
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
        /// <summary>
        /// Nodes with with a higher cost aren't generated
        /// </summary>
        protected int maxCost;
        /// <summary>
        /// For the low lever solver
        /// </summary>
        protected HeuristicCalculator heuristic;
        protected int lowLevelExpanded;
        protected int lowLevelGenerated;
        protected ICbsSolver solver;
        protected ICbsSolver lowLevelSolver;
        protected int mergeThreshold;
        protected int minDepth;
        protected int maxThreshold;
        protected int maxSizeGroup;

        public CBS_LocalConflicts(ICbsSolver solver, int maxThreshold = -1, int currentThreshold = -1, HeuristicCalculator heuristic = null)
        {
            this.closedList = new Dictionary<CbsNode, CbsNode>();
            this.openList = new BinaryHeap();
            this.mergeThreshold = currentThreshold;
            this.solver = solver;
            this.lowLevelSolver = solver;
            this.maxThreshold = maxThreshold;
            if (currentThreshold < maxThreshold)
            {
                this.solver = new CBS_LocalConflicts(solver, maxThreshold, currentThreshold + 1, heuristic);
            }
            this.heuristic = heuristic;
        }

        public virtual void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.instance = problemInstance;
            this.runner = runner;

            this.highLevelExpanded = 0;
            this.highLevelGenerated = 0;
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
            minDepth = 0;
            CbsNode root = new CbsNode(instance.m_vAgents.Length, problemInstance, this.solver, this.lowLevelSolver, runner);
            // Solve the root node
            root.Solve(minDepth, ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated);
            
            if (root.totalCost <= this.maxCost)
            {
                this.openList.Add(root);
                this.highLevelGenerated++;
                this.closedList.Add(root, root);
                this.addToGlobalConflictCount(root.GetConflict());
            }
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
                return "Basic CBS";
            return "CBS Local(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver;
        }

        public override string ToString()
        {
            return GetName();
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

            while (openList.Count > 0)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.Clear();
                    return false;
                }
                var currentNode = (CbsNode)openList.Remove();

                // Check if node is the goal
                if (currentNode.GoalTest())
                {
                    this.totalCost = currentNode.totalCost;
                    this.goalNode = currentNode;
                    this.solution = currentNode.CalculateJointPlan();
                    this.Clear();
                    return true;
                }
                // Expand
                var ret = Expand(currentNode);
                highLevelExpanded++;
                if (ret)
                    if (currentNode.collapse == CbsNode.ExpansionState.NOT_EXPANDED) // Actually means the node was fully expanded
                        currentNode.Clear(); // TODO: Consider if this is even worth doing
            }
            this.totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            return false;
        }

        protected virtual bool MergeConflicting(CbsNode node)
        {
            return node.MergeIf(mergeThreshold);
        }

        public virtual bool Expand(CbsNode node)
        {
            CbsConflict conflict = node.GetConflict();

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

            // Expand node, possibly partially:
            // Generate left child:
            CbsConstraint con;
            CbsNode toAdd;
            if (node.collapse != CbsNode.ExpansionState.A_NOT_EXPANDED && 
                Math.Max(minDepth, conflict.timeStep) > node.PathLength(conflict.agentA)) // Conflict happens after agent A reached its goal.
                                                                                          // Generating the left child would mean moving agent a from the goal,
                                                                                          // which costs a lot because:
                                                                                          // A) All STAY moves in the goal before leaving it now add to the g.
                                                                                          // B) We force the low level to compute a path longer than the optimal,
                                                                                          //    and with a bad suprise towards the end in the form of a constraint,
                                                                                          //    so the low-level's SIC heuristic performs poorly.
            {
                node.collapse = CbsNode.ExpansionState.A_NOT_EXPANDED;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.PathLength(conflict.agentA)); // Add the minimal delta in the left child's cost: since we're banning the goal at conflict.timeStep, it must at least do conflict.timeStep+1 steps
                openList.Add(node); // Re-insert node into open list with higher cost
            }
            else // Agent A expansion already skipped in the past or not forcing A from its goal - finally generate the child:
            {
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
                 if (node.collapse == CbsNode.ExpansionState.A_NOT_EXPANDED)
                     node.collapse = CbsNode.ExpansionState.NOT_EXPANDED; // FIXME: This may be correct but it's very confusing. Just have two vars for A expansion and B expansion
            }

            // Generate right child:
            if (node.collapse != CbsNode.ExpansionState.B_NOT_EXPANDED &&
                Math.Max(minDepth, conflict.timeStep) > node.PathLength(conflict.agentB)) // Again, skip expansion
            {
                if (node.collapse != CbsNode.ExpansionState.NOT_EXPANDED) // A_NOT_EXPANDED is unexpected because it means both agents were at their goal when they collided, and valid problems don't have colliding goals
                    throw new Exception("Expand/CBS");
                node.collapse = CbsNode.ExpansionState.B_NOT_EXPANDED;
                node.totalCost += (ushort)(conflict.timeStep + 1 - node.PathLength(conflict.agentB));
                openList.Add(node);
            }
            else
            {
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
                if (node.collapse == CbsNode.ExpansionState.B_NOT_EXPANDED)
                    node.collapse = CbsNode.ExpansionState.NOT_EXPANDED;
            }
            return true;
        }

        protected virtual void addToGlobalConflictCount(CbsConflict conflict) { }

        public virtual Plan GetPlan()
        {
            return this.solution;
        }

        private void PrintSolution(WorldState end) // ?
        {
        }

        public int GetSolutionDepth() { return -1; }
        public int GetNodesPassedPruningCounter() { return lowLevelExpanded; }
        public int getNodesFailedOn2Counter() { return -1; }
        public int getNodesFailedOn3Counter() { return -1; }
        public int getNodesFailedOn4Counter() { return -1; }
        public long GetMemoryUsed() { return Process.GetCurrentProcess().VirtualMemorySize64; }
        public WorldState GetGoal() { throw new NotSupportedException("CBS doesn't have a traditional goal state as it solves the problem independently for each agent"); }
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

    class CBS_GlobalConflicts : CBS_LocalConflicts
    {
        int[][] globalConflictsCounter;

        public CBS_GlobalConflicts(ICbsSolver solver, int maxThreshold, int currentThreshold, HeuristicCalculator heuristic = null)
            : base(solver, maxThreshold, currentThreshold, heuristic)
        {
            if (currentThreshold < maxThreshold) // FIXME: base's this.solver allocated for no reason
            {
                this.solver = new CBS_GlobalConflicts(solver, maxThreshold, currentThreshold + 1, heuristic);
            }
        }

        public CBS_GlobalConflicts(ICbsSolver solver) : base(solver) { }

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
                return "Basic CBS";
            return "CBS Global(" + mergeThreshold + ")(" + maxThreshold + ")+" + lowLevelSolver;
        }
    }

   
}
