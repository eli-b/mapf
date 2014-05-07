using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    class CBS_LocalConflicts : ICbsSolver
    {
        /// <summary>
        /// The key of the constraints list used for each CBS node
        /// </summary>
        public static string CONSTRAINTS = "constraints";
        /// <summary>
        /// The key of the must constraints list used for each CBS node
        /// </summary>
        public static string MUST_CONSTRAINTS = "must constraints";
        /// <summary>
        /// The key of the internal CAT for CBS, used to favor A* nodes that have fewer conflicts with other routes during tie-breaking
        /// </summary>
        public static string INTERNAL_CAT = "internalCAT";

        protected ProblemInstance instance;
        public BinaryHeap openList;
        public Dictionary<CbsNode, CbsNode> closedList;
        public int highLevelExpanded;
        public int highLevelGenerated;
        public int totalCost;
        protected int solutionDepth;
        protected Run runner;
        protected CbsNode goalNode;
        protected Plan solution;
        /// <summary>
        /// Nodes with with a higher cost aren't generated
        /// </summary>
        protected int maxCost;
        /// <summary>
        /// Search is stopped when the minimum cost passes the target
        /// </summary>
        public int targetCost {set; get;}
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

        public virtual void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            this.instance = problemInstance;
            this.runner = runner;

            this.highLevelExpanded = 0;
            this.highLevelGenerated = 0;
            this.lowLevelExpanded = 0;
            this.lowLevelGenerated = 0;
            this.maxSizeGroup = 1;
            this.totalCost = 0;
            this.solutionDepth = -1;
            this.targetCost = int.MaxValue;

            if (problemInstance.parameters.ContainsKey(Trevor.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[Trevor.MAXIMUM_COST_KEY]);
            else
                this.maxCost = int.MaxValue;

            if (problemInstance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT) == false) // Top-most CBS solver
            {
                problemInstance.parameters[CBS_LocalConflicts.INTERNAL_CAT] = new HashSet_U<TimedMove>();
                problemInstance.parameters[CBS_LocalConflicts.CONSTRAINTS] = new HashSet_U<CbsConstraint>();
            }

            this.minDepth = minDepth;

            CbsNode root = new CbsNode(instance.m_vAgents.Length, problemInstance, this.solver, this.lowLevelSolver, runner);
            // Solve the root node
            bool solved = root.Solve(minDepth, ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated);
            
            if (solved && root.totalCost <= this.maxCost)
            {
                this.openList.Add(root);
                this.highLevelGenerated++;
                this.closedList.Add(root, root);
                this.addToGlobalConflictCount(root.GetConflict());
            }
        }

        public virtual void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.Setup(problemInstance, 0, runner);
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

        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes (High-Level): {0}", this.GetHighLevelExpanded());
            Console.WriteLine("Total Generated Nodes (High-Level): {0}", this.GetHighLevelGenerated());
            Console.WriteLine("Total Expanded Nodes (Low-Level): {0}", this.GetLowLevelExpanded());
            Console.WriteLine("Total Generated Nodes (Low-Level): {0}", this.GetLowLevelGenerated());

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

        private bool debug = false;

        public bool Solve()
        {
            //Debug.WriteLine("Solving Sub-problem On Level - " + mergeThreshold);
            int initialEstimate = 0;
            if (openList.Count > 0)
                initialEstimate = ((CbsNode)openList.Peek()).totalCost;

            int lastCost = -1;

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

                if (debug)
                {
                    Debug.WriteLine("Total cost so far: " + currentNode.totalCost);
                    Debug.WriteLine("Constraints so far: ");
                    foreach (CbsConstraint constraint in currentNode.GetConstraints())
                    {
                        Debug.WriteLine(constraint);
                    }
                    Debug.WriteLine("Conflict: " + currentNode.GetConflict());
                    currentNode.CalculateJointPlan().PrintPlan();
                    Debug.WriteLine("");
                    Debug.WriteLine("");
                }

                Debug.Assert(currentNode.totalCost >= lastCost, "CBS node with decreasing cost: " + currentNode.totalCost + " < " + lastCost);
                lastCost = currentNode.totalCost;

                // Check if node is the goal
                if (currentNode.GoalTest())
                {
                    if (debug)
                        Debug.WriteLine("-------------------------");
                    this.totalCost = currentNode.totalCost;
                    this.solutionDepth = this.totalCost - initialEstimate;
                    this.goalNode = currentNode;
                    this.solution = currentNode.CalculateJointPlan();
                    this.Clear();
                    return true;
                }

                // Check if node is good enough
                if (currentNode.totalCost >= this.targetCost) {
                    if (debug)
                        Debug.WriteLine("-------------------------");
                    this.totalCost = currentNode.totalCost; // This is the min possible cost so far.
                    //this.openList.Add(currentNode); // To be able to continue the search later
                    this.Clear();
                    return false;
                }
                
                // Expand
                Expand(currentNode);
                highLevelExpanded++;
                if (currentNode.agentAExpansion == CbsNode.ExpansionState.EXPANDED &&
                    currentNode.agentBExpansion == CbsNode.ExpansionState.EXPANDED) // Fully expanded
                    currentNode.Clear();
            }

            this.totalCost = Constants.NO_SOLUTION_COST;
            this.Clear();
            return false;
        }

        protected virtual bool MergeConflicting(CbsNode node)
        {
            return node.MergeIf(mergeThreshold);
        }

        public virtual void Expand(CbsNode node)
        {
            CbsConflict conflict = node.GetConflict();

            if (this.maxThreshold != -1)
            {
                closedList.Remove(node); // This may be the last chance to do it, if a merge occurs in the next line
                if (MergeConflicting(node))
                {
                    closedList.Add(node, node); // With new hash code
                    bool success = node.Replan(conflict.agentA, this.minDepth,
                                ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated);
                    this.maxSizeGroup = Math.Max(this.maxSizeGroup, node.replanSize);
                    if (success == false)
                    {
                        this.Clear();
                        return;
                    }
                    if (node.totalCost <= maxCost)
                        openList.Add(node);
                    this.addToGlobalConflictCount(node.GetConflict());
                    return;
                }
                else
                    closedList.Add(node, node); // With the old hash code
            }

            // Expand node, possibly partially:
            // Generate left child:
            CbsConstraint con;
            CbsNode toAdd;
            if (node.agentAExpansion == CbsNode.ExpansionState.NOT_EXPANDED && conflict.vertex == true &&
                Math.Max(minDepth, conflict.timeStep) >= node.allSingleAgentCosts[conflict.agentA])
            // Conflict happens when or after agent A reaches its goal.
            // The left child would cost a lot because:
            // A) All WAIT moves in the goal before leaving it now add to the g.
            // B) We force the low level to compute a path longer than the optimal,
            //    and with a bad suprise towards the end in the form of a constraint,
            //    so the low-level's SIC heuristic performs poorly.
            // C) We're banning the GOAL from all directions (since this is a vertex conflict),
            //    so any alternative plan will at least cost 1 more.
            //    We're ignoring edge conflicts because they can only happen at the goal when reaching it,
            //    and aren't guaranteed to increase the cost because the goal can still be possibly reached from another edge.
            {
                node.agentAExpansion = CbsNode.ExpansionState.DEFERRED;
                int agentAOldCost = node.allSingleAgentCosts[conflict.agentA];
                // Add the minimal delta in the child's cost:
                // since we're banning the goal at conflict.timeStep, it must at least do conflict.timeStep+1 steps
                node.totalCost += (ushort)(conflict.timeStep + 1 - agentAOldCost);
                openList.Add(node); // Re-insert node into open list with higher cost
            }
            else if (node.agentAExpansion != CbsNode.ExpansionState.EXPANDED)
            // Agent A expansion already skipped in the past or not forcing A from its goal - finally generate the child:
            {
                con = new CbsConstraint(conflict, instance, true);
                toAdd = new CbsNode(node, con, conflict.agentA);

                if (closedList.ContainsKey(toAdd) == false)
                {
                    if (toAdd.Replan(conflict.agentA, minDepth, // The node takes the max between minDepth and the max time over all constraints.
                                     ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            closedList.Add(toAdd, toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.GetConflict());
                        }
                    }
                    // else a timeout probably occured
                }
                node.agentAExpansion = CbsNode.ExpansionState.EXPANDED;
            }

            // Generate right child:
            if (node.agentBExpansion == CbsNode.ExpansionState.NOT_EXPANDED && conflict.vertex == true &&
                Math.Max(minDepth, conflict.timeStep) >= node.allSingleAgentCosts[conflict.agentB]) // Again, skip expansion
            {
                if (node.agentAExpansion == CbsNode.ExpansionState.DEFERRED)
                    throw new Exception("Unexpected: Expansion of both children differed, but this is a vertex conflict so that means the targets for the two agents are equal, which is illegal");
                
                node.agentBExpansion = CbsNode.ExpansionState.DEFERRED;
                int agentBOldCost = node.allSingleAgentCosts[conflict.agentB];
                node.totalCost += (ushort)(conflict.timeStep + 1 - agentBOldCost);
                openList.Add(node); // Re-insert node into open list with higher cost
                // TODO: Code duplication with agentA. Make this into a function.
            }
            else if (node.agentBExpansion != CbsNode.ExpansionState.EXPANDED)
            {
                con = new CbsConstraint(conflict, instance, false);
                toAdd = new CbsNode(node, con, conflict.agentB);

                if (closedList.ContainsKey(toAdd) == false)
                {
                    if (toAdd.Replan(conflict.agentB, minDepth, // The node takes the max between minDepth and the max time over all constraints.
                                     ref highLevelExpanded, ref highLevelGenerated, ref lowLevelExpanded, ref lowLevelGenerated))
                    {
                        if (toAdd.totalCost <= this.maxCost)
                        {
                            openList.Add(toAdd);
                            closedList.Add(toAdd, toAdd);
                            this.highLevelGenerated++;
                            addToGlobalConflictCount(toAdd.GetConflict());
                        }
                    }
                    // else a timeout probably occured
                }
                node.agentBExpansion = CbsNode.ExpansionState.EXPANDED;
            }
        }

        protected virtual void addToGlobalConflictCount(CbsConflict conflict) { }

        public virtual Plan GetPlan()
        {
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
