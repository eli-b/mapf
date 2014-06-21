using System;
using System.Collections.Generic;
using System.IO;
using System.Diagnostics;

namespace CPF_experiment
{
    /// <summary>
    /// Merges agents if they conflict more times than the given threshold in the CT nodes from the root to the current CT nodes only.
    /// </summary>
    public class CBS_LocalConflicts : ICbsSolver
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
        public OpenList openList;
        public Dictionary<CbsNode, CbsNode> closedList;
        protected int highLevelExpanded;
        protected int highLevelGenerated;
        protected int closedListHits;
        protected int accHLExpanded;
        protected int accHLGenerated;
        protected int accClosedListHits;

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
        /// Search is stopped when the low level generated nodes count exceeds the cap
        /// </summary>
        public int lowLevelGeneratedCap { set; get; }
        /// <summary>
        /// Search is stopped when the millisecond count exceeds the cap
        /// </summary>
        public int milliCap { set; get; }
        protected ICbsSolver solver;
        protected ICbsSolver lowLevelSolver;
        protected int mergeThreshold;
        protected int minDepth;
        protected int maxMergeThreshold;
        protected int maxSizeGroup;
        /// <summary>
        /// Used to know when to clear problem parameters.
        /// </summary>
        protected bool topMost;

        public CBS_LocalConflicts(ICbsSolver solver, int maxThreshold = -1, int currentThreshold = -1)
        {
            this.closedList = new Dictionary<CbsNode, CbsNode>();
            this.openList = new OpenList(this);
            this.mergeThreshold = currentThreshold;
            this.solver = solver;
            this.lowLevelSolver = solver;
            this.maxMergeThreshold = maxThreshold;
            if (currentThreshold < maxThreshold)
            {
                this.solver = new CBS_LocalConflicts(solver, maxThreshold, currentThreshold + 1);
            }
        }
        
        public virtual void Setup(ProblemInstance problemInstance, int minDepth, Run runner)
        {
            this.instance = problemInstance;
            this.runner = runner;

            this.ClearPrivateStatistics();
            this.totalCost = 0;
            this.solutionDepth = -1;
            this.targetCost = int.MaxValue;
            this.lowLevelGeneratedCap = int.MaxValue;
            this.milliCap = int.MaxValue;

            if (problemInstance.parameters.ContainsKey(Trevor.MAXIMUM_COST_KEY))
                this.maxCost = (int)(problemInstance.parameters[Trevor.MAXIMUM_COST_KEY]);
            else
                this.maxCost = int.MaxValue;

            this.topMost = this.SetGlobals();

            this.minDepth = minDepth;

            CbsNode root = new CbsNode(instance.m_vAgents.Length, problemInstance, this.solver, this.lowLevelSolver, runner);
            // Solve the root node
            bool solved = root.Solve(minDepth);
            
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
            this.solver.SetHeuristic(heuristic);
        }

        public HeuristicCalculator GetHeuristic()
        {
            return this.solver.GetHeuristic();
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

        public virtual String GetName() 
        {
            if (mergeThreshold == -1)
                return "Basic CBS/" + lowLevelSolver;
            return "MA-CBS Local(" + mergeThreshold + ")(" + maxMergeThreshold + ")/" + lowLevelSolver;
        }

        public override string ToString()
        {
            return GetName();
        }

        public int GetSolutionCost() { return this.totalCost; }

        protected void ClearPrivateStatistics()
        {
            this.highLevelExpanded = 0;
            this.highLevelGenerated = 0;
            this.closedListHits = 0;
            this.maxSizeGroup = 1;
        }

        public virtual void OutputStatisticsHeader(TextWriter output)
        {
            output.Write(this.ToString() + " Expanded (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Generated (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            output.Write(this.ToString() + " Closed List Hits (HL)");
            output.Write(Run.RESULTS_DELIMITER);
            //output.Write(this.ToString() + " Max Group Size (HL)");
            //output.Write(Run.RESULTS_DELIMITER);

            this.solver.OutputStatisticsHeader(output);

            this.openList.OutputStatisticsHeader(output);
        }

        public virtual void OutputStatistics(TextWriter output)
        {
            Console.WriteLine("Total Expanded Nodes (High-Level): {0}", this.GetHighLevelExpanded());
            Console.WriteLine("Total Generated Nodes (High-Level): {0}", this.GetHighLevelGenerated());
            Console.WriteLine("Closed List Hits (High-Level): {0}", this.closedListHits);
            //Console.WriteLine("Max Group Size (High-Level): {0}", this.maxSizeGroup);

            output.Write(this.highLevelExpanded + Run.RESULTS_DELIMITER);
            output.Write(this.highLevelGenerated + Run.RESULTS_DELIMITER);
            output.Write(this.closedListHits + Run.RESULTS_DELIMITER);
            //output.Write(this.maxSizeGroup + Run.RESULTS_DELIMITER);

            this.solver.OutputAccumulatedStatistics(output);

            this.openList.OutputStatistics(output);
        }

        public virtual int NumStatsColumns
        {
            get
            {
                return 4 + this.solver.NumStatsColumns + this.openList.NumStatsColumns;
            }
        }

        public virtual void ClearStatistics()
        {
            if (this.topMost)
                this.solver.ClearAccumulatedStatistics(); // Is this correct? Or is it better not to do it?
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
            //this.accMaxGroupSize = Math.Max(this.accMaxGroupSize, this.maxSizeGroup)

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
            //output.Write(this.accMaxSizeGroup + Run.RESULTS_DELIMITER);

            this.solver.OutputAccumulatedStatistics(output);

            this.openList.OutputAccumulatedStatistics(output);
        }

        private bool debug = false;

        /// <summary>
        /// 
        /// </summary>
        /// <returns>Whether this is the top-most CBS</returns>
        protected bool SetGlobals()
        {
            AgentState.EquivalenceOverDifferentTimes = false;
            if (this.instance.parameters.ContainsKey(CBS_LocalConflicts.INTERNAL_CAT) == false) // Top-most CBS solver
            {
                this.instance.parameters[CBS_LocalConflicts.INTERNAL_CAT] = new HashSet_U<TimedMove>();
                this.instance.parameters[CBS_LocalConflicts.CONSTRAINTS] = new HashSet_U<CbsConstraint>();
                return true;
            }
            else
                return false;
        }

        protected void CleanGlobals()
        {
            AgentState.EquivalenceOverDifferentTimes = true;
            if (this.topMost) // Clear problem parameters
            {
                this.instance.parameters.Remove(CBS_LocalConflicts.INTERNAL_CAT);
                this.instance.parameters.Remove(CBS_LocalConflicts.CONSTRAINTS);
                // Don't remove must constraints:
                // A) It wasn't CBS that added them.
                // B) Must constraints only appear in temporary problems.
                //    There's no danger of leaking them to other solvers because if they exist then this is a temporary problem.
                // C) We don't have the information to re-create them later.
            }
        }

        public bool Solve()
        {
            this.SetGlobals(); // Again, because we might be resuming a search that was stopped.

            //Debug.WriteLine("Solving Sub-problem On Level - " + mergeThreshold);
            int initialEstimate = 0;
            if (openList.Count > 0)
                initialEstimate = ((CbsNode)openList.Peek()).totalCost;

            int maxExpandedCost = -1;

            while (openList.Count > 0)
            {
                // Check if max time has been exceeded
                if (runner.ElapsedMilliseconds() > Constants.MAX_TIME)
                {
                    totalCost = Constants.TIMEOUT_COST;
                    Console.WriteLine("Out of time");
                    this.Clear(); // Total search time exceeded - we're not going to resume this search.
                    this.CleanGlobals();
                    return false;
                }
                var currentNode = (CbsNode)openList.Remove();

                if (debug)
                {
                    Debug.WriteLine("Total cost so far: " + currentNode.totalCost);
                    Debug.WriteLine("Expansion state: " + currentNode.agentAExpansion + ", " + currentNode.agentBExpansion);
                    var constraints = currentNode.GetConstraints();
                    Debug.WriteLine(constraints.Count.ToString() + " constraints so far: ");
                    foreach (CbsConstraint constraint in constraints)
                    {
                        Debug.WriteLine(constraint);
                    }
                    Debug.WriteLine("Conflict: " + currentNode.GetConflict());
                    Debug.Write("Agent group assignments: ");
                    for (int i = 0; i < currentNode.agentsGroupAssignment.Length; i++)
                    {
                        Debug.Write(" " + currentNode.agentsGroupAssignment[i]);
                    }
                    Debug.WriteLine("");
                    currentNode.CalculateJointPlan().PrintPlan();
                    Debug.WriteLine("");
                    Debug.WriteLine("");
                }

                maxExpandedCost = Math.Max(maxExpandedCost, currentNode.totalCost);

                // Check if node is the goal
                if (currentNode.GoalTest())
                {
                    Debug.Assert(currentNode.totalCost >= maxExpandedCost, "CBS goal node found with lower cost than the max cost node ever expanded: " + currentNode.totalCost + " < " + maxExpandedCost);
                    // This is subtle, but MA-CBS may expand nodes in a non non-decreasing order:
                    // If a node with a non-optimal constraint is expanded and we decide to merge the agents,
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

                    if (debug)
                        Debug.WriteLine("-------------------------");
                    this.totalCost = currentNode.totalCost;
                    this.solutionDepth = this.totalCost - initialEstimate;
                    this.goalNode = currentNode;
                    this.solution = currentNode.CalculateJointPlan();
                    this.Clear(); // Goal found - we're not going to resume this search
                    this.CleanGlobals();
                    return true;
                }

                if (currentNode.totalCost >= this.targetCost || // Node is good enough
                    //(this.targetCost != int.MaxValue &&
                     //this.lowLevelGenerated > Math.Pow(Constants.NUM_ALLOWED_DIRECTIONS, this.instance.m_vAgents.Length))
                    this.solver.GetAccumulatedGenerated() > this.lowLevelGeneratedCap || // Stop because this is taking too long.
                                                                                         // We're looking at _generated_ low level nodes since that's an indication to the amount of work done,
                                                                                         // while expanded nodes is an indication of the amount of good work done.
                                                                                         // b**k is the maximum amount of nodes we'll generate if we expand this node with A*.
                    (this.milliCap != int.MaxValue && // (This check is much cheaper than the method call)
                     this.runner.ElapsedMilliseconds() > this.milliCap)) // Search is taking too long.
                {
                    if (debug)
                        Debug.WriteLine("-------------------------");
                    this.totalCost = currentNode.totalCost; // This is the min possible cost so far.
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

            this.totalCost = Constants.NO_SOLUTION_COST;
            this.Clear(); // unsolvable problem - we're not going to resume it
            this.CleanGlobals();
            return false;
        }

        protected virtual bool MergeConflicting(CbsNode node)
        {
            return node.MergeIf(mergeThreshold);
        }

        public virtual void Expand(CbsNode node)
        {
            CbsConflict conflict = node.GetConflict();

            if (this.maxMergeThreshold != -1)
            {
                closedList.Remove(node); // This may be the last chance to do it, if a merge occurs in the next line
                if (MergeConflicting(node))
                {
                    if (closedList.ContainsKey(node) == false) // We may have already merged these agents in the parent
                    {
                        if (debug)
                            Debug.WriteLine("Merging agents {0} and {1}", conflict.agentA, conflict.agentB);
                        closedList.Add(node, node); // With new hash code
                        bool success = node.Replan(conflict.agentA, this.minDepth);
                        if (debug)
                        {
                            Debug.WriteLine("New cost: " + node.totalCost);
                            Debug.WriteLine("Same constraints");
                            Debug.WriteLine("New conflict: " + node.GetConflict());
                            Debug.Write("Agent group assignments: ");
                            for (int i = 0; i < node.agentsGroupAssignment.Length; i++)
                            {
                                Debug.Write(" " + node.agentsGroupAssignment[i]);
                            }
                            Debug.WriteLine("");
                            node.CalculateJointPlan().PrintPlan();
                            Debug.WriteLine("");
                            Debug.WriteLine("");

                        }
                        this.maxSizeGroup = Math.Max(this.maxSizeGroup, node.replanSize);
                        // Clear partial expansion state - this is actually a new node with new paths and a new conflict.
                        // Any child node already created is trying to solve the old conflict that was just solved here by
                        // merging the agents. When it is expanded it will inevitably also choose to merge the agents instead
                        // of expanding, since the conflict count never decrements, and will be stopped by the closed list check.
                        // Any deffered child should have another chance to be deferred - this is a new conflict.
                        node.agentAExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
                        node.agentBExpansion = CbsNode.ExpansionState.NOT_EXPANDED;
                        if (success == false)
                        {
                            this.Clear();
                            return;
                        }
                        if (node.totalCost <= maxCost) // FIXME: Code dup with other new node creations
                        {
                            openList.Add(node);
                            this.addToGlobalConflictCount(node.GetConflict());
                        }
                    }
                    return; // Don't expand this node yet. Wait for it to pop from the open list again.
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
                openList.Add(node); // Re-insert node into open list with higher cost, don't re-increment global conflict counts
            }
            else if (node.agentAExpansion != CbsNode.ExpansionState.EXPANDED)
            // Agent A expansion already skipped in the past or not forcing A from its goal - finally generate the child:
            {
                con = new CbsConstraint(conflict, instance, true);
                toAdd = new CbsNode(node, con, conflict.agentA);

                if (closedList.ContainsKey(toAdd) == false)
                {
                    if (toAdd.Replan(conflict.agentA, minDepth)) // The node takes the max between minDepth and the max time over all constraints.
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
                else
                    this.closedListHits++;
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
                openList.Add(node); // Re-insert node into open list with higher cost, don't re-increment global conflict counts
                // TODO: Code duplication with agentA. Make this into a function.
            }
            else if (node.agentBExpansion != CbsNode.ExpansionState.EXPANDED)
            {
                con = new CbsConstraint(conflict, instance, false);
                toAdd = new CbsNode(node, con, conflict.agentB);

                if (closedList.ContainsKey(toAdd) == false)
                {
                    if (toAdd.Replan(conflict.agentB, minDepth)) // The node takes the max between minDepth and the max time over all constraints.
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
                else
                    this.closedListHits++;
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
    public class CBS_GlobalConflicts : CBS_LocalConflicts
    {
        int[][] globalConflictsCounter;

        public CBS_GlobalConflicts(ICbsSolver solver, int maxThreshold=-1, int currentThreshold=-1)
            : base(solver, maxThreshold, currentThreshold)
        {
            if (currentThreshold < maxThreshold) // FIXME: base's this.solver allocated for no reason
            {
                this.solver = new CBS_GlobalConflicts(solver, maxThreshold, currentThreshold + 1);
            }
        }

        public CBS_GlobalConflicts(ICbsSolver solver) : base(solver) { }

        /// <summary>
        /// Assumes agent nums start from 0 and are consecutive.
        /// </summary>
        /// <param name="problemInstance"></param>
        /// <param name="runner"></param>
        public override void Setup(ProblemInstance problemInstance, Run runner)
        {
            this.globalConflictsCounter = new int[problemInstance.m_vAgents.Length][];
            for (int i = 0; i < globalConflictsCounter.Length; i++)
            {
                this.globalConflictsCounter[i] = new int[i];
                for (int j = 0; j < i; j++)
                {
                    this.globalConflictsCounter[i][j] = 0;
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
                return "Basic CBS/" + lowLevelSolver;
            return "MA-CBS Global(" + mergeThreshold + ")(" + maxMergeThreshold + ")/" + lowLevelSolver;
        }
    }

   
}
